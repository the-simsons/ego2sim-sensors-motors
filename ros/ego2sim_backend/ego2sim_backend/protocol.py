"""Wire contract implemented by main/main.ino (unversioned newline-delimited ASCII).

No synthetic handshake, ACK or watchdog is added: acceptance comes from MS records.
"""
from collections import deque, OrderedDict
from dataclasses import dataclass
import math
import re

NAN = float('nan')
SENSORS = ('flex', 'light', 'ir', 'tof', 'ultrasonic')
SENSOR_FIELDS = ('flex_adc','flex_rad','flex_rad_f','light_v','lux','lux_f',
                 'ir_v','ir_m','ir_m_f','tof_m','sonar_pulse_us','sonar_m')
FIRMWARE_LIMITS = dict(servo_min=0., servo_max=math.pi,
                       move_max=20*math.pi, velocity_max=100*math.tau/60)
BOOT = 'ego2sim firmware ready'
KILL_SOURCES = ('boot', 'button', 'gui')
REJECTION = re.compile(r'^Rejected command #(\d+): (.*)$')


def integer(s, maximum=0xffffffff):
    if not s.isascii() or not s.isdecimal() or int(s) > maximum:
        raise ValueError('Expected bounded unsigned integer')
    return int(s)


def number(s, allow_inf=False):
    n = float(s)
    if math.isinf(n) and not allow_inf:
        raise ValueError('Unexpected infinite measurement')
    return n


class Framer:
    def __init__(self, limit=256):
        self.limit, self.buffer, self.discard = limit, bytearray(), False
        self.errors = 0

    def feed(self, data):
        lines = []
        for c in data:
            if c in (10, 13):
                if not self.discard and self.buffer:
                    try:
                        lines.append(self.buffer.decode('ascii'))
                    except UnicodeDecodeError:
                        self.errors += 1
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                if len(self.buffer) >= self.limit:
                    self.buffer.clear()
                    self.discard = True
                    self.errors += 1
                else:
                    self.buffer.append(c)
        return lines


def parse(line):
    kind = line.split(',', 1)[0]
    if kind == 'S':
        p = line.split(',')
        if len(p) != 14:
            raise ValueError('S needs 14 fields')
        values = {name:number(value, name in ('ir_m','ir_m_f'))
                  for name,value in zip(SENSOR_FIELDS,p[2:])}
        for key,maximum in [('flex_adc',1023),('light_v',5.1),('ir_v',5.1)]:
            v = values[key]
            if math.isfinite(v) and not 0 <= v <= maximum:
                raise ValueError('Raw sensor value out of bounds')
        if math.isfinite(values['sonar_pulse_us']) and values['sonar_pulse_us'] < 0:
            raise ValueError('Negative sonar pulse')
        return dict(kind=kind, ms=integer(p[1]), **values)
    if kind == 'MS':
        # Status text contains commas (e.g. "stopped, sensor control paused").
        p = line.split(',', 13)
        if len(p) != 14 or not p[13]:
            raise ValueError('MS needs 14 fields and a status')
        return dict(kind=kind, ms=integer(p[1]), motor=integer(p[2],2),
                    mode=integer(p[3],1), operation=integer(p[4],3),
                    enabled=integer(p[5],1), moving=integer(p[6],1),
                    position=number(p[7]), velocity=number(p[8]), target=number(p[9]),
                    pm=integer(p[10],1), vm=integer(p[11],1),
                    last_id=integer(p[12]), status=p[13])
    if kind == 'D':
        p = line.split(',',3)
        if len(p) != 4 or not p[2] or not p[3]:
            raise ValueError('Malformed diagnostic')
        return dict(kind=kind, level=integer(p[1],2), name=p[2], message=p[3])
    if kind == 'KS':
        p = line.split(',')
        if len(p) != 4 or p[3] not in KILL_SOURCES:
            raise ValueError('KS needs ms, killed and boot/button/gui source')
        return dict(kind=kind, ms=integer(p[1]), killed=bool(integer(p[2],1)), source=p[3])
    if kind == 'I' and ',' in line:
        return dict(kind=kind, message=line.split(',',1)[1])
    raise ValueError('Unknown firmware record')


class UptimeTracker:
    """Detect backward MCU uptime, permitting uint32 millis rollover and equal stamps."""
    def __init__(self):
        self.last = None

    def update(self, ms):
        restarted = self.last is not None and (ms-self.last) % 2**32 >= 2**31
        self.last = ms
        return restarted


def validate_command(motor, operation, target, limits=None):
    if motor not in (0,1,2) or operation not in (0,1,2,3) or not math.isfinite(target):
        raise ValueError('Invalid command fields')
    if operation == 3:
        return
    if operation not in ((0,), (1,2), (1,))[motor]:
        raise ValueError('Unsupported motor operation')
    effective = dict(FIRMWARE_LIMITS)
    if limits:
        effective['servo_min'] = max(0.,limits['servo_min'])
        for key in ('servo_max','move_max','velocity_max'):
            effective[key] = min(effective[key],limits[key])
    if motor == 0:
        valid = effective['servo_min'] <= target <= effective['servo_max']
    else:
        valid = abs(target) <= effective['velocity_max' if operation == 2 else 'move_max']
    if not valid:
        raise ValueError('Command exceeds firmware/configured limit')


@dataclass
class Outcome:
    command_id: int
    motor: int
    result: str
    detail: str


class Session:
    def __init__(self, limits=None, ack_timeout=2., stale_timeout=1.):
        self.limits = limits or FIRMWARE_LIMITS
        self.ack_timeout, self.stale_timeout = ack_timeout, stale_timeout
        self.pending, self.outcomes = {}, deque(maxlen=64)
        self.issued = OrderedDict()
        self.high_id = 0  # Firmware uses last_command_id=0 to mean no command yet.
        self.connected = False
        self.ready = False
        self.mode = None
        self.killed = None  # None until the firmware reports a KS line
        self.kill_source = None
        self.motor_seen = {}
        self.uptime = UptimeTracker()
        self.acquisition_ms = {}

    def reset(self, connected=False, reason='connection/reset'):
        for cid, pending in self.pending.items():
            pending['result'] = 'unknown'
            self.outcomes.append(Outcome(cid,pending['motor'],'unknown',reason+'; not retried'))
        self.pending.clear()
        self.connected, self.ready, self.mode = connected, False, None
        self.killed = self.kill_source = None
        self.motor_seen.clear()
        self.acquisition_ms.clear()
        self.uptime = UptimeTracker()
        # Retain the ID high-water mark across resets and reconnects.

    def receive(self, record, now):
        k = record['kind']
        restart = k == 'I' and record['message'] == BOOT
        if k in ('S','MS'):
            restart = self.uptime.update(record['ms']) or restart
        if restart:
            self.reset(connected=self.connected,reason='firmware restarted')
            if k in ('S','MS'):
                self.uptime.update(record['ms'])
        if k == 'MS' and self.connected:
            self.ready, self.mode = True, record['mode']
            self.motor_seen[record['motor']] = now
            self.acquisition_ms['motor'+str(record['motor'])] = record['ms']
            cid = record['last_id']
            if cid in self.issued and self.issued[cid]['motor'] == record['motor'] and self.issued[cid]['result'] != 'accepted':
                self.issued[cid]['result'] = 'accepted'
                self.pending.pop(cid,None)
                self.outcomes.append(Outcome(cid,record['motor'],'accepted','confirmed by firmware MS'))
        elif k == 'S':
            self.acquisition_ms['sensors'] = record['ms']
        elif k == 'KS' and self.connected:
            self.killed, self.kill_source = record['killed'], record['source']
        elif k == 'D' and record['name'] == 'motor_command':
            match = REJECTION.fullmatch(record['message'])
            if match:
                cid = integer(match[1])
                pending = self.issued.get(cid)
                if pending and pending['result'] in ('pending','unknown'):
                    self.pending.pop(cid,None)
                    pending['result'] = 'rejected'
                    self.outcomes.append(Outcome(cid,pending['motor'],'rejected',match[2]))
        return restart

    def command(self, cid, motor, operation, target, now):
        if not self.connected:
            raise ValueError('Serial disconnected')
        if not isinstance(cid,int) or not 1 <= cid <= 0xffffffff or cid <= self.high_id:
            raise ValueError('Command ID must be nonzero and strictly increasing; no replay')
        validate_command(motor,operation,target,self.limits)
        if operation != 3:
            if not self.ready or now-self.motor_seen.get(motor,-math.inf) > self.stale_timeout:
                raise ValueError('No fresh firmware feedback for this motor')
            if self.killed:
                raise ValueError('Kill switch engaged; only STOP is accepted')
            if self.mode != 0:
                raise ValueError('Motion commands require reported GUI mode')
        if len(self.pending) >= 64:
            raise ValueError('Too many pending commands')
        line = f'C,{cid},{motor},{operation},{target:.9g}\n'
        if len(line.rstrip('\n')) > 47:
            raise ValueError('Command exceeds firmware line buffer')
        self.high_id = cid
        self.pending[cid] = dict(motor=motor,sent=now,result='pending')
        self.issued[cid] = self.pending[cid]
        while len(self.issued) > 128:
            oldest = next(k for k in self.issued if k not in self.pending)
            self.issued.pop(oldest)
        return line

    def kill_command(self, killed):
        """Kill switch request; the firmware's KS line confirms the actual state."""
        if not self.connected:
            raise ValueError('Serial disconnected')
        return f'X,{int(bool(killed))}\n'

    def expire(self, now):
        for cid,pending in list(self.pending.items()):
            if now-pending['sent'] > self.ack_timeout:
                self.pending.pop(cid)
                pending['result'] = 'unknown'
                self.outcomes.append(Outcome(cid,pending['motor'],'unknown',
                    'no MS confirmation; outcome unknown; not retried'))
        if self.motor_seen and now-max(self.motor_seen.values()) > self.stale_timeout:
            self.ready = False
