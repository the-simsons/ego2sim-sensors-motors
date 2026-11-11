"""Version 1 bounded ASCII protocol; all state is owned by the executor thread."""
from collections import deque
import math
import secrets

SENSORS = ('flex', 'light', 'ir', 'tof', 'ultrasonic')

def integer(s, maximum=0xffffffff):
    if not s.isascii() or not s.isdecimal():
        raise ValueError('Expected unsigned integer')
    n = int(s)
    if n > maximum:
        raise ValueError('Integer overflow')
    return n

def number(s, finite=False):
    n = float(s)
    if math.isinf(n) or (finite and not math.isfinite(n)):
        raise ValueError('Nonfinite number')
    return n

class Framer:
    def __init__(self, limit=256):
        self.limit, self.buffer, self.discard = limit, bytearray(), False
        self.errors = 0

    def feed(self, data):
        lines = []
        for c in data:
            if c == 10:
                if not self.discard:
                    try:
                        lines.append(self.buffer.decode('ascii').rstrip('\r'))
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
    p = line.split(',')
    if len(p) < 2 or p[1] != '1':
        raise ValueError('Unsupported protocol')
    kind = p[0]
    if kind == 'B' and len(p) == 2:
        return {'kind': kind}
    if kind == 'H' and len(p) == 6:
        return dict(kind=kind, session=integer(p[2]), ms=integer(p[3]),
                    mode=integer(p[4], 1), watchdog=integer(p[5], 1))
    if kind == 'T' and len(p) == 7 and p[4] in SENSORS:
        status = integer(p[5], 255)
        value = number(p[6], finite=status == 0)
        if status == 0:
            lo, hi = {'flex': (0,1023), 'tof': (0,4000)}.get(p[4], (0, 20))
            if not lo <= value <= hi:
                raise ValueError('Telemetry out of bounds')
        return dict(kind=kind, seq=integer(p[2]), ms=integer(p[3]), sensor=p[4],
                    status=status, value=value if status == 0 else float('nan'))
    if kind == 'A' and len(p) == 6:
        return dict(kind=kind, session=integer(p[2]), id=integer(p[3]),
                    accepted=integer(p[4],1), reason=p[5])
    if kind == 'R' and len(p) == 6:
        return dict(kind=kind, session=integer(p[2]), mode=integer(p[3],1),
                    accepted=integer(p[4],1), reason=p[5])
    if kind == 'S' and len(p) == 18:
        return dict(kind=kind, seq=integer(p[2]), ms=integer(p[3]), motor=integer(p[4],2),
                    mode=integer(p[5],1), enabled=integer(p[6],1), moving=integer(p[7],1),
                    position=number(p[8]), velocity=number(p[9]), operation=integer(p[10],3),
                    target=number(p[11]), pm=integer(p[12],1), vm=integer(p[13],1),
                    last_id=integer(p[14]), session=integer(p[15]), status=p[16],
                    watchdog=integer(p[17],1))
    raise ValueError('Malformed record')

def validate_command(motor, operation, target, limits):
    if motor not in (0,1,2) or operation not in (0,1,2,3) or not math.isfinite(target):
        raise ValueError('Invalid command fields')
    if operation == 3:
        return
    if operation not in ((0,), (1,2), (1,))[motor]:
        raise ValueError('Unsupported motor operation')
    if motor == 0:
        valid = limits['servo_min'] <= target <= limits['servo_max']
    else:
        valid = abs(target) <= limits['velocity_max' if operation == 2 else 'move_max']
    if not valid:
        raise ValueError('Command exceeds configured limits')

class Session:
    def __init__(self, limits, ack_timeout=1.0):
        self.limits, self.ack_timeout = limits, ack_timeout
        self.pending, self.outcomes = {}, deque(maxlen=64)
        self.high_id = -1
        self.reset()

    def reset(self):
        for cid in self.pending:
            self.outcomes.append((cid, 'unknown: connection/reset; not retried'))
        self.pending.clear()
        self.token = secrets.randbelow(0xfffffffe)+1
        self.ready, self.mode, self.seq = False, 0, None
        self.acquisition_ms = {}
        # Keep high-water mark across reconnects: GUI must never replay an ID.
        return f'X,1,{self.token}\n'

    def receive(self, record):
        k = record['kind']
        if k == 'B':
            return self.reset()
        if k == 'H' and record['session'] == self.token:
            self.ready, self.mode = True, record['mode']
        if not self.ready:
            return None
        if k in ('A','S','R') and record['session'] != self.token:
            return None
        if k == 'A':
            cid = record['id']
            if cid in self.pending:
                self.pending.pop(cid)
                self.outcomes.append((cid, ('accepted:' if record['accepted'] else 'rejected:')+record['reason']))
        if k in ('T','S'):
            seq = record['seq']
            if self.seq is not None and not 0 < (seq-self.seq) % 2**32 < 2**31:
                raise ValueError('Duplicate/out-of-order sequence; awaiting boot handshake')
            self.seq = seq
            self.acquisition_ms[record.get('sensor', 'motor'+str(record.get('motor')))] = record['ms']
        return None

    def command(self, cid, motor, operation, target, now):
        if not self.ready:
            raise ValueError('Firmware handshake not complete')
        if not 0 <= cid <= 0xffffffff or cid <= self.high_id:
            raise ValueError('Duplicate/out-of-order command ID')
        validate_command(motor,operation,target,self.limits)
        if self.mode != 0 and operation != 3:
            raise ValueError('GUI command forbidden in SENSOR mode')
        if len(self.pending) >= 64:
            raise ValueError('Too many pending commands')
        self.high_id = cid
        self.pending[cid] = now
        return f'C,1,{self.token},{cid},{motor},{operation},{target:.9g}\n'

    def expire(self, now):
        for cid, sent in list(self.pending.items()):
            if now-sent > self.ack_timeout:
                self.pending.pop(cid)
                self.outcomes.append((cid, 'unknown: ACK timeout; not retried'))
