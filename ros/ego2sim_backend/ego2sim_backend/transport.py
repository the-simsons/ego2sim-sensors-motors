"""Bounded serial worker and deterministic simulated firmware. No ROS imports."""
import math
import queue
import threading
import time
from .protocol import validate_command

class SerialTransport:
    def __init__(self, port, baud=115200):
        import serial
        self.serial = serial
        self.port, self.baud = port, baud
        self.rx, self.tx = queue.Queue(256), queue.Queue(64)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.dropped = 0
        self.thread.start()

    def _event(self, event):
        try:
            self.rx.put_nowait(event)
        except queue.Full:
            self.dropped += 1
            # Losing protocol bytes makes state unsafe: force reconnect.
            raise OSError('RX queue overflow')

    def send(self, epoch, line):
        self.tx.put_nowait((epoch, line.encode('ascii')))

    def poll(self):
        events = []
        for _ in range(128):
            try:
                events.append(self.rx.get_nowait())
            except queue.Empty:
                break
        return events

    def _run(self):
        serial = self.serial
        epoch = 0
        while not self.stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.1) as port:
                    epoch += 1
                    port.reset_input_buffer()
                    self._event(('connected', epoch, b''))
                    while not self.stop.is_set():
                        for _ in range(8):
                            try:
                                target, data = self.tx.get_nowait()
                            except queue.Empty:
                                break
                            if target == epoch:
                                if port.write(data) != len(data):
                                    raise OSError('Partial serial write; outcome unknown')
                        data = port.read(1024)
                        if data:
                            self._event(('data', epoch, data))
            except (serial.SerialException, OSError) as exc:
                try:
                    self._event(('disconnected', epoch, str(exc)))
                except OSError:
                    pass
                self.stop.wait(1.0)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.0)

class MockTransport:
    """Uses the real wire parser. Motion is simulated, never encoder-measured."""
    def __init__(self, limits):
        self.limits = limits
        self.events = [('connected',1,b'')]
        self.token = None
        self.start = time.monotonic()
        self.last = self.start
        self.heartbeat = self.start
        self.seq, self.mode, self.high_id = 0, 0, -1
        self.watchdog = 0
        self.motors = [dict(pos=math.pi/2 if i == 0 else 0., vel=0., remaining=0.,
                            op=3, target=0., last_id=0, enabled=False) for i in range(3)]

    def emit(self, line):
        if len(self.events) >= 256:
            raise ValueError('Mock RX queue full')
        self.events.append(('data',1,(line+'\n').encode('ascii')))

    def hello(self):
        self.emit(f'H,1,{self.token},{self.ms()},{self.mode},{self.watchdog}')

    def ms(self):
        return int((time.monotonic()-self.start)*1000) % 2**32

    def send(self, epoch, line):
        p = line.strip().split(',')
        if p[:2] == ['X','1']:
            if int(p[2]) != self.token:
                self.high_id = -1
                for i,m in enumerate(self.motors):
                    m.update(vel=0.,remaining=0.,enabled=m['enabled'] if i == 0 else False)
            self.token = int(p[2])
            self.heartbeat = time.monotonic()
            self.watchdog = 0
            self.hello()
            return
        if self.token is None or int(p[2]) != self.token:
            return
        if p[0] == 'P':
            self.heartbeat = time.monotonic()
            self.watchdog = 0
            self.hello()
        elif p[0] == 'M':
            if int(p[3]) in (0,1):
                self.mode = int(p[3])
                for i,m in enumerate(self.motors):
                    m.update(vel=0.,remaining=0.,enabled=m['enabled'] if i == 0 else False)
            self.emit(f'R,1,{self.token},{p[3]},1,OK')
            self.hello()
        elif p[0] == 'C':
            cid, motor, op = map(int,p[3:6])
            target = float(p[6])
            reason = 'OK'
            try:
                validate_command(motor,op,target,self.limits)
                if cid <= self.high_id:
                    raise ValueError('DUPLICATE')
                if self.mode and op != 3:
                    raise ValueError('SENSOR_MODE')
                m = self.motors[motor]
                if op in (1,2) and (m['remaining'] or (op == 1 and m['vel'])):
                    raise ValueError('BUSY')
                m.update(op=op,target=target,last_id=cid,
                         enabled=(m['enabled'] if motor == 0 else False) if op == 3 else True)
                if op == 3:
                    m.update(vel=0.,remaining=0.)
                elif op == 0:
                    m.update(pos=target,vel=0.,remaining=0.)
                elif op == 1:
                    m.update(remaining=target,vel=0.)
                else:
                    m.update(vel=target,remaining=0.)
                self.high_id = cid
            except ValueError as exc:
                reason = str(exc)
            self.emit(f'A,1,{self.token},{cid},{int(reason == "OK")},{reason}')

    def poll(self):
        now = time.monotonic()
        dt = now-self.last
        if dt >= .1 and self.token is not None:
            self.last = now
            if now-self.heartbeat > 1.5:
                self.watchdog = 1
                for i,m in enumerate(self.motors):
                    m.update(vel=0.,remaining=0.,enabled=m['enabled'] if i == 0 else False)
            for sensor,value in [('flex',500+100*math.sin(now)),('ir',1.2),
                                 ('light',1.0),('tof',650.),('ultrasonic',5*24/512)]:
                self.seq = (self.seq+1) % 2**32
                self.emit(f'T,1,{self.seq},{self.ms()},{sensor},0,{value:.6f}')
            for i,m in enumerate(self.motors):
                step = math.copysign(min(abs(m['remaining']),dt),m['remaining'])
                v = step/dt if m['remaining'] else m['vel']
                m['pos'] += step + m['vel']*dt
                m['remaining'] -= step
                self.seq = (self.seq+1) % 2**32
                self.emit(f'S,1,{self.seq},{self.ms()},{i},{self.mode},{int(m["enabled"])},'
                          f'{int(v != 0)},{m["pos"]},{v},{m["op"]},{m["target"]},0,0,'
                          f'{m["last_id"]},{self.token},SIMULATED,{self.watchdog}')
        events, self.events = self.events, []
        return events

    def close(self):
        pass
