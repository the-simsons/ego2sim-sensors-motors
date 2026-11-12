"""Bounded USB worker and a simulated main/main.ino wire peer."""
import math
import queue
import threading
import time
from .protocol import BOOT, Framer, UptimeTracker, parse, validate_command


class SerialTransport:
    def __init__(self, port, baud=115200):
        import serial
        self.serial, self.port, self.baud = serial, port, baud
        self.rx, self.tx = queue.Queue(256), queue.Queue(64)
        self.stop = threading.Event()
        self.dropped = self.framing_errors = 0
        self.thread = threading.Thread(target=self._run,daemon=True)
        self.thread.start()

    def _event(self,event):
        try:
            self.rx.put_nowait(event)
        except queue.Full:
            self.dropped += 1
            raise OSError('RX queue overflow; reconnect required')

    def send(self,epoch,line):
        self.tx.put_nowait((epoch,line.encode('ascii')))

    def poll(self):
        result = []
        for _ in range(128):
            try:
                result.append(self.rx.get_nowait())
            except queue.Empty:
                break
        return result

    def _run(self):
        serial,epoch = self.serial,0
        while not self.stop.is_set():
            try:
                with serial.Serial(self.port,self.baud,timeout=.02,write_timeout=.1) as port:
                    epoch += 1
                    framer,uptime = Framer(),UptimeTracker()
                    self._event(('connected',epoch,''))
                    while not self.stop.is_set():
                        # Inspect incoming resets before draining queued commands.
                        data = port.read(1024)
                        before = framer.errors
                        for line in framer.feed(data):
                            reboot = False
                            try:
                                r = parse(line)
                                reboot = r['kind'] == 'I' and r['message'] == BOOT
                                if r['kind'] in ('S','MS'):
                                    reboot = uptime.update(r['ms']) or reboot
                            except ValueError:
                                pass  # Executor reports malformed records.
                            if reboot:
                                epoch += 1
                                uptime = UptimeTracker()
                                if r['kind'] in ('S','MS'):
                                    uptime.update(r['ms'])
                                self._event(('restart',epoch,'boot/uptime reset'))
                            self._event(('data',epoch,(line+'\n').encode('ascii')))
                        self.framing_errors += framer.errors-before
                        for _ in range(8):
                            try:
                                target,payload = self.tx.get_nowait()
                            except queue.Empty:
                                break
                            if target == epoch and port.write(payload) != len(payload):
                                raise OSError('Partial write; outcome unknown; never retried')
            except (serial.SerialException,OSError) as exc:
                try:
                    self._event(('disconnected',epoch,str(exc)))
                except OSError:
                    pass
                self.stop.wait(1.)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)


class MockTransport:
    """Simulated sensors/motion; same C/M/X inputs and S/MS/KS/D/I outputs as firmware.

    No hardware watchdog, version handshake or command ACK is invented.
    """
    def __init__(self, limits=None):
        self.limits = limits
        self.events = [('connected',1,'')]
        self.start = self.last = time.monotonic()
        self.epoch,self.mode = 1,0
        self.killed,self.kill_source = False,'boot'
        self.seen_ids = []
        self.motors = [dict(pos=math.pi/2 if i==0 else 0.,vel=0.,target=math.pi/2 if i==0 else 0.,
            op=3,last_id=0,held=False,status='SIMULATED idle') for i in range(3)]
        self.emit('I,'+BOOT)

    def emit(self,line):
        if len(self.events) >= 256:
            raise ValueError('Mock RX queue full')
        self.events.append(('data',self.epoch,(line+'\n').encode('ascii')))

    def ms(self):
        return int((time.monotonic()-self.start)*1000) % 2**32

    def stop_motor(self,i):
        m = self.motors[i]
        m.update(op=3,vel=0.,target=m['pos'])

    def send(self,epoch,line):
        if epoch != self.epoch:
            return
        p = line.strip().split(',')
        if p[0] == 'M' and len(p) == 2:
            mode = int(p[1])
            if mode in (0,1) and mode != self.mode:
                self.mode = mode
                for i,m in enumerate(self.motors):
                    self.stop_motor(i)
                    m.update(held=False,status='SIMULATED mode changed')
                self.emit('D,0,control_mode,'+('SENSOR' if mode else 'GUI'))
        elif p[0] == 'X' and len(p) == 2:
            killed = p[1] != '0'
            if killed != self.killed:
                self.killed,self.kill_source = killed,'gui'
                for i,m in enumerate(self.motors):
                    self.stop_motor(i)
                    m.update(held=False,status='SIMULATED killed' if killed else 'SIMULATED kill switch released')
                self.emit('D,1,killswitch,ENGAGED by GUI' if killed else 'D,0,killswitch,released by GUI')
            self.emit(f'KS,{self.ms()},{int(self.killed)},{self.kill_source}')
        elif p[0] == 'C' and len(p) == 5:
            cid,motor,op = map(int,p[1:4])
            target = float(p[4])
            if cid in self.seen_ids:
                return
            # The real firmware remembers the last eight IDs, including rejects.
            self.seen_ids = (self.seen_ids+[cid])[-8:]
            try:
                validate_command(motor,op,target,self.limits)
                if self.killed and op != 3:
                    raise ValueError('kill switch engaged')
                if self.mode and op != 3:
                    raise ValueError('motion commands need GUI mode')
                m = self.motors[motor]
                m.update(op=op,last_id=cid,status='SIMULATED accepted')
                if op == 3:
                    self.stop_motor(motor)
                    m['held'] = self.mode == 1
                    m['status'] = 'SIMULATED stopped, sensor control paused' if self.mode else 'SIMULATED stopped by GUI'
                elif op == 0:
                    m['target'] = math.floor(math.degrees(target)+.5)*math.pi/180
                elif op == 1:
                    delta = target
                    if motor == 2:
                        steps = math.copysign(math.floor(abs(target)/(math.tau/1600)+.5),target)
                        delta = steps*math.tau/1600
                    m['target'] = m['pos']+delta
                else:
                    m['target'] = target
            except ValueError as exc:
                self.emit(f'D,2,motor_command,Rejected command #{cid}: {exc}')
        elif p[0] == 'P':
            self.emit('I,enc_count=0 counts_per_rev=360.0')

    def poll(self):
        now = time.monotonic()
        dt = now-self.last
        if dt >= .05:
            self.last = now
            t = now-self.start
            flex = .5+.3*math.sin(t)
            ir,tof = .7+.4*math.sin(t*.5),.6+.3*math.sin(t*.4)
            self.emit(f'S,{self.ms()},500,{flex:.4f},{flex:.4f},1.000,200.0,200.0,1.200,'
                      f'{ir:.4f},{ir:.4f},{tof:.4f},3528,0.6096')
            if self.mode == 1 and not self.killed:
                servo,dc,step = self.motors
                if not servo['held']:
                    goal = min(math.pi,max(0.,2*flex))
                    if abs(goal-servo['target']) >= math.radians(2): servo['target'] = goal
                    servo['op'] = 0
                if not dc['held']:
                    dc.update(op=2,target=(-50 if ir<.5 else 50 if ir>.9 else 0)*math.tau/60)
                if not step['held']:
                    goal = min(1.,max(0.,(tof-.1)/.9))*math.tau
                    if abs(goal-step['target']) > math.radians(5):
                        step['target'] = round(goal/(math.tau/1600))*(math.tau/1600)
                    step['op'] = 1
            for i,m in enumerate(self.motors):
                if m['op'] == 2:
                    m['vel'] = m['target']
                    m['pos'] += m['vel']*dt
                elif m['op'] in (0,1):
                    speed = math.radians(300 if i==0 else 225 if i==2 else 360)
                    delta = math.copysign(min(abs(m['target']-m['pos']),speed*dt),m['target']-m['pos'])
                    m['pos'] += delta
                    m['vel'] = delta/dt
                else:
                    m['vel'] = 0.
                self.emit(f'MS,{self.ms()},{i},{self.mode},{m["op"]},{int(not (m["held"] or self.killed))},'
                          f'{int(abs(m["vel"])>1e-6)},{m["pos"]:.6f},{m["vel"]:.6f},'
                          f'{m["target"]:.6f},0,0,{m["last_id"]},'
                          f'{"SIMULATED killed" if self.killed else m["status"]}')
            self.emit(f'KS,{self.ms()},{int(self.killed)},{self.kill_source}')
        events,self.events = self.events,[]
        return events

    def close(self):
        pass
