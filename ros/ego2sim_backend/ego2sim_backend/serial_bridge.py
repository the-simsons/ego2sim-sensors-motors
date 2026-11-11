"""USB serial is owned exclusively by this node's transport worker."""
import math
import queue
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Range
from ego2sim_interfaces.msg import ScalarStamped, MotorCommand, MotorState, ControlMode
from .protocol import Framer, Session, SENSORS, parse
from .transport import MockTransport, SerialTransport

NAN = float('nan')

def qos():
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)

def range_message(stamp, frame, value, radiation, minimum, maximum, fov):
    msg = Range()
    msg.header.stamp, msg.header.frame_id = stamp, frame
    msg.radiation_type = radiation
    msg.min_range, msg.max_range, msg.field_of_view = float(minimum), float(maximum), float(fov)
    msg.range = float(value) if math.isfinite(value) and minimum <= value <= maximum else NAN
    return msg

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        def param(name, default):
            return self.declare_parameter(name,default).value
        self.mock = param('mock',False)
        port = param('port','/dev/ttyACM0')
        self.stale = param('stale_timeout_s',1.0)
        self.ack_timeout = param('ack_timeout_s',1.0)
        if self.stale <= 0 or self.ack_timeout <= 0:
            raise ValueError('Timeouts must be positive')
        limits = {k:float(param(k,v)) for k,v in dict(servo_min=0.,servo_max=math.pi,
                  move_max=2*math.pi,velocity_max=10.).items()}
        if not all(math.isfinite(v) for v in limits.values()) or limits['servo_min'] > limits['servo_max'] or limits['move_max'] <= 0 or limits['velocity_max'] <= 0:
            raise ValueError('Invalid motor limits')
        self.tof_min = param('tof_min_m',.04)
        self.tof_max = param('tof_max_m',1.3)
        self.tof_fov = param('tof_fov_rad',math.radians(27.))
        if not 0 <= self.tof_min < self.tof_max <= 4.:
            raise ValueError('Invalid ToF range metadata')
        self.raw = {s:self.create_publisher(ScalarStamped,'sensors/'+{
            'flex':'flex/raw','ir':'ir/raw_voltage','light':'light/raw_voltage',
            'ultrasonic':'ultrasonic/raw_voltage'}[s],qos()) for s in SENSORS if s != 'tof'}
        self.tof_pub = self.create_publisher(Range,'sensors/tof/range',qos())
        self.motor_pubs = [self.create_publisher(MotorState,f'motors/{s}/state',qos())
                           for s in ('servo','dc','stepper')]
        self.diag = self.create_publisher(DiagnosticArray,'diagnostics',qos())
        self.create_subscription(MotorCommand,'motors/command',self.command,qos())
        self.create_subscription(ControlMode,'control/mode_command',self.mode_command,qos())
        self.session = Session(limits,self.ack_timeout)
        self.transport = MockTransport(limits) if self.mock else SerialTransport(port)
        self.framer = Framer()
        self.epoch = None
        self.seen, self.motor_seen, self.sensor_status = {}, {}, {}
        self.last_diag = self.last_ping = self.last_handshake = 0.
        self.last_rx = 0.
        self.errors, self.restarts = 0, 0
        self.last_error = 'Starting'
        self.watchdog = 0
        self.mode_result = 'No mode request'
        self.create_timer(.02,self.tick)

    def send(self,line):
        if self.epoch is None:
            raise ValueError('Serial disconnected')
        try:
            self.transport.send(self.epoch,line)
        except queue.Full:
            self.last_error = 'TX queue full; command outcome unknown; no retry'
            raise ValueError(self.last_error)

    def command(self,msg):
        try:
            line = self.session.command(msg.command_id,msg.motor,msg.operation,msg.target,time.monotonic())
            self.send(line)
        except ValueError as exc:
            self.last_error = str(exc)
            self.get_logger().warning(f'Command {msg.command_id}: {exc}')

    def mode_command(self,msg):
        try:
            if msg.mode not in (0,1) or not self.session.ready:
                raise ValueError('Invalid mode or firmware not ready')
            self.send(f'M,1,{self.session.token},{msg.mode}\n')
        except ValueError as exc:
            self.last_error = str(exc)

    def raw_sensor(self,s,value,stamp):
        if s == 'tof':
            self.tof_pub.publish(range_message(stamp,'tof_link',value/1000.,Range.INFRARED,
                                               self.tof_min,self.tof_max,self.tof_fov))
        else:
            m = ScalarStamped()
            m.header.stamp, m.header.frame_id = stamp,s+'_link'
            m.valid, m.value = math.isfinite(value),float(value)
            self.raw[s].publish(m)

    def unavailable_motor(self,i,stamp):
        m = MotorState()
        m.header.stamp, m.header.frame_id = stamp,('servo','dc','stepper')[i]+'_link'
        m.motor, m.control_mode = i,self.session.mode
        m.position_rad = m.velocity_rad_s = m.target = NAN
        m.status = 'UNAVAILABLE: no current firmware feedback'
        m.operation = MotorCommand.STOP
        self.motor_pubs[i].publish(m)

    def record(self,r,stamp,now):
        k = r['kind']
        if k == 'B':
            self.restarts += 1
            self.seen.clear()
            self.motor_seen.clear()
            self.sensor_status.clear()
        reply = self.session.receive(r)
        if reply:
            self.send(reply)
        if not self.session.ready:
            return
        if k in ('S','A','H','R') and r['session'] != self.session.token:
            return
        if k == 'R':
            self.mode_result = ('accepted:' if r['accepted'] else 'rejected:')+r['reason']
        elif k == 'H':
            self.watchdog = r['watchdog']
        elif k == 'T':
            self.seen[r['sensor']] = now
            self.sensor_status[r['sensor']] = r['status']
            self.raw_sensor(r['sensor'],r['value'],stamp)
        elif k == 'S':
            self.session.mode = r['mode']
            self.watchdog = r['watchdog']
            self.motor_seen[r['motor']] = now
            m = MotorState()
            m.header.stamp = stamp
            m.header.frame_id = ('servo','dc','stepper')[r['motor']]+'_link'
            m.motor,m.control_mode,m.operation = r['motor'],r['mode'],r['operation']
            m.enabled,m.moving = bool(r['enabled']),bool(r['moving'])
            m.position_rad,m.velocity_rad_s,m.target = r['position'],r['velocity'],r['target']
            m.position_measured,m.velocity_measured = bool(r['pm']),bool(r['vm'])
            m.last_command_id,m.status = r['last_id'],r['status']
            self.motor_pubs[r['motor']].publish(m)

    def tick(self):
        try:
            self._tick()
        except ValueError as exc:
            self.last_error = str(exc)
            self.get_logger().warning(str(exc))

    def _tick(self):
        now = time.monotonic()
        for event,epoch,data in self.transport.poll():
            if event == 'connected':
                self.epoch = epoch
                self.framer = Framer()
                self.seen.clear(); self.motor_seen.clear(); self.sensor_status.clear()
                self.send(self.session.reset())
                self.last_handshake = self.last_rx = now
            elif event == 'disconnected' and epoch == self.epoch:
                self.epoch = None
                self.session.reset()
                self.seen.clear(); self.motor_seen.clear(); self.sensor_status.clear()
                self.last_error = str(data)
            elif event == 'data' and epoch == self.epoch:
                for line in self.framer.feed(data):
                    try:
                        self.record(parse(line),self.get_clock().now().to_msg(),now)
                        self.last_rx = now
                    except ValueError as exc:
                        self.errors += 1
                        self.last_error = str(exc)
        if self.session.ready and now-self.last_rx > max(1.5,self.stale):
            # Handles a reset whose B record was lost, or a silent device.
            self.seen.clear(); self.motor_seen.clear(); self.sensor_status.clear()
            self.last_error = 'Link silent; pending commands unknown; fresh session required'
            self.send(self.session.reset())
            self.last_handshake = now
        self.session.expire(now)
        if self.epoch is not None and now-self.last_ping >= .3:
            self.last_ping = now
            try:
                if self.session.ready:
                    self.send(f'P,1,{self.session.token}\n')
                elif now-self.last_handshake >= .5:
                    self.send(f'X,1,{self.session.token}\n')
                    self.last_handshake = now
            except ValueError as exc:
                self.last_error = str(exc)
        if now-self.last_diag >= .5:
            self.last_diag = now
            stamp = self.get_clock().now().to_msg()
            stale = [s for s in SENSORS if now-self.seen.get(s,-math.inf) > self.stale]
            for s in stale:
                self.raw_sensor(s,NAN,stamp)
            for i in range(3):
                if now-self.motor_seen.get(i,-math.inf) > self.stale:
                    self.unavailable_motor(i,stamp)
            invalid = [s for s,status in self.sensor_status.items() if status != 0]
            d = DiagnosticStatus()
            d.name,d.hardware_id = 'ego2sim/serial_bridge','mock' if self.mock else 'arduino'
            d.level = DiagnosticStatus.ERROR if not self.session.ready or now-self.last_rx > self.stale or self.watchdog else DiagnosticStatus.WARN if stale or invalid else DiagnosticStatus.OK
            d.message = 'Watchdog expired' if self.watchdog else ('Stale: '+','.join(stale) if stale else 'Invalid: '+','.join(invalid) if invalid else 'Telemetry active')
            details = dict(last_error=self.last_error,parse_errors=self.errors,
                framing_errors=self.framer.errors,restarts=self.restarts,mode=self.session.mode,
                sensor_status=self.sensor_status,mode_result=self.mode_result,pending=len(self.session.pending),outcomes=list(self.session.outcomes),
                acquisition_ms=self.session.acquisition_ms,rx_dropped=getattr(self.transport,'dropped',0),
                motor_support='SIMULATED' if self.mock else 'NOT IMPLEMENTED in main.ino')
            d.values = [KeyValue(key=k,value=str(v)) for k,v in details.items()]
            a = DiagnosticArray(); a.header.stamp=stamp; a.status=[d]
            self.diag.publish(a)

    def destroy_node(self):
        self.transport.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
