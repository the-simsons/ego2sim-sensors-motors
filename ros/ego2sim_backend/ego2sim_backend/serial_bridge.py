"""ROS adapter for the unchanged main/main.ino serial protocol."""
import math
import queue
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Illuminance, Range
from std_msgs.msg import Bool
from ego2sim_interfaces.msg import ScalarStamped, MotorCommand, MotorState, ControlMode
from .protocol import Framer, Session, FIRMWARE_LIMITS, Outcome, parse, NAN
from .telemetry import SENSOR_TOPICS, RAW_TOPICS, sensor_values
from .transport import MockTransport, SerialTransport


def qos():
    return QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def range_message(stamp,frame,value,radiation,minimum,maximum,fov,allow_inf=False):
    msg = Range()
    msg.header.stamp,msg.header.frame_id = stamp,frame
    msg.radiation_type = radiation
    msg.min_range,msg.max_range,msg.field_of_view = float(minimum),float(maximum),float(fov)
    valid = (math.isfinite(value) and minimum <= value <= maximum) or (allow_inf and math.isinf(value))
    msg.range = float(value) if valid else NAN
    return msg


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        def param(name,default): return self.declare_parameter(name,default).value
        self.mock = param('mock',False)
        self.stale = float(param('stale_timeout_s',1.))
        ack_timeout = float(param('ack_timeout_s',2.))
        if not all(math.isfinite(v) and v>0 for v in (self.stale,ack_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        limits = {k:float(param(k,v)) for k,v in FIRMWARE_LIMITS.items()}
        if not all(math.isfinite(v) for v in limits.values()) or not 0 <= limits['servo_min'] <= limits['servo_max'] <= math.pi or min(limits['move_max'],limits['velocity_max']) <= 0:
            raise ValueError('Invalid motor limits')
        self.processed = param('publish_firmware_processed',True)
        self.calibration = {s:param(s+'_calibration_verified',False) for s in ('flex','light','ir')}
        self.tof_min,self.tof_max = param('tof_min_m',.04),param('tof_max_m',1.3)
        self.tof_fov = param('tof_fov_rad',math.radians(27.))
        self.ir_fov,self.sonar_fov = param('ir_fov_rad',NAN),param('sonar_fov_rad',NAN)
        if not 0 <= self.tof_min < self.tof_max <= 4.:
            raise ValueError('Invalid ToF metadata')
        types = {'scalar':ScalarStamped,'lux':Illuminance,'ir':Range,'tof':Range,'sonar':Range}
        self.sensor_pubs = {topic:self.create_publisher(types[spec[0]],topic,qos())
            for topic,spec in SENSOR_TOPICS.items()
            if self.processed or topic in RAW_TOPICS or spec[0]=='tof'}
        self.motor_pubs = [self.create_publisher(MotorState,f'motors/{name}/state',qos())
                           for name in ('servo','dc','stepper')]
        self.diag = self.create_publisher(DiagnosticArray,'diagnostics',qos())
        self.create_subscription(MotorCommand,'motors/command',self.command,qos())
        self.create_subscription(ControlMode,'control/mode_command',self.mode_command,qos())
        # Kill switch: True = all motors killed. State comes only from firmware KS lines.
        self.kill_pub = self.create_publisher(Bool,'control/kill_state',qos())
        self.create_subscription(Bool,'control/kill_command',self.kill_command,qos())
        self.session = Session(limits,ack_timeout,self.stale)
        self.transport = MockTransport(limits) if self.mock else SerialTransport(param('port','/dev/ttyACM0'))
        self.framer,self.epoch = Framer(),None
        self.motor_cache = {}
        self.sensor_seen = self.last_diag = 0.
        self.last_diag = -math.inf
        self.errors,self.restarts = 0,0
        self.last_error,self.mode_request = '','none'
        self.firmware_diags = {}
        self.transport_restart = False
        self.create_timer(.02,self.tick)

    def send(self,line):
        if self.epoch is None:
            raise ValueError('Serial disconnected')
        try:
            self.transport.send(self.epoch,line)
        except queue.Full:
            raise ValueError('TX queue full; not sent; no automatic retry') from None

    def command_result(self,outcome):
        d = DiagnosticStatus()
        d.name,d.hardware_id = 'ego2sim/command','mock' if self.mock else 'arduino'
        d.level = bytes([0 if outcome.result=='accepted' else 2 if outcome.result=='rejected' else 1])
        d.message = f'Command #{outcome.command_id} {outcome.result}: {outcome.detail}'
        d.values = [KeyValue(key=k,value=str(v)) for k,v in dict(command_id=outcome.command_id,
                    motor=outcome.motor,result=outcome.result).items()]
        self.publish_diagnostics([d])

    def drain_outcomes(self):
        while self.session.outcomes:
            self.command_result(self.session.outcomes.popleft())

    def command(self,msg):
        try:
            self.send(self.session.command(msg.command_id,msg.motor,msg.operation,msg.target,time.monotonic()))
        except ValueError as exc:
            # A worker-queue failure did not reach the UART. Remove pending acceptance.
            # Do not discard a prior pending command when a duplicate ID is rejected.
            pending = self.session.pending.get(msg.command_id)
            if pending and str(exc).startswith('TX queue full'):
                self.session.pending.pop(msg.command_id)
            self.last_error = str(exc)
            self.command_result(Outcome(msg.command_id,msg.motor,'rejected',str(exc)))

    def mode_command(self,msg):
        try:
            now = time.monotonic()
            if msg.mode not in (0,1) or not self.session.ready or not self.session.motor_seen or now-max(self.session.motor_seen.values()) > self.stale:
                raise ValueError('Mode request requires fresh motor feedback')
            self.send(f'M,{msg.mode}\n')
            self.mode_request = f'requested {msg.mode}; awaiting MS feedback'
        except ValueError as exc:
            self.last_error = str(exc)

    def kill_command(self,msg):
        try:
            self.send(self.session.kill_command(msg.data))
            self.get_logger().warn('Kill switch %s requested from ROS' % ('ENGAGE' if msg.data else 'RELEASE'))
        except ValueError as exc:
            self.last_error = str(exc)
            self.get_logger().error(f'Kill switch request not sent: {exc}')

    def publish_sensor(self,topic,value,stamp):
        kind,field,_ = SENSOR_TOPICS[topic]
        frame = topic.split('/')[1]+'_link'
        if kind == 'scalar':
            msg = ScalarStamped(); msg.header.stamp,msg.header.frame_id = stamp,frame
            msg.valid,msg.value = math.isfinite(value),float(value) if math.isfinite(value) else NAN
        elif kind == 'lux':
            msg = Illuminance(); msg.header.stamp,msg.header.frame_id = stamp,frame
            msg.illuminance,msg.variance = float(value),0.
        else:
            lo,hi,fov = {'ir':(.2,1.5,self.ir_fov),'tof':(self.tof_min,self.tof_max,self.tof_fov),
                         'sonar':(.1524,6.4516,self.sonar_fov)}[kind]
            msg = range_message(stamp,frame,value,Range.ULTRASOUND if kind=='sonar' else Range.INFRARED,
                                lo,hi,fov,allow_inf=kind=='ir')
        self.sensor_pubs[topic].publish(msg)

    def unavailable_motor(self,i):
        msg = MotorState()
        if i in self.motor_cache:
            msg.header = self.motor_cache[i].header
            msg.control_mode = self.motor_cache[i].control_mode
            msg.last_command_id = self.motor_cache[i].last_command_id
        else:
            msg.control_mode = self.session.mode if self.session.mode is not None else 0
        msg.header.frame_id = ('servo','dc','stepper')[i]+'_link'
        msg.motor,msg.operation = i,MotorCommand.STOP
        msg.position_rad = msg.velocity_rad_s = msg.target = NAN
        msg.status = 'UNAVAILABLE: no fresh firmware feedback; physical state unknown'
        self.motor_pubs[i].publish(msg)

    def record(self,r,stamp,now):
        restarted = self.session.receive(r,now)
        if restarted:
            if not self.transport_restart: self.restarts += 1
            self.sensor_seen = 0.
            self.motor_cache.clear(); self.firmware_diags.clear()
        self.transport_restart = False
        if r['kind']=='S':
            self.sensor_seen = now
            for topic,value in sensor_values(r,self.calibration).items():
                if topic in self.sensor_pubs: self.publish_sensor(topic,value,stamp)
        elif r['kind']=='MS':
            msg = MotorState()
            msg.header.stamp,msg.header.frame_id = stamp,('servo','dc','stepper')[r['motor']]+'_link'
            msg.motor,msg.control_mode,msg.operation = r['motor'],r['mode'],r['operation']
            msg.enabled,msg.moving = bool(r['enabled']),bool(r['moving'])
            msg.position_rad,msg.velocity_rad_s,msg.target = r['position'],r['velocity'],r['target']
            msg.position_measured,msg.velocity_measured = bool(r['pm']),bool(r['vm'])
            msg.last_command_id,msg.status = r['last_id'],r['status']
            self.motor_cache[r['motor']] = msg
            self.motor_pubs[r['motor']].publish(msg)
            self.mode_request = f'actual {r["mode"]} from firmware MS'
        elif r['kind']=='KS':
            self.kill_pub.publish(Bool(data=r['killed']))
        elif r['kind']=='D':
            d = DiagnosticStatus()
            d.name,d.hardware_id,d.level,d.message = 'firmware/'+r['name'],'arduino',bytes([r['level']]),r['message']
            # Bound distinct firmware diagnostic names.
            if d.name not in self.firmware_diags and len(self.firmware_diags)>=32:
                self.firmware_diags.pop(next(iter(self.firmware_diags)))
            self.firmware_diags[d.name] = d
            self.publish_diagnostics([d])
        self.drain_outcomes()

    def publish_diagnostics(self,statuses):
        msg = DiagnosticArray(); msg.header.stamp=self.get_clock().now().to_msg(); msg.status=statuses
        self.diag.publish(msg)

    def tick(self):
        now = time.monotonic()
        for event,epoch,data in self.transport.poll():
            if event in ('connected','restart'):
                self.epoch = epoch
                self.session.reset(connected=True,reason='firmware reset' if event=='restart' else 'connection opened')
                self.framer = Framer()
                self.sensor_seen = 0.
                self.motor_cache.clear(); self.firmware_diags.clear()
                self.transport_restart = event=='restart'
                if event=='restart': self.restarts += 1
                self.drain_outcomes()
            elif event=='disconnected' and epoch==self.epoch:
                self.epoch = None
                self.session.reset(reason='serial disconnected')
                self.sensor_seen = 0.
                self.last_error = str(data)
                self.drain_outcomes()
            elif event=='data' and epoch==self.epoch:
                for line in self.framer.feed(data):
                    try:
                        self.record(parse(line),self.get_clock().now().to_msg(),now)
                    except ValueError as exc:
                        self.errors += 1; self.last_error = str(exc)
        self.session.expire(now)
        self.drain_outcomes()
        if now-self.last_diag>=.5:
            self.last_diag = now
            stamp = self.get_clock().now().to_msg()
            sensors_stale = not self.sensor_seen or now-self.sensor_seen>self.stale
            if sensors_stale:
                for topic in self.sensor_pubs: self.publish_sensor(topic,NAN,stamp)
            stale_motors = [i for i in range(3) if now-self.session.motor_seen.get(i,-math.inf)>self.stale]
            for i in stale_motors: self.unavailable_motor(i)
            d = DiagnosticStatus(); d.name='ego2sim/serial_bridge'; d.hardware_id='mock' if self.mock else 'arduino'
            d.level = bytes([2 if self.epoch is None or stale_motors or sensors_stale else 0])
            d.message = 'Serial disconnected' if self.epoch is None else 'Stale telemetry' if stale_motors or sensors_stale else 'Firmware telemetry active'
            details = dict(protocol='Arduino S/MS/KS/D/I; C/M/X commands',last_error=self.last_error,
                parse_errors=self.errors,framing_errors=self.framer.errors+getattr(self.transport,'framing_errors',0),
                restarts=self.restarts,mode=self.session.mode,killed=self.session.killed,mode_request=self.mode_request,
                pending=len(self.session.pending),stale_motors=stale_motors,
                acquisition_ms=self.session.acquisition_ms,rx_dropped=getattr(self.transport,'dropped',0))
            d.values=[KeyValue(key=k,value=str(v)) for k,v in details.items()]
            cal = DiagnosticStatus(); cal.name='ego2sim/calibration'; cal.hardware_id=d.hardware_id
            missing=[s for s,verified in self.calibration.items() if not verified]
            cal.level=bytes([1 if missing and self.processed else 0])
            cal.message=('Unverified firmware calibration: '+','.join(missing)+'; physical values suppressed') if missing and self.processed else 'Firmware calibration marked verified' if self.processed else 'Host processing selected'
            kill = DiagnosticStatus(); kill.name='ego2sim/killswitch'; kill.hardware_id=d.hardware_id
            killed = self.session.killed
            kill.level = bytes([0 if killed is False else 1 if killed else 2])
            kill.message = ('Motors enabled' if killed is False else
                            f'KILLED (last changed by {self.session.kill_source})' if killed else
                            'Kill switch state unknown (no KS from firmware)')
            link = DiagnosticStatus(); link.name='ego2sim/firmware_limits'; link.hardware_id=d.hardware_id
            link.level=bytes([1]); link.message='Firmware has no link-loss watchdog; process/USB loss cannot guarantee motor stop'
            link.values=[KeyValue(key='retry_policy',value='No command replay; MS confirms only last accepted ID')]
            self.publish_diagnostics([d,kill,cal,link,*self.firmware_diags.values()])

    def destroy_node(self):
        self.transport.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=SerialBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
