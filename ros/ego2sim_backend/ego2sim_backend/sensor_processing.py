"""Optional host calibration/filtering; launch only with firmware publication disabled."""
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Illuminance, Range
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ego2sim_interfaces.msg import ScalarStamped
from .processing import Calibration, LowPass, sonar_pulse, NAN
from .serial_bridge import qos, range_message

class SensorProcessing(Node):
    def __init__(self):
        super().__init__('sensor_processing')
        def param(name,default):
            return self.declare_parameter(name,default).value
        tau = param('filter_tau_s',.2)
        stale = param('filter_stale_s',1.)
        self.sonar_scale = param('sonar_us_per_inch',147.)
        sonar_pulse(NAN,self.sonar_scale)
        self.ir_fov = param('ir_fov_rad',NAN)
        self.sonar_fov = param('sonar_fov_rad',NAN)
        self.cal = {}
        for s,x,y in [('flex','adc','angle_rad'),('light','voltage','lux'),('ir','voltage','distance_m')]:
            # Explicit types permit empty calibration arrays in ROS 2 Humble.
            arrays = [self.declare_parameter(s+'_'+key,Parameter.Type.DOUBLE_ARRAY).value or []
                      for key in (x,y)]
            self.cal[s] = Calibration(*arrays)
        self.diagnostics = self.create_publisher(DiagnosticArray,'diagnostics',qos())
        self.create_timer(1.,self.report_calibration)
        self.filters = {s:LowPass(tau,stale) for s in ('flex','light','ir')}
        types = {'flex':(ScalarStamped,'angle'),'light':(Illuminance,'illuminance'),'ir':(Range,'range')}
        self.pubs,self.filtered = {},{}
        for s,(typ,topic) in types.items():
            self.pubs[s] = self.create_publisher(typ,f'sensors/{s}/{topic}',qos())
            self.filtered[s] = self.create_publisher(typ,f'sensors/{s}/{topic}_filtered',qos())
        self.sonar_pub = self.create_publisher(Range,'sensors/ultrasonic/range',qos())
        for s,topic in [('flex','raw'),('light','raw_voltage'),('ir','raw_voltage'),('ultrasonic','raw_pulse_width')]:
            self.create_subscription(ScalarStamped,f'sensors/{s}/{topic}',
                                     lambda msg,s=s:self.process(s,msg),qos())

    def report_calibration(self):
        missing = [s for s,c in self.cal.items() if not c.x]
        d = DiagnosticStatus()
        d.name,d.hardware_id = 'ego2sim/sensor_processing','calibration'
        d.level = DiagnosticStatus.WARN if missing else DiagnosticStatus.OK
        d.message = 'Uncalibrated: '+','.join(missing) if missing else 'Calibration loaded'
        d.values = [KeyValue(key='unknown_fov',value=str([s for s,v in
                    [('ir',self.ir_fov),('ultrasonic',self.sonar_fov)] if not math.isfinite(v)]))]
        m = DiagnosticArray(); m.header.stamp=self.get_clock().now().to_msg(); m.status=[d]
        self.diagnostics.publish(m)

    def physical(self,s,value,header):
        if s == 'flex':
            m = ScalarStamped(); m.header=header; m.value=float(value); m.valid=math.isfinite(value)
        elif s == 'light':
            m = Illuminance(); m.header=header; m.illuminance=float(value); m.variance=0.
        else:
            m = range_message(header.stamp,header.frame_id,value,Range.INFRARED,.2,1.5,self.ir_fov)
        return m

    def process(self,s,msg):
        raw = msg.value if msg.valid and math.isfinite(msg.value) else NAN
        if s == 'ultrasonic':
            self.sonar_pub.publish(range_message(msg.header.stamp,msg.header.frame_id,
                sonar_pulse(raw,self.sonar_scale),Range.ULTRASOUND,.1524,6.4516,self.sonar_fov))
            return
        value = self.cal[s].convert(raw)
        if s == 'ir' and not .2 <= value <= 1.5:
            value = NAN
        if s == 'light' and value < 0:
            value = NAN
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        filtered = self.filters[s].update(value,stamp)
        self.pubs[s].publish(self.physical(s,value,msg.header))
        self.filtered[s].publish(self.physical(s,filtered,msg.header))

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
