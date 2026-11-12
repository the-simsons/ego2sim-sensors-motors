"""Exercises generated messages, DDS topics and the firmware adapter when Humble is sourced."""
import importlib.util
import math
import time
import unittest

@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 rclpy unavailable')
class RosMockIntegrationTests(unittest.TestCase):
    def test_all_topics_and_command_acceptance(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from ego2sim_backend.serial_bridge import SerialBridge,qos
        from ego2sim_interfaces.msg import ScalarStamped,MotorState,MotorCommand,ControlMode
        from sensor_msgs.msg import Range,Illuminance
        from diagnostic_msgs.msg import DiagnosticArray
        rclpy.init(args=['--ros-args','-r','__ns:=/ego2sim','-p','mock:=true'])
        nodes = []
        executor = SingleThreadedExecutor()
        try:
            bridge,observer = SerialBridge(),Node('test_observer')
            nodes = [bridge,observer]
            seen = {}
            specs = {
                'sensors/flex/raw':ScalarStamped,'sensors/flex/angle':ScalarStamped,
                'sensors/flex/angle_filtered':ScalarStamped,'sensors/light/raw_voltage':ScalarStamped,
                'sensors/light/illuminance':Illuminance,'sensors/light/illuminance_filtered':Illuminance,
                'sensors/ir/raw_voltage':ScalarStamped,'sensors/ir/range':Range,
                'sensors/ir/range_filtered':Range,'sensors/tof/range':Range,
                'sensors/ultrasonic/raw_pulse_width':ScalarStamped,'sensors/ultrasonic/range':Range,
                'motors/servo/state':MotorState,'motors/dc/state':MotorState,
                'motors/stepper/state':MotorState,'diagnostics':DiagnosticArray}
            subscriptions = [observer.create_subscription(typ,topic,
                lambda msg,topic=topic:seen.__setitem__(topic,msg),qos()) for topic,typ in specs.items()]
            command = observer.create_publisher(MotorCommand,'motors/command',qos())
            mode = observer.create_publisher(ControlMode,'control/mode_command',qos())
            for n in nodes: executor.add_node(n)
            def spin_until(predicate,seconds=5):
                deadline = time.monotonic()+seconds
                while not predicate() and time.monotonic()<deadline:
                    executor.spin_once(timeout_sec=.02)
                self.assertTrue(predicate())
            spin_until(lambda:set(seen)==set(specs))
            self.assertTrue(math.isnan(seen['sensors/flex/angle'].value))
            self.assertFalse(seen['sensors/flex/angle'].valid)
            self.assertFalse(seen['sensors/flex/angle_filtered'].valid)
            self.assertTrue(math.isnan(seen['sensors/flex/angle_filtered'].value))
            self.assertTrue(math.isnan(seen['sensors/light/illuminance'].illuminance))
            self.assertTrue(math.isnan(seen['sensors/ir/range'].range))
            self.assertAlmostEqual(seen['sensors/ultrasonic/range'].range,.6096,places=4)
            bridge.calibration.update(flex=True,light=True,ir=True)
            spin_until(lambda:seen['sensors/flex/angle'].valid and
                       seen['sensors/flex/angle_filtered'].valid)
            angle = seen['sensors/flex/angle']
            filtered = seen['sensors/flex/angle_filtered']
            self.assertTrue(math.isfinite(angle.value))
            self.assertTrue(math.isfinite(filtered.value))
            self.assertEqual(angle.header,filtered.header)

            msg = MotorCommand(); msg.command_id=123; msg.motor=2; msg.operation=1; msg.target=.2
            command.publish(msg)
            spin_until(lambda:seen['motors/stepper/state'].last_command_id==123 and
                       abs(seen['motors/stepper/state'].position_rad-round(.2/(math.tau/1600))*(math.tau/1600))<1e-5)
            command.publish(msg) # Duplicate must not move a second time.
            until = time.monotonic()+.5
            while time.monotonic()<until: executor.spin_once(timeout_sec=.02)
            self.assertAlmostEqual(seen['motors/stepper/state'].position_rad,round(.2/(math.tau/1600))*(math.tau/1600),places=5)
            self.assertFalse(seen['motors/stepper/state'].position_measured)
            m = ControlMode(); m.mode=1; mode.publish(m)
            spin_until(lambda:seen['motors/stepper/state'].control_mode==1)
        finally:
            executor.shutdown()
            for n in reversed(nodes): n.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__': unittest.main()
