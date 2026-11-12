import math
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
from ego2sim_backend.processing import Calibration,LowPass,sonar,sonar_pulse
from ego2sim_backend.protocol import Framer,Session,UptimeTracker,FIRMWARE_LIMITS,parse,validate_command
from ego2sim_backend.telemetry import sensor_values
from ego2sim_backend.transport import MockTransport,SerialTransport

SAMPLE = 'S,1234,500,0.4,0.3,1.000,200.0,190.0,1.200,inf,-inf,0.650,3528,0.6096'

def state(cid=0,motor=1,mode=0,ms=1234,status='idle'):
    return parse(f'MS,{ms},{motor},{mode},3,1,0,0.0,0.0,0.0,1,1,{cid},{status}')

class ProcessingTests(unittest.TestCase):
    def test_missing_and_bounded_calibration(self):
        self.assertTrue(math.isnan(Calibration().convert(500)))
        c = Calibration([100.,200.,400.],[0.,1.,2.])
        self.assertEqual(c.convert(300),1.5)
        self.assertEqual(c.convert(400),2.)
        for x in (0,500,float('nan')):
            self.assertTrue(math.isnan(c.convert(x)))
        # Descending physical output is allowed for the Sharp voltage curve.
        self.assertEqual(Calibration([1.,2.],[1.5,.2]).convert(1.5),.85)

    def test_bad_calibration(self):
        for x,y in [([1],[2]),([1,1],[0,1]),([2,1],[0,1]),([1,2],[0]),([1,2],[0,math.nan])]:
            with self.assertRaises(ValueError): Calibration(x,y)

    def test_time_aware_filter_and_reset(self):
        f = LowPass(1.,2.)
        self.assertEqual(f.update(0.,1.),0.)
        self.assertAlmostEqual(f.update(1.,2.),1-math.exp(-1))
        self.assertEqual(f.update(9.,5.),9.)
        self.assertTrue(math.isnan(f.update(math.nan,6.)))
        self.assertEqual(f.update(2.,6.1),2.)
        self.assertEqual(f.update(3.,5.),3.) # ROS clock moves backward
        self.assertEqual(LowPass(0,1).update(4.,1.),4.)

    def test_sonar_units_and_saturation(self):
        self.assertAlmostEqual(sonar(5*24/512,5),.6096)
        self.assertAlmostEqual(sonar(3.3*6/512,3.3),.1524)
        for v in (-1,0,5,float('nan')):
            self.assertTrue(math.isnan(sonar(v,5)))
        with self.assertRaises(ValueError): sonar(1,0)


class ProtocolTests(unittest.TestCase):
    def ready(self):
        s=Session(); s.reset(connected=True)
        for i in range(3): s.receive(state(motor=i),0.)
        return s

    def test_partial_crlf_overflow_and_non_ascii(self):
        f=Framer(32)
        self.assertEqual(f.feed(b'I,hello'),[])
        self.assertEqual(f.feed(b' world\r\nI,next\n'),['I,hello world','I,next'])
        self.assertEqual(f.feed(b'x'*100+b'\nI,ok\n'),['I,ok'])
        self.assertEqual(f.feed(b'\xff\n'),[])
        self.assertEqual(f.errors,2)
        self.assertLessEqual(len(f.buffer),32)

    def test_firmware_packet_shapes_and_commas(self):
        s=parse(SAMPLE)
        self.assertEqual(s['sonar_pulse_us'],3528.)
        self.assertEqual(s['tof_m'],.65)
        self.assertEqual(s['ir_m'],math.inf)
        self.assertEqual(state(status='stopped, sensor control paused')['status'],'stopped, sensor control paused')
        self.assertEqual(parse('D,2,motor_command,Rejected command #42: stopped, retry manually')['message'],
                         'Rejected command #42: stopped, retry manually')
        self.assertEqual(parse('I,ego2sim firmware ready')['kind'],'I')

    def test_malformed_packets_rejected(self):
        for text in ('S,0,1','MS,0,3,0,0,1,0,0,0,0,0,0,0,idle','D,3,foo,bad',
                     SAMPLE.replace('1.000','6.000'),SAMPLE.replace('1234','-1'),
                     SAMPLE.replace('3528','inf'),'T,1,1,1,flex,0,5','X,1,123','MS,0'):
            with self.subTest(text=text),self.assertRaises(ValueError):parse(text)

    def test_uptime_wrap_and_restart(self):
        t=UptimeTracker()
        for ms in (0xfffffffe,0xffffffff,0,0,1,100):self.assertFalse(t.update(ms))
        self.assertTrue(t.update(2))

    def test_command_format_and_firmware_limits(self):
        s=self.ready()
        self.assertEqual(s.command(42,1,1,math.pi/2,0.),'C,42,1,1,1.57079633\n')
        for args in ((0,0,math.pi),(1,1,20*math.pi),(1,2,100*math.tau/60),(2,1,-20*math.pi),(0,3,0)):
            validate_command(*args)
        for args in ((0,1,1),(2,2,1),(1,2,11),(2,1,63),(0,0,4),(1,1,math.nan)):
            with self.assertRaises(ValueError):validate_command(*args)
        with self.assertRaises(ValueError):s.command(42,1,1,1,0.)
        with self.assertRaises(ValueError):s.command(0,0,3,0,0.)

    def test_only_matching_motor_feedback_confirms_acceptance(self):
        s=self.ready();s.command(42,1,1,1.,0.)
        self.assertIn(42,s.pending)
        s.receive(state(42,motor=0),.1)
        self.assertIn(42,s.pending)
        s.receive(state(42,motor=1),.2)
        self.assertNotIn(42,s.pending)
        self.assertEqual(s.outcomes[-1].result,'accepted')

    def test_rejection_and_late_confirmation(self):
        s=self.ready();s.command(42,1,1,1.,0.)
        s.receive(parse('D,2,motor_command,Rejected command #42: motion commands need GUI mode'),.1)
        self.assertEqual(s.outcomes[-1].result,'rejected')
        s.command(43,1,1,1.,.2);s.expire(3.)
        self.assertEqual(s.outcomes[-1].result,'unknown')
        s.receive(state(43,ms=2000),3.1)
        self.assertEqual(s.outcomes[-1].result,'accepted')

    def test_no_replay_after_disconnect_or_boot(self):
        s=self.ready();s.command(42,1,1,1.,0.)
        s.reset(reason='unplugged')
        self.assertFalse(s.ready);self.assertFalse(s.pending)
        self.assertEqual(s.outcomes[-1].result,'unknown')
        with self.assertRaises(ValueError):s.command(43,1,1,1.,0.)
        s.reset(connected=True);s.receive(state(ms=20),.1)
        with self.assertRaises(ValueError):s.command(42,1,1,1.,.1)
        s.command(43,1,1,1.,.1)
        self.assertTrue(s.receive(parse('I,ego2sim firmware ready'),.2))
        self.assertFalse(s.ready);self.assertFalse(s.pending)
        self.assertEqual(s.high_id,43)

    def test_missed_banner_uptime_restart_and_stale_stop(self):
        s=self.ready();s.command(42,1,1,1.,0.)
        self.assertTrue(s.receive(state(ms=2),.1))
        self.assertEqual(s.outcomes[-1].result,'unknown')
        s.expire(2.)
        with self.assertRaises(ValueError):s.command(43,1,1,1.,2.)
        self.assertEqual(s.command(43,1,3,0.,2.),'C,43,1,3,0\n')
        s.receive(state(43,mode=1,ms=5),2.1)
        with self.assertRaises(ValueError):s.command(44,1,1,1.,2.1)
        self.assertEqual(s.command(44,1,3,0.,2.1),'C,44,1,3,0\n')

    def test_bounds_pending_and_history(self):
        s=self.ready()
        for cid in range(1,65):s.command(cid,1,1,1.,0.)
        with self.assertRaises(ValueError):s.command(65,1,1,1.,0.)
        s.expire(3.)
        self.assertFalse(s.pending)
        for cid in range(65,250):
            s.receive(state(ms=2000+cid),float(cid))
            s.command(cid,1,1,1.,float(cid));s.receive(state(cid,ms=2001+cid),float(cid))
        self.assertLessEqual(len(s.issued),128)
        self.assertLessEqual(len(s.outcomes),64)

class TelemetryTests(unittest.TestCase):
    def test_calibration_gate_and_pulse_width_units(self):
        r=parse(SAMPLE)
        v=sensor_values(r,{})
        self.assertTrue(math.isnan(v['sensors/flex/angle']))
        self.assertTrue(math.isnan(v['sensors/light/illuminance']))
        self.assertTrue(math.isnan(v['sensors/ir/range']))
        self.assertEqual(v['sensors/ultrasonic/raw_pulse_width'],3528)
        self.assertNotIn('sensors/ultrasonic/raw_voltage',v)
        v=sensor_values(r,dict(flex=True,light=True,ir=True))
        self.assertEqual(v['sensors/flex/angle_filtered'],.3)
        self.assertEqual(v['sensors/ir/range'],math.inf)
        self.assertEqual(v['sensors/ir/range_filtered'],-math.inf)
        self.assertAlmostEqual(sonar_pulse(3528),.6096)

class MockIntegrationTests(unittest.TestCase):
    def test_wire_motion_ack_dedup_modes_and_stop_hold(self):
        clock=[10.]
        with patch('ego2sim_backend.transport.time.monotonic',side_effect=lambda:clock[0]):
            m=MockTransport();s=Session()
            def drain():
                records=[]
                for event,epoch,data in m.poll():
                    if event=='connected':s.reset(connected=True)
                    elif event=='data':
                        r=parse(data.decode().strip());records.append(r);s.receive(r,clock[0])
                return records
            drain();self.assertFalse(s.ready)
            clock[0]+=.1;drain();self.assertTrue(s.ready)
            wire=s.command(42,2,1,.25,clock[0]);m.send(1,wire);m.send(1,wire)
            clock[0]+=.2
            records=drain();step=next(r for r in records if r['kind']=='MS' and r['motor']==2)
            expected=round(.25/(math.tau/1600))*(math.tau/1600)
            self.assertAlmostEqual(step['position'],expected,places=5)
            self.assertFalse(step['pm']);self.assertEqual(step['last_id'],42)
            self.assertEqual(s.outcomes[-1].result,'accepted')
            m.send(1,'M,1\n');clock[0]+=.1;drain();self.assertEqual(s.mode,1)
            m.send(1,s.command(43,2,3,0.,clock[0]));clock[0]+=.1;drain()
            pos=m.motors[2]['pos'];clock[0]+=.5;drain()
            self.assertEqual(m.motors[2]['pos'],pos)
            self.assertTrue(m.motors[2]['held'])
            m.send(1,'M,0\n');clock[0]+=.1;drain();self.assertFalse(m.motors[2]['held'])

class SerialWorkerTests(unittest.TestCase):
    def test_reconnect_drops_previous_epoch_commands(self):
        written = []
        reads = [threading.Event(),threading.Event()]
        opened = []
        class Port:
            def __init__(self,*a,**k):
                self.index = len(opened); opened.append(self)
            def __enter__(self): return self
            def __exit__(self,*a): return False
            def reset_input_buffer(self): pass
            def write(self,data): written.append((self.index,data)); return len(data)
            def read(self,n):
                reads[self.index].wait(.02)
                if reads[self.index].is_set(): raise OSError('unplugged')
                return b''
        module = types.SimpleNamespace(Serial=Port,SerialException=OSError)
        with patch.dict(sys.modules,serial=module):
            transport = SerialTransport('fake')
            try:
                deadline = time.monotonic()+3
                events = []
                while not events and time.monotonic()<deadline:
                    events += transport.poll(); time.sleep(.01)
                self.assertEqual(events[0][:2],('connected',1))
                reads[0].set()
                while len(opened)<2 and time.monotonic()<deadline: time.sleep(.01)
                self.assertEqual(len(opened),2)
                transport.send(1,'OLD\n')
                transport.send(2,'NEW\n')
                while not written and time.monotonic()<deadline: time.sleep(.01)
                self.assertEqual(written,[(1,b'NEW\n')])
            finally: transport.close()


if __name__ == '__main__': unittest.main()
