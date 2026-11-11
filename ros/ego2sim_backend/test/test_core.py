import importlib.util
import math
import queue
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
from ego2sim_backend.processing import Calibration, LowPass, sonar
from ego2sim_backend.protocol import Framer, Session, parse, validate_command
from ego2sim_backend.transport import MockTransport, SerialTransport

LIMITS = dict(servo_min=0.,servo_max=math.pi,move_max=2*math.pi,velocity_max=10.)

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
    def session(self):
        s = Session(LIMITS)
        s.receive(parse(f'H,1,{s.token},0,0,0'))
        return s

    def test_partial_multiple_and_overlong_frames(self):
        f = Framer(32)
        self.assertEqual(f.feed(b'T,1,1,'),[])
        self.assertEqual(f.feed(b'10,flex,0,500\r\nB,1\n'),['T,1,1,10,flex,0,500','B,1'])
        self.assertEqual(f.feed(b'x'*100+b'\nB,1\n'),['B,1'])
        self.assertEqual(f.feed(b'\xff\n'),[])
        self.assertEqual(f.errors,2)
        self.assertLessEqual(len(f.buffer),32)

    def test_parser_validation(self):
        self.assertEqual(parse('T,1,3,12,ir,0,1.2')['value'],1.2)
        self.assertTrue(math.isnan(parse('T,1,4,13,tof,2,123')['value']))
        for text in ('T,2,1,1,flex,0,1','T,1,-1,1,flex,0,1','T,1,1,1,flex,0,nan',
                     'T,1,1,1,other,0,1','T,1,1,1,flex,0,1024','A,1,1,0,2,OK',
                     'H,1,4294967296,0,0,0','junk','T,1,1,1,tof,0,inf'):
            with self.subTest(text=text),self.assertRaises(ValueError): parse(text)

    def test_command_limits(self):
        for args in [(0,0,math.pi),(1,1,-2.),(1,2,-3.),(2,1,1.),(2,3,0)]:
            validate_command(*args,LIMITS)
        for args in [(0,1,1.),(2,2,1.),(1,0,1.),(0,0,4.),(1,2,11.),(2,1,7.),(9,3,0.),(0,3,math.nan)]:
            with self.assertRaises(ValueError): validate_command(*args,LIMITS)

    def test_dedup_ack_and_timeout_no_retry(self):
        s = self.session()
        wire = s.command(0,1,1,1.,0.)
        self.assertIn(',0,1,1,1',wire)
        with self.assertRaises(ValueError): s.command(0,1,1,1.,.1)
        s.receive(parse(f'A,1,{s.token+1},0,1,OK'))
        self.assertIn(0,s.pending)
        s.receive(parse(f'A,1,{s.token},0,1,OK'))
        self.assertNotIn(0,s.pending)
        self.assertEqual(s.outcomes[-1],(0,'accepted:OK'))
        s.command(1,2,1,.5,0.)
        s.expire(2.)
        self.assertFalse(s.pending)
        self.assertIn('not retried',s.outcomes[-1][1])
        with self.assertRaises(ValueError): s.command(1,2,1,.5,2.)

    def test_disconnect_restart_session_fence(self):
        s = self.session()
        old = s.token
        s.command(42,1,1,1.,0.)
        reply = s.receive(parse('B,1'))
        self.assertTrue(reply.startswith('X,1,'))
        self.assertFalse(s.ready)
        self.assertFalse(s.pending)
        s.receive(parse(f'H,1,{old},0,0,0'))
        self.assertFalse(s.ready)
        s.receive(parse(f'H,1,{s.token},0,0,0'))
        with self.assertRaises(ValueError): s.command(42,1,1,1.,0.)
        self.assertIn('unknown',s.outcomes[-1][1])

    def test_sequence_wrap_and_acquisition_retention(self):
        s = self.session()
        s.receive(parse('T,1,4294967295,4294967295,flex,0,100'))
        s.receive(parse('T,1,0,0,flex,0,101'))
        self.assertEqual(s.acquisition_ms['flex'],0)
        with self.assertRaises(ValueError): s.receive(parse('T,1,0,0,flex,0,101'))

    def test_sensor_mode_stop_and_bound(self):
        s = self.session()
        s.receive(parse(f'H,1,{s.token},0,1,0'))
        with self.assertRaises(ValueError): s.command(1,1,1,1.,0.)
        s.command(1,1,3,0.,0.)
        s.mode = 0
        for cid in range(2,65): s.command(cid,1,1,1.,0.)
        with self.assertRaises(ValueError): s.command(65,1,1,1.,0.)
        s.expire(2.)
        self.assertLessEqual(len(s.outcomes),64)

class MockIntegrationTests(unittest.TestCase):
    def test_wire_ack_relative_once_feedback_modes_watchdog(self):
        clock = [10.]
        with patch('ego2sim_backend.transport.time.monotonic',side_effect=lambda:clock[0]):
            m = MockTransport(LIMITS)
            s = Session(LIMITS)
            m.poll()
            m.send(1,s.reset())
            for _,_,data in m.poll(): s.receive(parse(data.decode().strip()))
            wire = s.command(10,2,1,.25,clock[0])
            m.send(1,wire)
            m.send(1,wire) # Firmware dedup independent of host dedup.
            records = [parse(d.decode().strip()) for _,_,d in m.poll()]
            self.assertEqual([r['accepted'] for r in records],[1,0])
            for r in records: s.receive(r)
            clock[0] += .5
            records = [parse(d.decode().strip()) for _,_,d in m.poll()]
            for r in records: s.receive(r)
            state = [r for r in records if r['kind']=='S' and r['motor']==2][0]
            self.assertEqual(state['position'],.25)
            self.assertFalse(state['pm'])
            self.assertEqual(state['last_id'],10)
            self.assertEqual({r['sensor'] for r in records if r['kind']=='T'},set(('flex','ir','light','tof','ultrasonic')))
            m.send(1,f'M,1,{s.token},1\n')
            for _,_,d in m.poll(): s.receive(parse(d.decode().strip()))
            self.assertEqual(s.mode,1)
            with self.assertRaises(ValueError): s.command(11,1,2,1.,clock[0])
            m.send(1,s.command(11,0,3,0.,clock[0]))
            m.poll()
            clock[0] += 2.
            records = [parse(d.decode().strip()) for _,_,d in m.poll()]
            states = [r for r in records if r['kind']=='S']
            self.assertTrue(all(not r['moving'] and not r['enabled'] and r['watchdog'] for r in states))

    def test_servo_stop_holds_commanded_angle(self):
        m = MockTransport(LIMITS)
        m.send(1,'X,1,99\n')
        m.send(1,'C,1,99,1,0,0,1.2\n')
        m.send(1,'C,1,99,2,0,3,0\n')
        self.assertEqual(m.motors[0]['pos'],1.2)
        self.assertTrue(m.motors[0]['enabled'])
        m.send(1,'X,1,100\n')
        self.assertEqual(m.motors[0]['pos'],1.2)
        self.assertTrue(m.motors[0]['enabled'])

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
