"""Feed actual unchanged firmware emissions through the Python wire parser."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from ego2sim_backend.protocol import parse,Session

class FirmwareTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('clang++') or shutil.which('g++'),'No host C++ compiler')
    def test_actual_firmware_wire_compatibility(self):
        root=Path(__file__).resolve().parents[3]
        fixtures=Path(__file__).parent/'firmware'
        compiler=shutil.which('clang++') or shutil.which('g++')
        with tempfile.TemporaryDirectory() as d:
            binary=str(Path(d)/'firmware_test')
            build=subprocess.run([compiler,'-std=c++17','-Wall','-Wextra',
                '-I'+str(fixtures),'-DMAIN_SKETCH="'+str(root/'main/main.ino')+'"',
                str(fixtures/'test_main.cpp'),'-o',binary],capture_output=True,text=True)
            self.assertEqual(build.returncode,0,build.stderr)
            run=subprocess.run([binary],capture_output=True,text=True)
            self.assertEqual(run.returncode,0,run.stderr)
            records=[parse(line) for line in run.stdout.splitlines()]
            self.assertEqual({r['kind'] for r in records},{'S','MS','D','I','KS'})
            sensors=next(r for r in records if r['kind']=='S')
            self.assertEqual(sensors['sonar_pulse_us'],3528)
            self.assertAlmostEqual(sensors['sonar_m'],.6096)
            states=[r for r in records if r['kind']=='MS']
            self.assertEqual([r['last_id'] for r in states],[42,43,46,47])
            self.assertTrue(states[0]['pm'])
            self.assertFalse(states[1]['pm'])
            self.assertIn('stopped, sensor control paused',states[2]['status'])
            rejects=[r['message'] for r in records if r['kind']=='D' and r['name']=='motor_command']
            self.assertTrue(any('#44:' in x for x in rejects))
            self.assertTrue(any('#45:' in x for x in rejects))
