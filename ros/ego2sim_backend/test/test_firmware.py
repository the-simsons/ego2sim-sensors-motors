"""Host C++ harness: checks actual sketch logic; not an Arduino cross-compile."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

class FirmwareTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('clang++') or shutil.which('g++'), 'No C++ compiler')
    def test_actual_sketch_protocol_and_watchdog(self):
        root = Path(__file__).resolve().parents[3]
        fixtures = Path(__file__).parent/'firmware'
        compiler = shutil.which('clang++') or shutil.which('g++')
        with tempfile.TemporaryDirectory() as d:
            binary = str(Path(d)/'firmware_test')
            subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',
                '-I'+str(fixtures),'-DMAIN_SKETCH="'+str(root/'main/main.ino')+'"',
                str(fixtures/'test_main.cpp'),'-o',binary],check=True,capture_output=True,text=True)
            result = subprocess.run([binary],check=True,capture_output=True,text=True)
            self.assertIn('watchdog passed',result.stdout)
