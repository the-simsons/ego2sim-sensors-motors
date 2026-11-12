from glob import glob
from setuptools import setup
setup(name='ego2sim_backend', version='0.1.0', packages=['ego2sim_backend'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/ego2sim_backend']),
                  ('share/ego2sim_backend', ['package.xml']),
                  ('share/ego2sim_backend/launch', glob('launch/*.launch.py')),
                  ('share/ego2sim_backend/config', glob('config/*.yaml'))],
      install_requires=['setuptools', 'pyserial'], tests_require=['pytest'], zip_safe=True,
      maintainer='Ego2Sim maintainers', maintainer_email='maintainers@example.com',
      description='ROS 2 lab serial backend', license='BSD-3-Clause',
      entry_points={'console_scripts': [
          'serial_bridge = ego2sim_backend.serial_bridge:main',
          'sensor_processing = ego2sim_backend.sensor_processing:main']})
