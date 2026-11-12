# ROS 2 Humble backend

`main/main.ino` is the serial protocol reference and is maintained by the firmware
teammate. The backend now reads its existing S/MS/D/I records and writes C/M
commands at 115200 baud. The firmware file is unchanged by this integration.

## Build and start

From the repository root on the Humble laptop:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros --ignore-src -r -y --rosdistro humble
colcon build --base-paths ros --symlink-install
source install/setup.bash
ros2 launch ego2sim_backend hardware.launch.py port:=/dev/ttyACM0
```

In another terminal, from the repository root:

```bash
./ros/scripts/start_foxglove_bridge.bash port:=8765
```

This sources Humble and the built workspace before starting Foxglove Bridge.
Connect Foxglove to `ws://localhost:8765`. Do not open Serial Monitor at the same
time as the backend. `mock.launch.py` provides a ROS mock using the same wire
parser without USB hardware; stop the standalone GUI mock server before using
Foxglove Bridge because both use port 8765.

## What changed

- `serial_bridge` forwards firmware physical and filtered values directly. No
  incompatible X/H handshake, version/session tokens or synthetic ACKs are sent.
- The default launch does **not** run host sensor_processing, avoiding duplicate
  publishers and double filtering. Optional `host_processing:=true` instead enables
  host calibration/filtering from raw data and disables firmware physical publishers.
- Sonar raw data is now `/ego2sim/sensors/ultrasonic/raw_pulse_width`, ScalarStamped
  in microseconds, matching the firmware's PW acquisition (147 us/inch). The old
  raw_voltage topic is removed. All other topic names and message definitions remain.
- Flex raw is ADC counts; flex angles are ScalarStamped radians. Distances are
  meters, illuminance lux. IR +/-inf values retain the firmware's out-of-range
  semantics; NaN means unavailable. Headers use ROS receipt time; MCU uptime is
  retained for diagnostics/reset detection.
- Physical flex/light/IR output is suppressed until its measured calibration is
  verified. Set `flex_calibration_verified`, `light_calibration_verified` and
  `ir_calibration_verified` in config/backend.yaml individually after checking the
  corresponding firmware constants. Defaults are false; raw values still publish.
- Host limits match firmware: servo 0–pi rad, relative moves +/-20*pi rad (3600 deg),
  DC velocity +/-100 RPM. Configured limits can be tighter, never looser.

## Commands and feedback

ROS MotorCommand -> `C,id,motor,operation,target` plus newline. Motor IDs are
0=servo, 1=DC, 2=stepper; operations 0=absolute, 1=relative, 2=velocity, 3=STOP.
Targets are radians or rad/s. ControlMode -> `M,0` (GUI) or `M,1` (SENSOR).
Example DC +90 degree move: `C,42,1,1,1.57079633`.

Motion requires fresh motor feedback and reported GUI mode. STOP is allowed in
either mode while connected. Command IDs must be nonzero and increasing for the
backend process lifetime; zero is the firmware's initial no-command sentinel.
Relative moves are sent once and never retried or replayed on reconnect. MS
last_command_id confirms acceptance, not completion. D rejection lines become
command diagnostics; missing confirmation means unknown outcome, not failure.
Fast successive commands may overwrite the firmware's last ID before telemetry,
so send one command at a time when checking acceptance. Actual mode comes from MS.
Relative-motion MS target values are absolute goal positions.

Stale sensors become NaN; stale motor states become UNAVAILABLE. Boot banners and
backward uptime detect resets (uint32 wrap is supported). Queued old-generation
writes are dropped after detected resets/reconnects. This cannot retract bytes
already delivered: the firmware has no session fence and only remembers eight IDs.

**Firmware limitation:** there is no communication-loss watchdog. A running motor
may continue after USB loss or a backend crash. The GUI/backend do not claim that
loss of telemetry stops the motor; this needs firmware-owner coordination.

## Checks and hardware test

```bash
PYTHONPATH=ros/ego2sim_backend python3 -m unittest discover -s ros/ego2sim_backend/test -v
```

The host C++ harness compiles the unchanged sketch against test doubles and feeds
its real output into the Python parser. It is not a board cross-compilation.
On the robot, check all sensors, then servo/DC/stepper commands individually,
encoder scaling (firmware default: 360 counts/output rev with 4x decoding), GUI and
button mode switching, STOP in both modes, and reconnect behavior. Verify calibration
and physical motor limits before treating displayed values as measured results.
