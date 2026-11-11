# Ego2Sim ROS 2 backend (Humble)

`ego2sim_interfaces` is an ament_cmake message package. `ego2sim_backend` is
ament_python/rclpy. The GUI owns no serial connection: it uses only the topics
below. Run one backend instance per board. Do not open Arduino Serial Monitor
while running the hardware bridge. Existing `motor_test/motor_test.ino` remains a
standalone manual test and is not the protocol firmware.

## Implemented and pending

- `serial_bridge` owns USB, parses telemetry and feedback, validates and sends
  motor/mode requests, tracks ACKs, detects boot/disconnect/stale data, and reconnects.
- `sensor_processing` converts measured calibration tables and filters physical values.
- Mock uses the same parser, messages, topics, QoS and command rules, with simulated
  motor motion, ACK/NACK, mode arbitration and watchdog. It does not model encoder
  hardware or define an unspecified mapping from sensors to motors in SENSOR mode.
- `main/main.ino` retains A1 flex, A2 IR, ten-sample analog averaging, and VL53L1X
  I2C short mode/50 ms budget/100 ms intermeasurement. Human-readable output is
  replaced with protocol v1. Flex is a bend sensor, not an FSR.
- Light and sonar pins default to **disabled** (`-1`). No wiring was found for them.
- Hardware motors are **not integrated**. Every motor command, including STOP, gets
  a NACK, and motor states are disabled/unavailable with NaN physical fields. GUI
  mode requests are ACKed; SENSOR mode is NACKed; actual mode remains GUI. The
  motor test sketch's pins were not copied into the acquisition sketch. There is
  no encoder code, steps/revolution, gear ratio or confirmed joint limits in it.
  Host validation is not a substitute for implementing motor control on firmware.

## Install, build and launch

Use Ubuntu 22.04 with ROS 2 Humble installed and sourced (Python 3.10). From this
repository's root:

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-serial python3-pytest
# If rosdep has not previously been initialized: sudo rosdep init
rosdep update
rosdep install --from-paths ros --ignore-src -r -y --rosdistro humble
colcon build --base-paths ros --symlink-install
source install/setup.bash
ros2 launch ego2sim_backend mock.launch.py
# Hardware, after uploading main/main.ino:
ros2 launch ego2sim_backend hardware.launch.py port:=/dev/ttyACM0
# Alternative with explicit launch options:
ros2 launch ego2sim_backend backend.launch.py mock:=false port:=/dev/ttyACM0 config:=/absolute/path/backend.yaml
```

Wrapper launches forward `port` and `config` through the launch context. On Linux,
grant the current user serial access using the board's device group (usually
`dialout`), then log in again. A stable `/dev/serial/by-id/...` path is preferable
when available. The serial baud rate is fixed at 115200, 8N1.

Install SparkFun's **Qwiic Distance Sensor (VL53L1X)** Arduino library. Compile and
upload the existing `main/` sketch for the **confirmed board FQBN**. No board model
was supplied, so no FQBN or upload port is assumed. Verify the ADC reference and
resolution: the existing sketch uses 5 V and 10-bit conversion. Before enabling
TEMT6000/MB1000, verify their connections, supply and ADC compatibility, then define
`LIGHT_PIN` and `SONAR_PIN` to the confirmed analog pins (compile flags or explicit
source definitions). Do not choose A1/A2 or I2C pins already in use. Set
`sonar_supply_v` to the measured MB1000 supply.

## Foxglove startup order

Install the bridge with `sudo apt install ros-humble-foxglove-bridge`. In **each**
terminal launching ROS nodes or the Foxglove bridge, source Humble first and the
workspace containing both Ego2Sim packages second. Starting the Foxglove desktop
app first is harmless; it is the **foxglove_bridge process** that must inherit the
custom-message workspace environment. Restart an already running bridge after
building or changing the interface package.

From the repository root, after the build above:

```bash
# Terminal 1: backend
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ego2sim_backend mock.launch.py  # or hardware.launch.py

# Terminal 2: bridge (sources Humble and this repository's install/setup.bash)
./ros/scripts/start_foxglove_bridge.bash port:=8765
# If built in a different colcon workspace:
EGO2SIM_SETUP=/absolute/path/to/ego2sim_ws/install/setup.bash ./ros/scripts/start_foxglove_bridge.bash port:=8765
```

Equivalent manual startup in Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 interface show ego2sim_interfaces/msg/ScalarStamped
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Connect the GUI to `ws://localhost:8765` using Foxglove WebSocket and import the
layout described in `gui/README.md`. Stop the standalone `gui/mock-server` before
using the ROS bridge: both use port 8765. The ROS backend's `mock.launch.py` runs
**with** foxglove_bridge; the standalone GUI mock replaces it.
[Foxglove ROS 2 documentation](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2).

Flex topic paths and angle payloads match the pulled GUI: raw ADC counts on
`/ego2sim/sensors/flex/raw` and radians on `/ego2sim/sensors/flex/angle` and
`/ego2sim/sensors/flex/angle_filtered`, all as `ego2sim_interfaces/msg/ScalarStamped`.
The angle topics were changed from JointState with user approval to match the GUI's
`value`/`valid` fields. Rebuild, source the workspace and restart the backend and
bridge after this type change; do not run older JointState publishers on those topics.

## Motor pin conflict pending wiring confirmation

The standalone motor test assigns STEP=D2, DIR=D3; DC drive IN1=D13, IN2=D12,
ENA=D11; servo=D9; stepper MS1=D4, MS2=D5, EN=D6. The reported DC conflict on D2/D3
is not represented by encoder wiring/code in the repository; the GUI panel,
contract, mock server and layout do not define Arduino pin assignments either. Reserve those pins
for the confirmed DC requirement and confirm free STEP/DIR pins before rewiring
or changing the motor test. Do not select replacement pins from appearance alone.
The ROS acquisition sketch currently configures no motor outputs, so this issue
blocks future motor integration rather than changing current sensor acquisition.

## Public topic contract

Everything is under `/ego2sim`. All publishers and subscribers use reliable,
volatile QoS, depth 10. Types prefixed `ego2sim_interfaces/` are custom messages.
The four message definitions preserve the agreed fields and constants exactly.

| Backend publishes (relative to namespace) | Type |
|---|---|
| sensors/flex/raw | ego2sim_interfaces/msg/ScalarStamped |
| sensors/flex/angle | ego2sim_interfaces/msg/ScalarStamped |
| sensors/flex/angle_filtered | ego2sim_interfaces/msg/ScalarStamped |
| sensors/light/raw_voltage | ego2sim_interfaces/msg/ScalarStamped |
| sensors/light/illuminance | sensor_msgs/msg/Illuminance |
| sensors/light/illuminance_filtered | sensor_msgs/msg/Illuminance |
| sensors/ir/raw_voltage | ego2sim_interfaces/msg/ScalarStamped |
| sensors/ir/range | sensor_msgs/msg/Range |
| sensors/ir/range_filtered | sensor_msgs/msg/Range |
| sensors/tof/range | sensor_msgs/msg/Range |
| sensors/ultrasonic/raw_voltage | ego2sim_interfaces/msg/ScalarStamped |
| sensors/ultrasonic/range | sensor_msgs/msg/Range |
| motors/servo/state | ego2sim_interfaces/msg/MotorState |
| motors/dc/state | ego2sim_interfaces/msg/MotorState |
| motors/stepper/state | ego2sim_interfaces/msg/MotorState |
| diagnostics | diagnostic_msgs/msg/DiagnosticArray |

| Backend subscribes | Type |
|---|---|
| motors/command | ego2sim_interfaces/msg/MotorCommand |
| control/mode_command | ego2sim_interfaces/msg/ControlMode |

```bash
ros2 topic echo /ego2sim/sensors/flex/raw
ros2 topic echo /ego2sim/diagnostics
# Motor 0=servo, 1=DC, 2=stepper. Operations 0=absolute, 1=relative, 2=velocity, 3=STOP.
ros2 topic pub --once /ego2sim/motors/command ego2sim_interfaces/msg/MotorCommand '{command_id: 1, motor: 0, operation: 0, target: 1.57}'
ros2 topic pub --once /ego2sim/motors/command ego2sim_interfaces/msg/MotorCommand '{command_id: 2, motor: 1, operation: 1, target: -0.5}'
ros2 topic pub --once /ego2sim/motors/command ego2sim_interfaces/msg/MotorCommand '{command_id: 3, motor: 1, operation: 2, target: 1.0}'
ros2 topic pub --once /ego2sim/motors/command ego2sim_interfaces/msg/MotorCommand '{command_id: 4, motor: 1, operation: 3, target: 0.0}'
ros2 topic pub --once /ego2sim/control/mode_command ego2sim_interfaces/msg/ControlMode '{mode: 1}'
```

The GUI must allocate strictly increasing uint32 command IDs, starting at any ID
(including zero), for the lifetime of a backend process. Duplicate or older IDs
are rejected, including after reconnects/resets. Do not wrap IDs within that
process; coordinate a new session before exhaustion. Never repeat relative moves
periodically. No command is automatically retried. NACKs and ACK timeouts are
visible in diagnostics; a timeout/disconnect means outcome **unknown**, not stopped
or rejected. The bounded history holds 64 outcomes; dedup uses a constant-size
high-water mark. At most 64 commands may await ACK. Restarting the backend creates
a new session; GUI code must not automatically replay previously sent commands.

The host accepts servo absolute radians, DC relative radians or signed rad/s,
stepper relative radians, and STOP for any motor in either mode. Default validation
limits are servo [0, pi], relative magnitude <=2*pi, velocity magnitude <=10 rad/s.
These are conservative software/mock limits, **not verified hardware limits**.
Adjust to measured hardware before integration, and implement the same or tighter
limits on the Arduino. Feedback alone determines actual global GUI/SENSOR mode.
Mock rejects overlapping relative motion with BUSY instead of silently replacing a move.
Servo STOP preserves its commanded angle; DC/stepper STOP clears simulated motion.

`last_command_id` comes exclusively from firmware `S` records; transmitting or
receiving an ACK never fabricates a motor state. An unavailable state uses zero
for this field (there is no validity flag in the agreed message); inspect `status`
and `enabled` before interpreting it. DC real feedback must be encoder-measured;
servo angle is commanded unless a feedback sensor exists; stepper position must be
estimated from **executed** steps. Mock positions/velocities are explicitly
`SIMULATED`, with both measured flags false, including DC.

## Measurements, calibration and filters

Headers initially use ROS receipt time, never MCU uptime as ROS time. MCU uint32
acquisition milliseconds are retained internally and summarized in diagnostics;
wraparound is expected. Filter output retains the exact input header. Frame IDs
are `flex_link`, `light_link`, `ir_link`, `tof_link`, `ultrasonic_link` and each
motor's name plus `_link`; TF publication is outside this backend's scope.

- Flex raw is ADC counts; angle and angle_filtered are radians. All three use
  ScalarStamped (`value`, `valid`); invalid or uncalibrated angles have valid=false
  and value=NaN. Filtered angles preserve the source header.
- Light/IR/ultrasonic raw values are volts. Light physical output is lux. All
  physical distances are meters. ToF wire millimeters are converted by the bridge.
- Invalid scalar: `valid=false`, value NaN. Invalid/unavailable physical data: NaN.
  Source status failure is immediately propagated. Missing/stale channels publish
  NaN every 0.5 s after `stale_timeout_s` (default 1 s); motor states also expire.
- Sharp GP2Y0A02YK has min/max 0.2/1.5 m; no GP2Y0A21YK curve is used. VL53L1X
  uses 0.04/1.3 m metadata for this sketch's short mode and a typical 27-degree
  full FoV from ST with default full ROI. This is not a guarantee of accuracy for
  every target. Reconfigure metadata if firmware mode/ROI changes.
- Sharp/ToF radiation type is INFRARED; sonar is ULTRASOUND. Sharp and sonar beam
  widths depend on the target/geometry; their FoV defaults to NaN. Configure a
  defensible measurement if the GUI requires a cone; never substitute zero or an
  invented angle. Some visualization tools may require handling this unknown FoV.
- MB1000: distance_inches=voltage*512/supply; meters=inches*0.0254. Quantifiable
  output range is 6–254 inches (0.1524–6.4516 m). Six inches is a **saturated lower
  bound**, representing any closer object too, not an accurate six-inch distance.
  Values outside the output range are invalid. No fabricated calibration is used.

Flex angle, lux and IR distance remain NaN until measured calibration arrays are
provided, **including in mock mode**. Copy `config/backend.yaml` and add paired
float arrays under `/ego2sim/sensor_processing/ros__parameters`:

- `flex_adc` and `flex_angle_rad`: fixture the installed flex at known angles and
  collect mean ADC counts. Sensor orientation/divider wiring determine slope.
- `light_voltage` and `light_lux`: measure alongside a reference luxmeter under the
  intended spectrum. Do not equate an unverified resistor/typical photocurrent
  formula with a measured illuminance calibration.
- `ir_voltage` and `ir_distance_m`: use known target distances across 0.2–1.5 m.
  Sort by **increasing voltage**; distances may decrease. The inverse relation is
  nonlinear, so use several measured points. Near-field ambiguity is not resolved
  by an extrapolated curve.

Each table needs >=2 finite paired points with strictly increasing inputs. Empty
arrays are allowed and mean uncalibrated. Piecewise linear interpolation never
extrapolates. Negative lux/out-of-range IR is invalid. Calibration is loaded at
startup: restart nodes after editing the YAML. Sensor-processing diagnostics list
missing calibration and unknown FoV metadata.

`filter_tau_s` sets exponential smoothing: alpha=1-exp(-dt/tau), computed using
source ROS stamps; tau=0 disables smoothing. Invalid input, nonincreasing time or
a gap >`filter_stale_s` resets filter state. Filtered topics exist only for flex,
light and IR as agreed.

## Serial protocol v1

ASCII, comma-separated tokens, LF terminated (CRLF accepted), no spaces/commas in
reason/status tokens. 115200 8N1. Host RX frames <=256 bytes; Arduino commands <=95
bytes excluding newline; overlong frames are discarded through LF. Firmware
services at most 64 input bytes per loop. Unsigned integers are decimal uint32
unless an enum/bool; finite decimal floats or `nan` for invalid telemetry.

Host to Arduino:

```text
X,1,session                       establish nonzero random uint32 host session
P,1,session                       heartbeat every 0.3 s
C,1,session,id,motor,operation,target
M,1,session,mode                  0=GUI, 1=SENSOR
```

Arduino to host:

```text
B,1                              boot notification
H,1,session,uptime_ms,actual_mode,watchdog_expired
T,1,sequence,acquisition_ms,sensor,status,value
A,1,session,id,accepted,reason     motor ACK/NACK (accepted=0 or 1)
R,1,session,requested_mode,accepted,reason  mode ACK/NACK
S,1,sequence,acquisition_ms,motor,actual_mode,enabled,moving,position_rad,velocity_rad_s,operation,target,position_measured,velocity_measured,last_command_id,session,status,watchdog_expired
```

`T.sensor` is flex/light/ir/tof/ultrasonic. `T.value` is ADC counts for flex,
millimeters for ToF, volts otherwise. `T.status`: 0=OK, 1=unwired/unavailable,
2=invalid measurement (reserved generic), 3=timeout, 16+native VL53L1X range error
code (255 if outside the encodable range). Nonzero status always becomes NaN.
Sequence numbers are global across T and S, modulo 2^32. Duplicate/backward
sequences are rejected; MCU acquisition milliseconds also wrap modulo 2^32.
The sensor-only sketch emits no S records, so all real motor feedback is unavailable.

A connection or B resets pending outcomes to unknown and creates a fresh session.
Host ignores data until H echoes that session. Old-session motor/ACK/mode records
are ignored. Device commands with old sessions are rejected. A missing boot line
is handled by silence detection (>max(1.5 s, stale_timeout_s)) and a new handshake.
Handshake-only retries are safe; **motor commands are never replayed**. The serial
worker tags writes with a connection generation and discards writes from earlier
connections; reset session tokens fence queued old commands on the same connection.

Only firmware acceptance may advance the motor's last accepted command ID. Future
motor firmware must deduplicate accepted IDs before executing motion, retain the
accepted high-water mark across repeated X for the same session, invalidate motion
on a new session, and emit S from measured/estimated executed state. A must be
sent only after validation/acceptance; a duplicate must never execute again. Never
ACK a move just because it parsed successfully.

The serial worker has 256 bounded RX entries (reads <=1024 bytes) and 64 TX entries;
serial read/write timeouts are 20/100 ms, outside the ROS executor. Overflow or
serial failure forces reconnect, with 1 s reconnect backoff. RX overflow is counted;
partial writes have unknown outcomes and are never retried. Diagnostics include
parse/framing counts, stale channels, watchdog, pending outcomes, acquisition
counters and explicit real/mock motor support.

Firmware H is emitted on handshake/heartbeat and at least every 0.5 s while in a
session. A 1.5 s heartbeat timeout sets watchdog_expired; matching P/X restores
communication status. **Implemented:** monitoring and reporting only, because this
sketch owns no motor outputs. **Required for motor integration:** nonblocking
control, validated pin mapping, STOP in either mode, DC encoder integration,
steps/revolution and executed-step tracking, servo hold behavior, actual mode
mapping, accepted-command deduplication, fresh-session stop and watchdog motor
shutdown. Do not deploy the separate blocking/manual motor-test sketch as this
protocol firmware or claim its outputs are protected by this watchdog.

## Verification

Without ROS or third-party Python packages:

```bash
PYTHONPATH=ros/ego2sim_backend python3 -m unittest discover -s ros/ego2sim_backend/test -v
```

The suite covers malformed/partial/overlong frames, sequence rollover, invalid
calibration, time-aware filtering and resets, sonar units/saturation, limits,
bounded pending/history, deduplication, ACK vs timeout/unknown outcomes, restart
session fencing, serial reconnect generation filtering, and wire-level mock
integration with relative motion, mode changes and watchdog. A host C++ harness
compiles the actual main.ino against test doubles and checks protocol, limits,
unsupported motors, disabled pins and watchdog. It needs clang++ or g++; this is
**not** an AVR/board compilation or I2C/hardware timing verification.

With Humble sourced and both packages built:

```bash
source install/setup.bash
colcon test --packages-select ego2sim_backend --event-handlers console_direct+
colcon test-result --verbose
```

The ROS integration test runs both nodes, subscribes to all 16 public output
topics and checks uncalibrated NaNs, sonar units, accepted simulated movement,
measured flags, deduplication and actual mode feedback. It skips when rclpy is
absent. Do not run a second backend alongside this test in the same ROS domain.

On the development Mac, 15 non-ROS tests passed; 1 ROS integration test skipped.
Python compilation and colcon package discovery passed. A colcon build was
attempted and stopped at missing `ament_cmake`; Humble/rclpy and pyserial are not
in the active environment. No Arduino CLI/board target was available. Humble
build/test, real generated-message serialization/DDS, board cross-compilation,
USB/I2C tests, calibration and motor hardware remain unverified.

## Hardware references

- [Adafruit 182 long flex](https://www.adafruit.com/product/182)
- [SparkFun VL53L1X board](https://www.sparkfun.com/sparkfun-distance-sensor-breakout-4-meter-vl53l1x-qwiic.html)
- [SparkFun TEMT6000 board](https://www.sparkfun.com/sparkfun-ambient-light-sensor-breakout-temt6000.html)
- [Sharp GP2Y0A02YK datasheet](https://cdn-shop.adafruit.com/product-files/1031/GP2Y0A02YK-DATA-SHEET.PDF)
- [Pololu 723 MB1000](https://www.pololu.com/product/723)
- [MaxBotix LV-MaxSonar-EZ datasheet](https://maxbotix.com/pages/lv-maxsonar-ez-datasheet)
- [ST VL53L1X datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)
- [SparkFun driver API](https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library)
