# Ego2Sim GUI (Foxglove)

The GUI is a custom Foxglove panel plus a saved Foxglove layout. It talks to the
ROS 2 backend only through `foxglove_bridge`; it never opens the Arduino serial port.

```
Arduino Uno --USB serial--> ROS 2 backend --topics--> foxglove_bridge --WebSocket--> Foxglove
                            (serial_bridge,                                          (Ego2Sim Control
                             sensor_processing)                                       panel + plots)
```

| Folder | What it is |
|---|---|
| `ego2sim-panel/` | Foxglove extension: the **Ego2Sim Control** panel |
| `layouts/ego2sim.json` | Foxglove layout: control panel + raw-vs-filtered plots |
| `mock-server/` | Fake backend for GUI work without ROS 2 or the Uno |

## Interface contract

The panel follows *Ego2Sim ROS 2 GUI interface v2.0*, with one change: the FSR
topics are replaced by flex sensor topics, because the kit part is a Spectra Symbol
flex sensor (adafruit.com/product/182).

| Topic (prefix `/ego2sim/`) | Type | Units |
|---|---|---|
| `sensors/flex/raw` | `ego2sim_interfaces/ScalarStamped` | ADC counts |
| `sensors/flex/angle`, `sensors/flex/angle_filtered` | `ego2sim_interfaces/ScalarStamped` | bend angle, rad |

All other topics, message types and behavior are unchanged from v2.0. Topic names
live in one place: `ego2sim-panel/src/contract.ts`.

What the panel does:

- Shows each sensor in physical units (flex °, ranges cm, light lux), the filtered
  value when one is published, and the raw electrical reading.
- Shows each motor's position, velocity, target, mode and status, and marks state
  older than 1 s as stale.
- Sends `MotorCommand`s: servo absolute angle, DC relative angle or signed velocity
  (RPM), stepper relative angle (rounded to whole 1/8 microsteps, with the actual
  angle moved shown afterwards), and STOP on any motor.
- Toggles GUI/SENSOR mode with `ControlMode`. The mode shown is the one reported in
  motor feedback. Motion controls lock in SENSOR mode; STOP always works.
- Tracks each command ID until it appears as `last_command_id` in motor state, and
  lists diagnostics warnings and errors (where rejected commands show up).

## Setup on the Ubuntu 22.04 / ROS 2 Humble laptop

1. **Node.js 18 or newer.** Ubuntu 22.04's apt `nodejs` is too old. Use nvm:
   ```bash
   curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
   # open a new terminal
   nvm install --lts
   ```
2. **Foxglove desktop app.** Download the Linux `.deb` from
   https://foxglove.dev/download, then:
   ```bash
   sudo apt install ./foxglove-studio-*.deb
   ```
   Open it and sign in with a free account.
3. **foxglove_bridge:**
   ```bash
   sudo apt install ros-humble-foxglove-bridge
   ```
4. **Build and install the panel** (rerun after every panel change, then reload Foxglove
   with Ctrl+R):
   ```bash
   cd gui/ego2sim-panel
   npm ci
   npm run local-install
   ```
5. **Import the layout:** in Foxglove, open the layout menu (top bar), choose
   *Import from file*, and pick `gui/layouts/ego2sim.json`. If the control panel
   area shows as unknown, add it by hand: *Add panel* → **Ego2Sim Control**.

## Running against the real backend

Source the workspace that contains `ego2sim_interfaces` **before** starting the
bridge. Without it, the bridge cannot decode the custom messages and the panel's
commands are dropped.

```bash
source /opt/ros/humble/setup.bash
source ~/ego2sim_ws/install/setup.bash     # workspace with ego2sim_interfaces
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

When building from this repository's root, the workspace setup is
`install/setup.bash`. You can start the bridge from that root with
`./ros/scripts/start_foxglove_bridge.bash port:=8765`; it sources Humble and the
backend workspace and checks that `ego2sim_interfaces` is discoverable before
launching. For another build workspace, set `EGO2SIM_SETUP` to its
`install/setup.bash`. Source the workspace before starting **foxglove_bridge**;
the desktop application's startup order does not affect ROS type discovery.

The backend now matches the panel/mock/layout flex angle payloads:
`ego2sim_interfaces/msg/ScalarStamped`, with radians in `value` and availability in
`valid`, on both `sensors/flex/angle` and `sensors/flex/angle_filtered`. Raw flex
remains ScalarStamped ADC counts. Rebuild and restart the backend/bridge after
updating from the earlier JointState backend; the old publisher must not run on
these topics alongside the new version.

In Foxglove: *Open connection* → *Foxglove WebSocket* → `ws://localhost:8765`.

## Running against the mock backend

The mock serves the same topics and accepts the same commands on the same port, so
stop `foxglove_bridge` first. It runs on any machine with Node.js (no ROS needed).

```bash
cd gui/mock-server
npm ci
npm start
```

Then connect Foxglove to `ws://localhost:8765`. The mock prints every command it
receives. It also acts like the backend on these points:

- duplicate command IDs are ignored;
- invalid commands and motion commands in SENSOR mode are rejected via diagnostics;
- a mode change stops motion.

In SENSOR mode it drives the motors from the fake sensors with the planned mapping:

| Sensor | Motor | Mapping |
|---|---|---|
| Flex | Servo | bend 0–90° → servo 0–180° |
| Sharp IR (filtered) | DC motor | < 0.5 m reverse, > 0.9 m forward, stop in between |
| VL53L1X ToF | Stepper | 0.1–1.0 m → 0–360°, 5° deadband |

## Panel development

```bash
cd gui/ego2sim-panel
npm run build      # compile to dist/
npm run lint
npm run package    # build a .foxe file (drag it onto Foxglove to install)
```

## Current firmware integration

The ROS adapter now matches main/main.ino. Ultrasonic raw data uses
`/ego2sim/sensors/ultrasonic/raw_pulse_width` in µs. Rebuild/reinstall the panel
after updating. Sensor conversion/filtering comes from firmware by default;
flex/light/IR physical values require verified calibration flags in the backend
configuration. Command acceptance is confirmed through motor feedback; rejection
and unknown outcomes appear in diagnostics. No automatic command retries occur.
See `../ros/README.md` for the short build/start instructions.
