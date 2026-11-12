# Sensors & Motors

Make sure to install Arduino library:  
  - Qwiic Distance Sensor (VL53L1X) [[LINK](https://learn.sparkfun.com/tutorials/qwiic-distance-sensor-vl53l1x-hookup-guide/arduino-library-overview)]

## Firmware (`main/main.ino`)

One sketch runs everything on the Uno: 5 sensors, 3 motors, the GUI/SENSOR mode
switch, and the serial link to the ROS 2 backend. The folders `motor_test/`,
`maxsonar_pwm/` and `temt6000/` are the standalone test sketches it was merged from.
Build it with the Servo library (built in) and the VL53L1X library above.

### Pin map

| Pin | Connection |
|---|---|
| A0 | TEMT6000 ambient light SIG |
| A1 | Flex sensor divider (flex to 5 V, 10 kΩ to GND) |
| A2 | Sharp IR Vo |
| A3 | Pushbutton to GND (internal pull-up). Debounced; each press toggles GUI/SENSOR mode |
| A4 / A5 | VL53L1X SDA / SCL |
| D2 / D3 | DC encoder A / B |
| D4, D5, D6, D7, D8 | EasyDriver MS1, MS2, EN, STEP, DIR (1/8 microstep) |
| D9 | Servo signal |
| D10 | MaxSonar PW (**moved from D6**, which is the stepper EN pin) |
| D11, D12, D13 | L298 ENA, IN2, IN1 |

### SENSOR-mode mapping

| Sensor | Motor | Mapping |
|---|---|---|
| Flex (filtered) | Servo | bend 0–90° → servo 0–180°, 2° deadband |
| Sharp IR (filtered) | DC motor | < 0.5 m: −50 RPM; > 0.9 m or nothing in range: +50 RPM; stop in between |
| VL53L1X ToF | Stepper | 0.1–1.0 m → 0–360° from the power-up position, 5° deadband |

The TEMT6000 and MaxSonar are reported only. Changing mode (button or GUI) stops all
motors. In SENSOR mode, GUI motion commands are rejected. STOP still works and pauses
sensor control of that motor until the mode changes again.

### Serial protocol (115200 baud, one comma-separated ASCII line each, `\n`-terminated)

All values use ROS units (m, rad, rad/s, lux, V). `nan` means no valid reading.
For ranges, `inf` means nothing in range and `-inf` means closer than the minimum
range (REP 117).

Uno → laptop:

| Line | Fields |
|---|---|
| `S` (20 Hz) | `S,ms,flex_adc,flex_rad,flex_rad_f,light_v,lux,lux_f,ir_v,ir_m,ir_m_f,tof_m,sonar_pulse_us,sonar_m` |
| `MS` (10 Hz per motor) | `MS,ms,motor,mode,op,enabled,moving,pos_rad,vel_rad_s,target,pos_measured,vel_measured,last_command_id,status` |
| `D` | `D,level,name,message` (0 OK / 1 WARN / 2 ERROR). Rejected commands use `name=motor_command` |
| `I` | `I,text` (boot banner, `P` reply) |

`motor`, `op`, and `mode` use the numbers in `gui/ego2sim-panel/src/contract.ts`.
For relative moves, `target` is the absolute goal position. The ultrasonic sensor
uses the PW output, so its raw signal is the pulse width in µs, not a voltage.

Laptop → Uno:

| Line | Meaning |
|---|---|
| `C,id,motor,op,target` | `MotorCommand`, passed through unchanged (target in rad or rad/s) |
| `M,mode` | `ControlMode` (0 = GUI, 1 = SENSOR) |
| `Z` | Zero the DC encoder |
| `K,counts` | Set DC encoder counts per output revolution |
| `P` | Print the encoder count and counts per revolution |
