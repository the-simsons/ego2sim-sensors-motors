// Mock ego2sim backend for developing the Foxglove panel without ROS 2 or the Uno.
//
// Speaks the Foxglove WebSocket protocol on ws://localhost:8765, the same way
// foxglove_bridge does: publishes fake /ego2sim sensor, motor state and diagnostics
// topics (CDR-encoded ROS 2 messages) and accepts MotorCommand / ControlMode
// messages from the panel. Motor behavior follows the interface contract:
// deduplicated command IDs, invalid commands rejected via diagnostics, mode
// changes stop motion, STOP works in either mode.
//
// Run: npm install && npm start   (then Foxglove: Open connection > Foxglove WebSocket)

import { FoxgloveServer } from "@foxglove/ws-protocol";
import { parse } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg2-serialization";
import { WebSocketServer } from "ws";

const PORT = Number(process.env.PORT ?? 8765);
const PREFIX = "/ego2sim";
const STATE_HZ = 10;

// --- Message definitions (ros2msg text, dependencies appended) ---

const SEP = "\n" + "=".repeat(80) + "\n";
const DEFS = {
  "builtin_interfaces/msg/Time": "int32 sec\nuint32 nanosec",
  "std_msgs/msg/Header": "builtin_interfaces/Time stamp\nstring frame_id",
  "ego2sim_interfaces/msg/ScalarStamped": "std_msgs/Header header\nfloat64 value\nbool valid",
  "sensor_msgs/msg/Range":
    "uint8 ULTRASOUND=0\nuint8 INFRARED=1\nstd_msgs/Header header\nuint8 radiation_type\n" +
    "float32 field_of_view\nfloat32 min_range\nfloat32 max_range\nfloat32 range",
  "sensor_msgs/msg/Illuminance": "std_msgs/Header header\nfloat64 illuminance\nfloat64 variance",
  "ego2sim_interfaces/msg/MotorState":
    "std_msgs/Header header\nuint8 motor\nuint8 control_mode\nuint8 operation\nbool enabled\n" +
    "bool moving\nfloat64 position_rad\nfloat64 velocity_rad_s\nfloat64 target\n" +
    "bool position_measured\nbool velocity_measured\nuint32 last_command_id\nstring status",
  "diagnostic_msgs/msg/KeyValue": "string key\nstring value",
  "diagnostic_msgs/msg/DiagnosticStatus":
    "byte OK=0\nbyte WARN=1\nbyte ERROR=2\nbyte STALE=3\nbyte level\nstring name\n" +
    "string message\nstring hardware_id\ndiagnostic_msgs/KeyValue[] values",
  "diagnostic_msgs/msg/DiagnosticArray":
    "std_msgs/Header header\ndiagnostic_msgs/DiagnosticStatus[] status",
  "ego2sim_interfaces/msg/MotorCommand":
    "uint32 command_id\nuint8 motor\nuint8 operation\nfloat64 target",
  "ego2sim_interfaces/msg/ControlMode": "uint8 mode",
};

// Full schema text for a type: its own definition, then every nested type it uses.
function schemaFor(type) {
  const deps = [];
  const visit = (name) => {
    for (const line of DEFS[name].split("\n")) {
      const fieldType = line.split(" ")[0].replace("[]", "");
      if (fieldType.includes("/")) {
        const full = fieldType.replace("/", "/msg/");
        if (!deps.includes(full)) {
          deps.push(full);
          visit(full);
        }
      }
    }
  };
  visit(type);
  return [DEFS[type], ...deps.map((dep) => `MSG: ${dep.replace("/msg/", "/")}\n${DEFS[dep]}`)].join(
    SEP,
  );
}

// --- Server setup ---

const server = new FoxgloveServer({
  name: "ego2sim-mock",
  capabilities: ["clientPublish"],
  supportedEncodings: ["cdr"],
});

const channels = new Map(); // topic -> { id, writer }

function advertise(suffix, type) {
  const schema = schemaFor(type);
  const id = server.addChannel({
    topic: `${PREFIX}/${suffix}`,
    encoding: "cdr",
    schemaName: type,
    schema,
    schemaEncoding: "ros2msg",
  });
  channels.set(suffix, { id, writer: new MessageWriter(parse(schema, { ros2: true })) });
}

function publish(suffix, message) {
  const channel = channels.get(suffix);
  const nowNs = BigInt(Date.now()) * 1_000_000n;
  server.sendMessage(channel.id, nowNs, channel.writer.writeMessage(message));
}

const SCALAR = "ego2sim_interfaces/msg/ScalarStamped";
const RANGE = "sensor_msgs/msg/Range";
for (const [suffix, type] of [
  ["sensors/flex/raw", SCALAR],
  ["sensors/flex/angle", SCALAR],
  ["sensors/flex/angle_filtered", SCALAR],
  ["sensors/light/raw_voltage", SCALAR],
  ["sensors/light/illuminance", "sensor_msgs/msg/Illuminance"],
  ["sensors/light/illuminance_filtered", "sensor_msgs/msg/Illuminance"],
  ["sensors/ir/raw_voltage", SCALAR],
  ["sensors/ir/range", RANGE],
  ["sensors/ir/range_filtered", RANGE],
  ["sensors/tof/range", RANGE],
  ["sensors/ultrasonic/raw_voltage", SCALAR],
  ["sensors/ultrasonic/range", RANGE],
  ["motors/servo/state", "ego2sim_interfaces/msg/MotorState"],
  ["motors/dc/state", "ego2sim_interfaces/msg/MotorState"],
  ["motors/stepper/state", "ego2sim_interfaces/msg/MotorState"],
  ["diagnostics", "diagnostic_msgs/msg/DiagnosticArray"],
]) {
  advertise(suffix, type);
}

const readers = {
  [`${PREFIX}/motors/command`]: new MessageReader(
    parse(schemaFor("ego2sim_interfaces/msg/MotorCommand"), { ros2: true }),
  ),
  [`${PREFIX}/control/mode_command`]: new MessageReader(
    parse(schemaFor("ego2sim_interfaces/msg/ControlMode"), { ros2: true }),
  ),
};

// --- Simulation state ---

const MOTOR = { SERVO: 0, DC: 1, STEPPER: 2 };
const OP = { POSITION_ABSOLUTE: 0, MOVE_RELATIVE: 1, VELOCITY: 2, STOP: 3 };
const MODE = { GUI: 0, SENSOR: 1 };
const DEG = Math.PI / 180;
const STEP_RAD = (360 / 1600) * DEG; // 1/8 microstepping, 200 steps/rev
const SERVO_SPEED = 315 * DEG; // HS-311 ~0.19 s / 60 deg
const STEPPER_SPEED = 180 * DEG;
const DC_MAX_SPEED = 100 * ((2 * Math.PI) / 60); // 100 RPM

let mode = MODE.GUI;
const seenIds = new Set();
const pendingDiagnostics = [];

const makeMotor = (motor, position) => ({
  motor,
  operation: OP.STOP,
  enabled: true,
  moving: false,
  position,
  velocity: 0,
  target: position,
  lastCommandId: 0,
  status: "idle",
});
const motors = [makeMotor(MOTOR.SERVO, 90 * DEG), makeMotor(MOTOR.DC, 0), makeMotor(MOTOR.STEPPER, 0)];

function stopAll(reason) {
  for (const m of motors) {
    m.operation = OP.STOP;
    m.target = m.motor === MOTOR.DC ? m.position : m.target;
    m.velocity = 0;
    m.status = reason;
  }
}

function reject(command, reason) {
  console.log(`  rejected #${command.command_id}: ${reason}`);
  pendingDiagnostics.push({
    level: 2,
    name: "motor_command",
    message: `Rejected command #${command.command_id}: ${reason}`,
    hardware_id: "ego2sim",
    values: [{ key: "command_id", value: String(command.command_id) }],
  });
}

function handleCommand(command) {
  const { command_id: id, motor: motorIndex, operation, target } = command;
  console.log(`MotorCommand #${id} motor=${motorIndex} op=${operation} target=${target.toFixed(4)}`);
  if (seenIds.has(id)) {
    console.log("  duplicate id, ignored");
    return;
  }
  seenIds.add(id);
  const m = motors[motorIndex];
  if (!m) return reject(command, `unknown motor ${motorIndex}`);
  if (!Number.isFinite(target)) return reject(command, "target is not finite");
  if (operation === OP.STOP) {
    m.operation = OP.STOP;
    m.velocity = 0;
    if (m.motor !== MOTOR.SERVO) m.target = m.position; // servo holds its position
    m.lastCommandId = id;
    m.status = "stopped by GUI";
    return;
  }
  if (mode !== MODE.GUI) return reject(command, "motion commands need GUI mode");
  const allowed = {
    [MOTOR.SERVO]: [OP.POSITION_ABSOLUTE],
    [MOTOR.DC]: [OP.MOVE_RELATIVE, OP.VELOCITY],
    [MOTOR.STEPPER]: [OP.MOVE_RELATIVE],
  }[m.motor];
  if (!allowed.includes(operation)) return reject(command, `operation ${operation} not allowed`);
  if (m.motor === MOTOR.SERVO && (target < 0 || target > Math.PI)) {
    return reject(command, "servo target outside 0-180 deg");
  }
  if (operation === OP.VELOCITY && Math.abs(target) > DC_MAX_SPEED) {
    return reject(command, "velocity above limit");
  }
  m.operation = operation;
  m.lastCommandId = id;
  if (operation === OP.POSITION_ABSOLUTE) m.target = target;
  if (operation === OP.MOVE_RELATIVE) {
    const delta = m.motor === MOTOR.STEPPER ? Math.round(target / STEP_RAD) * STEP_RAD : target;
    m.target = m.position + delta;
  }
  if (operation === OP.VELOCITY) m.target = target;
  m.status = "accepted";
}

server.on("message", ({ channel, data }) => {
  const reader = readers[channel.topic];
  if (!reader) {
    console.log(`ignored message on ${channel.topic}`);
    return;
  }
  const message = reader.readMessage(data);
  if (channel.topic.endsWith("mode_command")) {
    console.log(`ControlMode ${message.mode === MODE.SENSOR ? "SENSOR" : "GUI"}`);
    if (message.mode !== mode) {
      mode = message.mode;
      stopAll("mode changed");
    }
  } else {
    handleCommand(message);
  }
});
server.on("advertise", (channel) => {
  console.log(`client advertised ${channel.topic} (${channel.schemaName}, ${channel.encoding})`);
});
server.on("error", (err) => {
  console.error("server error:", err);
});

// --- Fake sensors ---

const noise = (amplitude) => (Math.random() * 2 - 1) * amplitude;
const ema = (previous, value, alpha) => (previous == undefined ? value : previous + alpha * (value - previous));
const filtered = {};

function header() {
  const ms = Date.now();
  return { stamp: { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 }, frame_id: "" };
}
const scalar = (value, valid = true) => ({ header: header(), value: valid ? value : NaN, valid });
const range = (radiationType, fov, min, max, value) => ({
  header: header(),
  radiation_type: radiationType,
  field_of_view: fov,
  min_range: min,
  max_range: max,
  range: value,
});

function sampleSensors(t) {
  // Flex: slow bend 0-90 deg, noisy. ADC from a 10 k divider (flex 10 k flat -> ~60 k bent).
  const flexTrue = 45 + 45 * Math.sin(t * 0.4);
  const flexAngle = flexTrue + noise(4);
  filtered.flex = ema(filtered.flex, flexAngle, 0.2);
  const flexOhms = 10_000 + (flexTrue / 90) * 50_000;
  const flexAdc = Math.round((1023 * 10_000) / (10_000 + flexOhms) + noise(3));

  // Sharp GP2Y0A02YK: 0.3-1.2 m, V ~ 61.6 / cm^1.1 approximation, noisy.
  const irTrue = 0.75 + 0.45 * Math.sin(t * 0.25);
  const irMeasured = irTrue + noise(0.04);
  filtered.ir = ema(filtered.ir, irMeasured, 0.2);
  const irVoltage = 61.6 * Math.pow(irTrue * 100, -1.1) + noise(0.03);

  // VL53L1X: hand moving 0.1-1.0 m; occasionally out of range.
  const tofTrue = 0.55 + 0.45 * Math.sin(t * 0.3 + 1);
  const tofValid = Math.random() > 0.03;

  // MB1000: 9.8 mV/in from a 5 V supply, saturates at 6 in.
  const sonar = Math.max(0.1524, 1.5 + 1.0 * Math.sin(t * 0.15) + noise(0.02));
  const sonarVoltage = (sonar / 0.0254) * 0.0098;

  // TEMT6000: room light ~300 lux with a hand shadow every ~12 s.
  const luxTrue = t % 12 < 2 ? 40 : 300 + 30 * Math.sin(t * 2);
  const luxMeasured = luxTrue + noise(15);
  filtered.lux = ema(filtered.lux, luxMeasured, 0.2);
  const lightVoltage = (luxTrue / 1000) * 5 * 0.5;

  publish("sensors/flex/raw", scalar(flexAdc));
  publish("sensors/flex/angle", scalar(flexAngle * DEG));
  publish("sensors/flex/angle_filtered", scalar(filtered.flex * DEG));
  publish("sensors/ir/raw_voltage", scalar(irVoltage));
  publish("sensors/ir/range", range(1, 0.087, 0.2, 1.5, irMeasured));
  publish("sensors/ir/range_filtered", range(1, 0.087, 0.2, 1.5, filtered.ir));
  publish("sensors/tof/range", range(1, 0.47, 0.04, 1.3, tofValid ? tofTrue + noise(0.005) : NaN));
  publish("sensors/ultrasonic/raw_voltage", scalar(sonarVoltage));
  publish("sensors/ultrasonic/range", range(0, 0.6, 0.1524, 6.4516, sonar));
  publish("sensors/light/raw_voltage", scalar(lightVoltage));
  publish("sensors/light/illuminance", { header: header(), illuminance: luxMeasured, variance: 0 });
  publish("sensors/light/illuminance_filtered", {
    header: header(),
    illuminance: filtered.lux,
    variance: 0,
  });

  return { flexDeg: filtered.flex, irM: filtered.ir, tofM: tofValid ? tofTrue : undefined };
}

// --- Motor simulation ---

function moveToward(m, speed, dt) {
  const error = m.target - m.position;
  const step = Math.sign(error) * Math.min(Math.abs(error), speed * dt);
  m.position += step;
  m.velocity = step / dt;
  m.moving = Math.abs(error) > 1e-6;
}

function stepMotors(sensors, dt) {
  const [servo, dc, stepper] = motors;
  if (mode === MODE.SENSOR) {
    // Flex 0-90 deg -> servo 0-180 deg.
    servo.target = Math.min(Math.PI, Math.max(0, sensors.flexDeg * 2 * DEG));
    servo.operation = OP.POSITION_ABSOLUTE;
    // IR: < 0.5 m reverse, > 0.9 m forward, deadband between.
    const ir = sensors.irM;
    dc.operation = OP.VELOCITY;
    dc.target = ir < 0.5 ? -0.5 * DC_MAX_SPEED : ir > 0.9 ? 0.5 * DC_MAX_SPEED : 0;
    // ToF 0.1-1.0 m -> stepper 0-360 deg, 5 deg deadband.
    if (sensors.tofM != undefined) {
      const goal = Math.round((((sensors.tofM - 0.1) / 0.9) * 2 * Math.PI) / STEP_RAD) * STEP_RAD;
      if (Math.abs(goal - stepper.target) > 5 * DEG) stepper.target = goal;
    }
    stepper.operation = OP.MOVE_RELATIVE;
    for (const m of motors) m.status = "sensor control";
  }

  if (servo.operation !== OP.STOP || mode === MODE.SENSOR) moveToward(servo, SERVO_SPEED, dt);
  else servo.moving = false;

  if (dc.operation === OP.VELOCITY) {
    dc.velocity += (dc.target - dc.velocity) * Math.min(1, dt * 5) + noise(0.05);
    dc.position += dc.velocity * dt;
    dc.moving = Math.abs(dc.velocity) > 0.05;
  } else if (dc.operation === OP.MOVE_RELATIVE) {
    moveToward(dc, DC_MAX_SPEED * 0.6, dt);
    dc.position += noise(0.002);
  } else {
    dc.velocity = 0;
    dc.moving = false;
  }

  if (stepper.operation === OP.MOVE_RELATIVE) {
    moveToward(stepper, STEPPER_SPEED, dt);
    stepper.position = Math.round(stepper.position / STEP_RAD) * STEP_RAD;
    if (!stepper.moving && mode === MODE.GUI && stepper.status === "accepted") stepper.status = "done";
  } else {
    stepper.moving = false;
    stepper.velocity = 0;
  }
  if (servo.status === "accepted" && !servo.moving) servo.status = "done";
  if (dc.status === "accepted" && dc.operation === OP.MOVE_RELATIVE && !dc.moving) dc.status = "done";
}

function publishMotorStates() {
  const names = ["servo", "dc", "stepper"];
  for (const m of motors) {
    publish(`motors/${names[m.motor]}/state`, {
      header: header(),
      motor: m.motor,
      control_mode: mode,
      operation: m.operation,
      enabled: m.enabled,
      moving: m.moving,
      position_rad: m.position,
      velocity_rad_s: m.velocity,
      target: m.target,
      // Contract: encoder DC is measured; servo is commanded; stepper is step-estimated.
      position_measured: m.motor === MOTOR.DC,
      velocity_measured: m.motor === MOTOR.DC,
      last_command_id: m.lastCommandId,
      status: m.status,
    });
  }
}

function publishDiagnostics(extra = []) {
  publish("diagnostics", {
    header: header(),
    status: [
      { level: 0, name: "serial_bridge", message: "connected (mock)", hardware_id: "uno", values: [] },
      { level: 0, name: "calibration", message: "loaded (mock)", hardware_id: "ego2sim", values: [] },
      ...extra,
    ],
  });
}

const start = Date.now();
let lastTick = start;
let lastDiag = 0;
setInterval(() => {
  const now = Date.now();
  const dt = (now - lastTick) / 1000;
  lastTick = now;
  const t = (now - start) / 1000;
  const sensors = sampleSensors(t);
  stepMotors(sensors, dt);
  publishMotorStates();
  if (pendingDiagnostics.length > 0) {
    publishDiagnostics(pendingDiagnostics.splice(0));
    lastDiag = now;
  } else if (now - lastDiag >= 1000) {
    publishDiagnostics();
    lastDiag = now;
  }
}, 1000 / STATE_HZ);

const wss = new WebSocketServer({
  port: PORT,
  handleProtocols: (protocols) => server.handleProtocols(protocols),
});
wss.on("listening", () => {
  console.log(`ego2sim mock backend on ws://localhost:${PORT}`);
});
wss.on("connection", (conn, req) => {
  const name = `${req.socket.remoteAddress}:${req.socket.remotePort}`;
  console.log(`client connected: ${name}`);
  server.handleConnection(conn, name);
});
