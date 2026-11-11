// ROS 2 interface contract between the ego2sim backend and this GUI panel.
// Source: Ego2Sim_ROS2_GUI_Interface v2.0, with the FSR topics replaced by flex
// sensor topics (sensors/flex/*), since the kit part is a Spectra Symbol flex sensor.
// If the backend renames a topic, change it here only.

export const TOPIC_PREFIX = "/ego2sim";

const t = (suffix: string): string => `${TOPIC_PREFIX}/${suffix}`;

export const TOPICS = {
  flexRaw: t("sensors/flex/raw"),
  flexAngle: t("sensors/flex/angle"),
  flexAngleFiltered: t("sensors/flex/angle_filtered"),
  lightRawVoltage: t("sensors/light/raw_voltage"),
  lightIlluminance: t("sensors/light/illuminance"),
  lightIlluminanceFiltered: t("sensors/light/illuminance_filtered"),
  irRawVoltage: t("sensors/ir/raw_voltage"),
  irRange: t("sensors/ir/range"),
  irRangeFiltered: t("sensors/ir/range_filtered"),
  tofRange: t("sensors/tof/range"),
  ultrasonicRawVoltage: t("sensors/ultrasonic/raw_voltage"),
  ultrasonicRange: t("sensors/ultrasonic/range"),
  servoState: t("motors/servo/state"),
  dcState: t("motors/dc/state"),
  stepperState: t("motors/stepper/state"),
  diagnostics: t("diagnostics"),
  motorCommand: t("motors/command"),
  modeCommand: t("control/mode_command"),
} as const;

export const SCHEMAS = {
  motorCommand: "ego2sim_interfaces/msg/MotorCommand",
  controlMode: "ego2sim_interfaces/msg/ControlMode",
} as const;

export const Motor = { SERVO: 0, DC: 1, STEPPER: 2 } as const;
export type Motor = (typeof Motor)[keyof typeof Motor];

export const Operation = { POSITION_ABSOLUTE: 0, MOVE_RELATIVE: 1, VELOCITY: 2, STOP: 3 } as const;
export type Operation = (typeof Operation)[keyof typeof Operation];

export const Mode = { GUI: 0, SENSOR: 1 } as const;
export type Mode = (typeof Mode)[keyof typeof Mode];

export const MOTOR_NAMES: Record<Motor, string> = {
  [Motor.SERVO]: "Servo",
  [Motor.DC]: "DC motor",
  [Motor.STEPPER]: "Stepper",
};

export const OPERATION_NAMES: Record<Operation, string> = {
  [Operation.POSITION_ABSOLUTE]: "absolute",
  [Operation.MOVE_RELATIVE]: "relative",
  [Operation.VELOCITY]: "velocity",
  [Operation.STOP]: "STOP",
};

export function operationName(operation: number): string {
  return (OPERATION_NAMES as Record<number, string | undefined>)[operation] ?? `op ${operation}`;
}

export function modeName(mode: number): string {
  return mode === Mode.GUI ? "GUI" : mode === Mode.SENSOR ? "SENSOR" : `mode ${mode}`;
}

// Hardware limits shown in the GUI. The backend still enforces its own limits.
export const LIMITS = {
  servoMinDeg: 0,
  servoMaxDeg: 180,
  dcMaxRpm: 100,
  dcMaxRelativeDeg: 3600,
  stepperMaxRelativeDeg: 3600,
  // SM-42BYG011-25: 200 full steps/rev; EasyDriver default 1/8 microstepping.
  stepperDegPerStep: 360 / (200 * 8),
};

// A state message older than this is shown as stale (contract: 1 s).
export const STALE_AFTER_MS = 1000;
// A command with no matching last_command_id after this is flagged.
export const ACK_TIMEOUT_MS = 2000;

// --- Message shapes (as deserialized by Foxglove) ---

export type Time = { sec: number; nanosec: number };
export type Header = { stamp: Time; frame_id: string };

export type ScalarStamped = { header: Header; value: number; valid: boolean };
export type Range = {
  header: Header;
  radiation_type: number;
  field_of_view: number;
  min_range: number;
  max_range: number;
  range: number;
};
export type Illuminance = { header: Header; illuminance: number; variance: number };

export type MotorState = {
  header: Header;
  motor: number;
  control_mode: number;
  operation: number;
  enabled: boolean;
  moving: boolean;
  position_rad: number;
  velocity_rad_s: number;
  target: number;
  position_measured: boolean;
  velocity_measured: boolean;
  last_command_id: number;
  status: string;
};

export type DiagnosticStatus = {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
  values: { key: string; value: string }[];
};
export type DiagnosticArray = { header: Header; status: DiagnosticStatus[] };

export type MotorCommand = {
  command_id: number;
  motor: Motor;
  operation: Operation;
  target: number;
};
export type ControlMode = { mode: Mode };

// Message definitions handed to Foxglove so it can CDR-encode what the panel
// publishes, even before the backend advertises these types.
const field = (name: string, type: string) => ({ name, type, isComplex: false, isArray: false });

export const PUBLISH_DATATYPES = new Map([
  [
    SCHEMAS.motorCommand,
    {
      name: SCHEMAS.motorCommand,
      definitions: [
        field("command_id", "uint32"),
        field("motor", "uint8"),
        field("operation", "uint8"),
        field("target", "float64"),
      ],
    },
  ],
  [
    SCHEMAS.controlMode,
    {
      name: SCHEMAS.controlMode,
      definitions: [field("mode", "uint8")],
    },
  ],
]);

// --- Units: ROS uses rad, rad/s, m; the GUI shows deg, RPM, cm ---

export const degToRad = (deg: number): number => (deg * Math.PI) / 180;
export const radToDeg = (rad: number): number => (rad * 180) / Math.PI;
export const rpmToRadS = (rpm: number): number => (rpm * 2 * Math.PI) / 60;
export const radSToRpm = (radS: number): number => (radS * 60) / (2 * Math.PI);
export const mToCm = (m: number): number => m * 100;
