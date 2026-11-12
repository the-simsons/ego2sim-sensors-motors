import { PanelExtensionContext } from "@foxglove/extension";
import {
  ReactElement,
  ReactNode,
  useCallback,
  useEffect,
  useLayoutEffect,
  useRef,
  useState,
} from "react";
import { createRoot } from "react-dom/client";

import {
  ACK_TIMEOUT_MS,
  Bool,
  ControlMode,
  DiagnosticArray,
  DiagnosticStatus,
  Illuminance,
  LIMITS,
  MOTOR_NAMES,
  Mode,
  modeName,
  operationName,
  Motor,
  MotorCommand,
  MotorState,
  Operation,
  PUBLISH_DATATYPES,
  Range,
  SCHEMAS,
  STALE_AFTER_MS,
  ScalarStamped,
  TOPICS,
  degToRad,
  mToCm,
  radSToRpm,
  radToDeg,
  rpmToRadS,
} from "./contract";
import { PANEL_CSS } from "./styles";

type Entry = { message: unknown; receivedAt: number };
type Latest = ReadonlyMap<string, Entry>;
type Got<T> = { message: T; stale: boolean } | undefined;

type SentCommand = MotorCommand & {
  sentAt: number;
  // Motor position when the command was sent, used to report the actual move.
  startPositionRad?: number;
};

type CommandOutcome = { result: "accepted" | "rejected" | "unknown"; detail: string };

type LoggedDiagnostic = DiagnosticStatus & { receivedAt: number };

const SUBSCRIBED_TOPICS = Object.values(TOPICS).filter(
  (topic) =>
    topic !== TOPICS.motorCommand && topic !== TOPICS.modeCommand && topic !== TOPICS.killCommand,
);

const MOTOR_STATE_TOPICS: Record<Motor, string> = {
  [Motor.SERVO]: TOPICS.servoState,
  [Motor.DC]: TOPICS.dcState,
  [Motor.STEPPER]: TOPICS.stepperState,
};

const DIAG_LEVELS = ["OK", "WARN", "ERROR", "STALE"];

// IDs must be unique per bridge session. Seeding from the clock keeps them
// increasing across panel reloads, so the backend never sees a repeat.
function initialCommandId(): number {
  return Math.floor(Date.now() / 100) % 2 ** 32;
}

function fmt(value: number | undefined, digits = 1): string {
  return value == undefined || !Number.isFinite(value) ? "--" : value.toFixed(digits);
}

function parseNumber(text: string): number | undefined {
  const value = Number(text.trim());
  return text.trim() === "" || !Number.isFinite(value) ? undefined : value;
}

function Ego2SimPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [latest, setLatest] = useState<Latest>(new Map());
  const [ackedIds, setAckedIds] = useState<ReadonlySet<number>>(new Set());
  const [outcomes, setOutcomes] = useState<ReadonlyMap<number, CommandOutcome>>(new Map());
  const [diagLog, setDiagLog] = useState<LoggedDiagnostic[]>([]);
  const [colorScheme, setColorScheme] = useState<"dark" | "light">("dark");
  const [now, setNow] = useState(Date.now());
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const [sent, setSent] = useState<SentCommand[]>([]);
  const [requestedMode, setRequestedMode] = useState<number | undefined>();
  const [error, setError] = useState<string | undefined>();
  const nextCommandId = useRef(initialCommandId());

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
      const events = renderState.currentFrame;
      if (!events || events.length === 0) {
        return;
      }
      const receivedAt = Date.now();
      const newAcks: number[] = [];
      const newOutcomes: [number, CommandOutcome][] = [];
      const newDiags: LoggedDiagnostic[] = [];
      for (const event of events) {
        if (event.topic === TOPICS.diagnostics) {
          for (const status of (event.message as DiagnosticArray).status) {
            const values = Object.fromEntries(status.values.map(({ key, value }) => [key, value]));
            const result = values.result;
            if (values.command_id != undefined &&
                (result === "accepted" || result === "rejected" || result === "unknown")) {
              newOutcomes.push([Number(values.command_id), { result, detail: status.message }]);
            } else {
              const rejected = /^Rejected command #(\d+): (.*)$/.exec(status.message);
              if (rejected) newOutcomes.push([Number(rejected[1]), {
                result: "rejected", detail: rejected[2] ?? "rejected by firmware",
              }]);
            }
            if (status.level !== 0) {
              newDiags.push({ ...status, receivedAt });
            }
          }
        } else if (Object.values(MOTOR_STATE_TOPICS).includes(event.topic)) {
          const state = event.message as MotorState;
          if (state.last_command_id > 0 && !state.status.startsWith("UNAVAILABLE")) {
            newAcks.push(state.last_command_id);
          }
        }
      }
      setLatest((prev) => {
        const next = new Map(prev);
        for (const event of events) {
          next.set(event.topic, { message: event.message, receivedAt });
        }
        return next;
      });
      if (newOutcomes.length > 0) {
        setOutcomes((prev) => new Map([...prev, ...newOutcomes].slice(-128)));
      }
      if (newAcks.length > 0) {
        setAckedIds((prev) => {
          if (newAcks.every((id) => prev.has(id))) {
            return prev;
          }
          return new Set([...prev, ...newAcks].slice(-128));
        });
      }
      if (newDiags.length > 0) {
        setDiagLog((prev) => [...newDiags.reverse(), ...prev].slice(0, 12));
      }
    };
    context.watch("currentFrame");
    context.watch("colorScheme");
    context.subscribe(SUBSCRIBED_TOPICS.map((topic) => ({ topic })));
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Re-render periodically so stale and ack-timeout indicators update with no new data.
  useEffect(() => {
    const timer = setInterval(() => {
      setNow(Date.now());
    }, 250);
    return () => {
      clearInterval(timer);
    };
  }, []);

  useEffect(() => {
    context.advertise?.(TOPICS.motorCommand, SCHEMAS.motorCommand, {
      datatypes: PUBLISH_DATATYPES,
    });
    context.advertise?.(TOPICS.modeCommand, SCHEMAS.controlMode, {
      datatypes: PUBLISH_DATATYPES,
    });
    context.advertise?.(TOPICS.killCommand, SCHEMAS.bool, {
      datatypes: PUBLISH_DATATYPES,
    });
    return () => {
      context.unadvertise?.(TOPICS.motorCommand);
      context.unadvertise?.(TOPICS.modeCommand);
      context.unadvertise?.(TOPICS.killCommand);
    };
  }, [context]);

  const get = useCallback(
    (topic: string): Got<unknown> => {
      const entry = latest.get(topic);
      if (!entry) {
        return undefined;
      }
      return { message: entry.message, stale: now - entry.receivedAt > STALE_AFTER_MS };
    },
    [latest, now],
  );

  const motorState = (motor: Motor): Got<MotorState> => {
    const entry = get(MOTOR_STATE_TOPICS[motor]) as Got<MotorState>;
    if (!entry) return undefined;
    return { ...entry, stale: entry.stale || entry.message.status.startsWith("UNAVAILABLE") };
  };

  const sendCommand = (motor: Motor, operation: Operation, target: number) => {
    if (!context.publish) {
      setError("This data source does not accept commands. Connect with Foxglove WebSocket.");
      return;
    }
    const feedback = motorState(motor);
    if (operation !== Operation.STOP && killed !== false) {
      setError("Kill switch engaged (or state unknown): only STOP is allowed.");
      return;
    }
    if (operation !== Operation.STOP &&
        (!feedback || feedback.stale || feedback.message.control_mode !== Mode.GUI)) {
      setError("Motion requires fresh motor feedback in GUI mode.");
      return;
    }
    if (nextCommandId.current > 0xffffffff) {
      setError("Command IDs exhausted; start a new coordinated backend/GUI session.");
      return;
    }
    const command: MotorCommand = {
      command_id: nextCommandId.current,
      motor,
      operation,
      target,
    };
    nextCommandId.current += 1;
    try {
      context.publish(TOPICS.motorCommand, command);
      setError(undefined);
    } catch (err) {
      setError(`Publish failed: ${String(err)}`);
      return;
    }
    const startPositionRad = motorState(motor)?.message.position_rad;
    setSent((prev) => [{ ...command, sentAt: Date.now(), startPositionRad }, ...prev].slice(0, 20));
  };

  const sendMode = (mode: Mode) => {
    if (!context.publish) {
      setError("This data source does not accept commands. Connect with Foxglove WebSocket.");
      return;
    }
    try {
      const message: ControlMode = { mode };
      context.publish(TOPICS.modeCommand, message);
      setRequestedMode(mode);
      setError(undefined);
    } catch (err) {
      setError(`Publish failed: ${String(err)}`);
    }
  };

  const sendKill = (message: Bool) => {
    if (!context.publish) {
      setError("This data source does not accept commands. Connect with Foxglove WebSocket.");
      return;
    }
    try {
      context.publish(TOPICS.killCommand, message);
      setError(undefined);
    } catch (err) {
      setError(`Publish failed: ${String(err)}`);
    }
  };

  // Actual mode comes from motor feedback, never from what the GUI last sent.
  const reportedModes = [Motor.SERVO, Motor.DC, Motor.STEPPER]
    .map((motor) => motorState(motor))
    .filter((state) => state != undefined && !state.stale)
    .map((state) => state!.message.control_mode);
  const actualMode =
    reportedModes.length === 0
      ? undefined
      : reportedModes.every((mode) => mode === reportedModes[0])
        ? reportedModes[0]
        : "mixed";
  const modePending = requestedMode != undefined && actualMode !== requestedMode;
  // Kill state comes only from firmware (KS lines); undefined = unknown or stale.
  const killEntry = get(TOPICS.killState) as Got<Bool>;
  const killed = killEntry && !killEntry.stale ? killEntry.message.data : undefined;
  const guiControl = actualMode === Mode.GUI && killed === false;

  const lastSent = (motor: Motor) => sent.find((command) => command.motor === motor);

  const commandStatus = (command: SentCommand): { label: string; tone: string } => {
    const outcome = outcomes.get(command.command_id);
    if (outcome) {
      return { label: outcome.result === "unknown" ? "unconfirmed; outcome unknown" : outcome.result,
               tone: outcome.result === "accepted" ? "ok" : "error" };
    }
    if (ackedIds.has(command.command_id)) {
      return { label: "accepted", tone: "ok" };
    }
    if (now - command.sentAt > ACK_TIMEOUT_MS) {
      return { label: "unconfirmed; outcome unknown", tone: "error" };
    }
    return { label: "pending", tone: "warn" };
  };

  const diagnostics = get(TOPICS.diagnostics) as Got<DiagnosticArray>;

  return (
    <div className={`e2s e2s-${colorScheme}`}>
      <style>{PANEL_CSS}</style>

      <header className="e2s-header">
        <div className="e2s-title">Ego2Sim control</div>
        <div className="e2s-mode">
          <span className="e2s-muted">Control mode</span>
          <div className="e2s-segmented" role="group" aria-label="Control mode">
            <button
              className={actualMode === Mode.GUI ? "active" : ""}
              onClick={() => {
                sendMode(Mode.GUI);
              }}
            >
              GUI
            </button>
            <button
              className={actualMode === Mode.SENSOR ? "active" : ""}
              onClick={() => {
                sendMode(Mode.SENSOR);
              }}
            >
              SENSOR
            </button>
          </div>
          <span className="e2s-muted">
            {actualMode == undefined
              ? "no motor feedback"
              : actualMode === "mixed"
                ? "motors disagree"
                : `reported: ${modeName(actualMode)}`}
            {modePending ? ` (requested ${modeName(requestedMode)}…)` : ""}
          </span>
        </div>
      </header>

      <div className={`e2s-kill ${killed === false ? "armed" : "killed"}`}>
        <div>
          <div className="e2s-kill-state">
            {killed == undefined ? "KILL SWITCH: UNKNOWN" : killed ? "MOTORS KILLED" : "MOTORS ENABLED"}
          </div>
          <div className="e2s-small">
            {killed == undefined
              ? "No kill switch state from the firmware. Motion is locked."
              : "The A3 button and this switch both toggle it."}
          </div>
        </div>
        {killed === true ? (
          <button className="e2s-kill-btn release" onClick={() => { sendKill({ data: false }); }}>
            RE-ENABLE
          </button>
        ) : (
          <button className="e2s-kill-btn kill" onClick={() => { sendKill({ data: true }); }}>
            KILL
          </button>
        )}
      </div>

      {error && <div className="e2s-banner error">{error}</div>}
      {!context.publish && (
        <div className="e2s-banner warn">
          Read-only data source: commands are disabled. Connect with Foxglove WebSocket to
          foxglove_bridge.
        </div>
      )}

      <section className="e2s-sensors">
        <SensorTile
          label="Flex"
          unit="°"
          drives="Servo"
          filtered={scalarDeg(get(TOPICS.flexAngleFiltered) as Got<ScalarStamped>)}
          unfiltered={scalarDeg(get(TOPICS.flexAngle) as Got<ScalarStamped>)}
          raw={rawLine(get(TOPICS.flexRaw) as Got<ScalarStamped>, "ADC", 0)}
        />
        <SensorTile
          label="Sharp IR"
          unit="cm"
          drives="DC motor"
          filtered={rangeCm(get(TOPICS.irRangeFiltered) as Got<Range>)}
          unfiltered={rangeCm(get(TOPICS.irRange) as Got<Range>)}
          raw={rawLine(get(TOPICS.irRawVoltage) as Got<ScalarStamped>, "V", 3)}
        />
        <SensorTile
          label="ToF"
          unit="cm"
          drives="Stepper"
          unfiltered={rangeCm(get(TOPICS.tofRange) as Got<Range>)}
        />
        <SensorTile
          label="Ultrasonic"
          unit="cm"
          unfiltered={rangeCm(get(TOPICS.ultrasonicRange) as Got<Range>)}
          raw={rawLine(get(TOPICS.ultrasonicRawPulseWidth) as Got<ScalarStamped>, "µs", 0)}
        />
        <SensorTile
          label="Light"
          unit="lux"
          filtered={lux(get(TOPICS.lightIlluminanceFiltered) as Got<Illuminance>)}
          unfiltered={lux(get(TOPICS.lightIlluminance) as Got<Illuminance>)}
          raw={rawLine(get(TOPICS.lightRawVoltage) as Got<ScalarStamped>, "V", 3)}
        />
      </section>

      <section className="e2s-motors">
        <MotorCard motor={Motor.SERVO} state={motorState(Motor.SERVO)}>
          <ServoControls
            disabled={!guiControl}
            onSend={(deg) => {
              sendCommand(Motor.SERVO, Operation.POSITION_ABSOLUTE, degToRad(deg));
            }}
          />
          <CommandLine command={lastSent(Motor.SERVO)} status={commandStatus} />
          <StopButton
            onStop={() => {
              sendCommand(Motor.SERVO, Operation.STOP, 0);
            }}
          />
        </MotorCard>

        <MotorCard motor={Motor.DC} state={motorState(Motor.DC)}>
          <DcControls
            disabled={!guiControl}
            onRelative={(deg) => {
              sendCommand(Motor.DC, Operation.MOVE_RELATIVE, degToRad(deg));
            }}
            onVelocity={(rpm) => {
              sendCommand(Motor.DC, Operation.VELOCITY, rpmToRadS(rpm));
            }}
          />
          <CommandLine command={lastSent(Motor.DC)} status={commandStatus} />
          <StopButton
            onStop={() => {
              sendCommand(Motor.DC, Operation.STOP, 0);
            }}
          />
        </MotorCard>

        <MotorCard motor={Motor.STEPPER} state={motorState(Motor.STEPPER)}>
          <StepperControls
            disabled={!guiControl}
            onSend={(deg) => {
              sendCommand(Motor.STEPPER, Operation.MOVE_RELATIVE, degToRad(deg));
            }}
          />
          <StepperResult
            command={lastSent(Motor.STEPPER)}
            state={motorState(Motor.STEPPER)?.stale ? undefined : motorState(Motor.STEPPER)?.message}
          />
          <CommandLine command={lastSent(Motor.STEPPER)} status={commandStatus} />
          <StopButton
            onStop={() => {
              sendCommand(Motor.STEPPER, Operation.STOP, 0);
            }}
          />
        </MotorCard>
      </section>

      <section className="e2s-lower">
        <div className="e2s-card">
          <div className="e2s-card-title">Commands sent</div>
          {sent.length === 0 && <div className="e2s-muted">None yet</div>}
          <table className="e2s-table">
            <tbody>
              {sent.slice(0, 8).map((command) => {
                const status = commandStatus(command);
                return (
                  <tr key={command.command_id}>
                    <td className="e2s-mono">#{command.command_id}</td>
                    <td>{MOTOR_NAMES[command.motor]}</td>
                    <td>{operationName(command.operation)}</td>
                    <td className="e2s-mono">{describeTarget(command)}</td>
                    <td className={`e2s-tone-${status.tone}`}>{status.label}</td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>

        <div className="e2s-card">
          <div className="e2s-card-title">
            Diagnostics{" "}
            {diagnostics == undefined ? (
              <span className="e2s-muted">(none received)</span>
            ) : diagnostics.stale ? (
              <span className="e2s-tone-warn">(stale)</span>
            ) : undefined}
          </div>
          {diagnostics?.message.status.map((status) => (
            <div key={status.name} className="e2s-diag">
              <span className={`e2s-pill e2s-level-${status.level}`}>
                {DIAG_LEVELS[status.level] ?? status.level}
              </span>
              <span className="e2s-diag-name">{status.name}</span>
              <span className="e2s-muted">{status.message}</span>
            </div>
          ))}
          {diagLog.length > 0 && <div className="e2s-subtitle">Recent warnings and errors</div>}
          {diagLog.map((status, index) => (
            <div key={`${status.receivedAt}-${index}`} className="e2s-diag">
              <span className="e2s-muted e2s-mono">
                {new Date(status.receivedAt).toLocaleTimeString()}
              </span>
              <span className={`e2s-pill e2s-level-${status.level}`}>
                {DIAG_LEVELS[status.level] ?? status.level}
              </span>
              <span className="e2s-diag-name">{status.name}</span>
              <span className="e2s-muted">{status.message}</span>
            </div>
          ))}
        </div>
      </section>
    </div>
  );
}

// --- Sensor readouts ---

type Reading = { value?: number; stale: boolean; invalid: boolean; note?: string } | undefined;

function scalarDeg(entry: { message: ScalarStamped; stale: boolean } | undefined): Reading {
  if (!entry) {
    return undefined;
  }
  const { value, valid } = entry.message;
  const invalid = !valid || !Number.isFinite(value);
  return { value: invalid ? undefined : radToDeg(value), stale: entry.stale, invalid };
}

function rangeCm(entry: { message: Range; stale: boolean } | undefined): Reading {
  if (!entry) {
    return undefined;
  }
  if (entry.message.range === Infinity || entry.message.range === -Infinity) {
    return { stale: entry.stale, invalid: false,
             note: entry.message.range > 0 ? "beyond range" : "closer than minimum" };
  }
  const invalid = !Number.isFinite(entry.message.range);
  return { value: invalid ? undefined : mToCm(entry.message.range), stale: entry.stale, invalid };
}

function lux(entry: { message: Illuminance; stale: boolean } | undefined): Reading {
  if (!entry) {
    return undefined;
  }
  const invalid = !Number.isFinite(entry.message.illuminance);
  return { value: invalid ? undefined : entry.message.illuminance, stale: entry.stale, invalid };
}

function rawLine(
  entry: { message: ScalarStamped; stale: boolean } | undefined,
  unit: string,
  digits: number,
): string | undefined {
  if (!entry) {
    return undefined;
  }
  const { value, valid } = entry.message;
  return valid && Number.isFinite(value) ? `raw ${value.toFixed(digits)} ${unit}` : "raw invalid";
}

function SensorTile(props: {
  label: string;
  unit: string;
  drives?: string;
  filtered?: Reading;
  unfiltered?: Reading;
  raw?: string;
}): ReactElement {
  const { label, unit, drives, filtered, unfiltered, raw } = props;
  // Show the filtered value when the backend publishes one; otherwise the unfiltered value.
  const main = filtered ?? unfiltered;
  const tone = main == undefined ? "missing" : main.stale ? "warn" : main.invalid ? "error" : "ok";
  const note =
    main == undefined ? "no data" : main.stale ? "stale" : main.invalid ? "invalid" : main.note;
  return (
    <div className="e2s-card e2s-sensor">
      <div className="e2s-sensor-head">
        <span className="e2s-card-title">{label}</span>
        {drives && <span className="e2s-muted">→ {drives}</span>}
      </div>
      <div className={`e2s-sensor-value e2s-tone-${tone}`}>
        {fmt(main?.value)}
        <span className="e2s-unit">{unit}</span>
      </div>
      <div className="e2s-muted e2s-small">
        {note ?? (filtered ? "filtered" : "unfiltered")}
        {filtered && unfiltered && ` · unfiltered ${fmt(unfiltered.value)} ${unit}`}
      </div>
      {raw && <div className="e2s-muted e2s-small e2s-mono">{raw}</div>}
    </div>
  );
}

// --- Motor cards ---

function MotorCard(props: {
  motor: Motor;
  state: { message: MotorState; stale: boolean } | undefined;
  children: ReactNode;
}): ReactElement {
  const { motor, state, children } = props;
  const msg = state?.message;
  const isVelocity = msg?.operation === Operation.VELOCITY;
  return (
    <div className="e2s-card e2s-motor">
      <div className="e2s-motor-head">
        <span className="e2s-card-title">{MOTOR_NAMES[motor]}</span>
        {msg == undefined ? (
          <span className="e2s-pill e2s-level-3">no state</span>
        ) : (
          <>
            {state!.stale && <span className="e2s-pill e2s-level-1">stale</span>}
            <span className="e2s-pill">{modeName(msg.control_mode)}</span>
            <span className={`e2s-pill ${msg.enabled ? "" : "e2s-level-2"}`}>
              {msg.enabled ? "enabled" : "disabled"}
            </span>
            {msg.moving && <span className="e2s-pill e2s-level-1">moving</span>}
          </>
        )}
      </div>
      <dl className="e2s-readout">
        <dt>Position</dt>
        <dd>
          {fmt(msg && radToDeg(msg.position_rad))}°
          {msg && !msg.position_measured && <span className="e2s-muted"> (estimated)</span>}
        </dd>
        <dt>Velocity</dt>
        <dd>
          {fmt(msg && radSToRpm(msg.velocity_rad_s))} RPM
          {msg && !msg.velocity_measured && <span className="e2s-muted"> (estimated)</span>}
        </dd>
        <dt>Target</dt>
        <dd>
          {msg == undefined
            ? "--"
            : isVelocity
              ? `${fmt(radSToRpm(msg.target))} RPM`
              : `${fmt(radToDeg(msg.target))}°`}
          {msg && <span className="e2s-muted"> ({operationName(msg.operation)})</span>}
        </dd>
        <dt>Status</dt>
        <dd>{msg?.status ?? "--"}</dd>
      </dl>
      {children}
    </div>
  );
}

function NumberField(props: {
  label: string;
  value: string;
  unit: string;
  disabled?: boolean;
  onChange: (text: string) => void;
  onSubmit: () => void;
}): ReactElement {
  return (
    <label className="e2s-field">
      <span>{props.label}</span>
      <input
        type="number"
        value={props.value}
        disabled={props.disabled}
        onChange={(event) => {
          props.onChange(event.target.value);
        }}
        onKeyDown={(event) => {
          if (event.key === "Enter") {
            props.onSubmit();
          }
        }}
      />
      <span className="e2s-muted">{props.unit}</span>
    </label>
  );
}

function ServoControls(props: { disabled: boolean; onSend: (deg: number) => void }): ReactElement {
  const [deg, setDeg] = useState("90");
  const value = parseNumber(deg);
  const valid = value != undefined && value >= LIMITS.servoMinDeg && value <= LIMITS.servoMaxDeg;
  const send = (target: number) => {
    setDeg(String(target));
    props.onSend(target);
  };
  return (
    <div className="e2s-controls">
      <input
        type="range"
        className="e2s-slider"
        min={LIMITS.servoMinDeg}
        max={LIMITS.servoMaxDeg}
        step={1}
        value={value ?? 90}
        disabled={props.disabled}
        onChange={(event) => {
          setDeg(event.target.value);
        }}
        // Send once on release rather than flooding the backend while dragging.
        onPointerUp={(event) => {
          send(Number(event.currentTarget.value));
        }}
        onKeyUp={(event) => {
          send(Number(event.currentTarget.value));
        }}
      />
      <div className="e2s-row">
        <NumberField
          label="Angle"
          value={deg}
          unit="°"
          disabled={props.disabled}
          onChange={setDeg}
          onSubmit={() => {
            if (valid) {
              send(value);
            }
          }}
        />
        <button
          disabled={props.disabled || !valid}
          onClick={() => {
            if (valid) {
              send(value);
            }
          }}
        >
          Go
        </button>
      </div>
      <div className="e2s-row">
        {[LIMITS.servoMinDeg, 90, LIMITS.servoMaxDeg].map((preset) => (
          <button
            key={preset}
            disabled={props.disabled}
            onClick={() => {
              send(preset);
            }}
          >
            {preset}°
          </button>
        ))}
      </div>
      {!valid && (
        <div className="e2s-tone-error e2s-small">
          Enter {LIMITS.servoMinDeg}–{LIMITS.servoMaxDeg}°
        </div>
      )}
    </div>
  );
}

function DcControls(props: {
  disabled: boolean;
  onRelative: (deg: number) => void;
  onVelocity: (rpm: number) => void;
}): ReactElement {
  const [deg, setDeg] = useState("90");
  const [rpm, setRpm] = useState("30");
  const degValue = parseNumber(deg);
  const rpmValue = parseNumber(rpm);
  const degValid = degValue != undefined && Math.abs(degValue) <= LIMITS.dcMaxRelativeDeg;
  const rpmValid = rpmValue != undefined && Math.abs(rpmValue) <= LIMITS.dcMaxRpm;
  return (
    <div className="e2s-controls">
      <div className="e2s-row">
        <NumberField
          label="Move by"
          value={deg}
          unit="°"
          disabled={props.disabled}
          onChange={setDeg}
          onSubmit={() => {
            if (degValid) {
              props.onRelative(degValue);
            }
          }}
        />
        <button
          disabled={props.disabled || !degValid}
          onClick={() => {
            if (degValid) {
              props.onRelative(degValue);
            }
          }}
        >
          Move
        </button>
      </div>
      <input
        type="range"
        className="e2s-slider"
        min={-LIMITS.dcMaxRpm}
        max={LIMITS.dcMaxRpm}
        step={1}
        value={rpmValue ?? 0}
        disabled={props.disabled}
        onChange={(event) => {
          setRpm(event.target.value);
        }}
      />
      <div className="e2s-row">
        <NumberField
          label="Velocity"
          value={rpm}
          unit="RPM (− = reverse)"
          disabled={props.disabled}
          onChange={setRpm}
          onSubmit={() => {
            if (rpmValid) {
              props.onVelocity(rpmValue);
            }
          }}
        />
        <button
          disabled={props.disabled || !rpmValid}
          onClick={() => {
            if (rpmValid) {
              props.onVelocity(rpmValue);
            }
          }}
        >
          Set
        </button>
      </div>
      {(!degValid || !rpmValid) && (
        <div className="e2s-tone-error e2s-small">
          Limits: ±{LIMITS.dcMaxRelativeDeg}°, ±{LIMITS.dcMaxRpm} RPM
        </div>
      )}
    </div>
  );
}

function stepperQuantize(deg: number): { steps: number; deg: number } {
  const steps = Math.round(deg / LIMITS.stepperDegPerStep);
  return { steps, deg: steps * LIMITS.stepperDegPerStep };
}

function StepperControls(props: {
  disabled: boolean;
  onSend: (deg: number) => void;
}): ReactElement {
  const [deg, setDeg] = useState("90");
  const value = parseNumber(deg);
  const valid = value != undefined && Math.abs(value) <= LIMITS.stepperMaxRelativeDeg;
  const quantized = valid ? stepperQuantize(value) : undefined;
  const send = (target: number) => {
    // Send the angle rounded to a whole step so the GUI and backend agree on it.
    props.onSend(stepperQuantize(target).deg);
  };
  return (
    <div className="e2s-controls">
      <div className="e2s-row">
        <NumberField
          label="Move by"
          value={deg}
          unit="°"
          disabled={props.disabled}
          onChange={setDeg}
          onSubmit={() => {
            if (valid) {
              send(value);
            }
          }}
        />
        <button
          disabled={props.disabled || !valid}
          onClick={() => {
            if (valid) {
              send(value);
            }
          }}
        >
          Move
        </button>
      </div>
      <div className="e2s-row">
        {[-90, -15, 15, 90].map((preset) => (
          <button
            key={preset}
            disabled={props.disabled}
            onClick={() => {
              send(preset);
            }}
          >
            {preset > 0 ? "+" : ""}
            {preset}°
          </button>
        ))}
      </div>
      <div className="e2s-muted e2s-small">
        {quantized
          ? `Rounds to ${quantized.deg.toFixed(3)}° (${quantized.steps} steps of ${LIMITS.stepperDegPerStep}°)`
          : `Enter up to ±${LIMITS.stepperMaxRelativeDeg}°`}
      </div>
    </div>
  );
}

function StepperResult(props: {
  command: SentCommand | undefined;
  state: MotorState | undefined;
}): ReactElement {
  const { command, state } = props;
  if (
    command?.operation !== Operation.MOVE_RELATIVE ||
    command.startPositionRad == undefined ||
    state == undefined ||
    state.last_command_id !== command.command_id ||
    !Number.isFinite(state.position_rad)
  ) {
    return <></>;
  }
  const moved = radToDeg(state.position_rad - command.startPositionRad);
  return (
    <div className="e2s-result">
      Last move: requested {radToDeg(command.target).toFixed(3)}°, actual <b>{moved.toFixed(3)}°</b>{" "}
      {state.moving ? "(moving…)" : ""}
    </div>
  );
}

function describeTarget(command: MotorCommand): string {
  switch (command.operation) {
    case Operation.STOP:
      return "";
    case Operation.VELOCITY:
      return `${radSToRpm(command.target).toFixed(1)} RPM`;
    case Operation.POSITION_ABSOLUTE:
    case Operation.MOVE_RELATIVE:
      return `${radToDeg(command.target).toFixed(1)}°`;
  }
}

function CommandLine(props: {
  command: SentCommand | undefined;
  status: (command: SentCommand) => { label: string; tone: string };
}): ReactElement {
  if (!props.command) {
    return <div className="e2s-muted e2s-small">No command sent yet</div>;
  }
  const status = props.status(props.command);
  return (
    <div className="e2s-small">
      Last command <span className="e2s-mono">#{props.command.command_id}</span>{" "}
      {operationName(props.command.operation)} {describeTarget(props.command)}:{" "}
      <span className={`e2s-tone-${status.tone}`}>{status.label}</span>
    </div>
  );
}

function StopButton(props: { onStop: () => void }): ReactElement {
  // STOP works in either control mode, so it is never disabled.
  return (
    <button className="e2s-stop" onClick={props.onStop}>
      STOP
    </button>
  );
}

export function initEgo2SimPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<Ego2SimPanel context={context} />);
  return () => {
    root.unmount();
  };
}
