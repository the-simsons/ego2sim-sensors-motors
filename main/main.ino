// ego2sim unified firmware: 5 sensors, 3 motors, GUI/SENSOR mode, serial link to ROS 2.
//
// Merged from the per-part test sketches:
//   main/main.ino (flex, Sharp IR, VL53L1X), temt6000/temt6000.ino (light + filter),
//   maxsonar_pwm/maxsonar_pwm.ino (ultrasonic), motor_test/motor_test.ino (servo,
//   stepper, DC motor with encoder PID).
//
// Everything in loop() is non-blocking so the DC PID loop and the stepper keep running
// while sensors are read and telemetry is sent. The serial protocol is documented in
// the repository README ("Firmware serial protocol").
//
// Pin map (Arduino Uno)
//   A0  TEMT6000 ambient light SIG
//   A1  Flex sensor divider (flex to 5 V, 10 k to GND)
//   A2  Sharp IR rangefinder Vo
//   A3  Pushbutton to GND (internal pull-up), toggles GUI/SENSOR mode
//   A4/A5  VL53L1X ToF (I2C SDA/SCL)
//   D2/D3  DC motor encoder A/B (interrupts)
//   D4/D5  EasyDriver MS1/MS2      D6 EasyDriver EN
//   D7  EasyDriver STEP            D8 EasyDriver DIR
//   D9  Servo signal (Servo library disables PWM on D9/D10)
//   D10 MaxSonar PW output (moved from D6, which is the stepper EN pin)
//   D11 L298 ENA (PWM)  D12 L298 IN2  D13 L298 IN1

#include <Wire.h>
#include <Servo.h>
#include <SparkFun_VL53L1X.h>
#include <math.h>

// ===================== Pins =====================
const uint8_t PIN_LIGHT = A0;
const uint8_t PIN_FLEX = A1;
const uint8_t PIN_IR = A2;
const uint8_t PIN_BUTTON = A3;
const uint8_t PIN_ENC_A = 2;
const uint8_t PIN_ENC_B = 3;
const uint8_t PIN_MS1 = 4;
const uint8_t PIN_MS2 = 5;
const uint8_t PIN_STEP_EN = 6;
const uint8_t PIN_STEP = 7;
const uint8_t PIN_STEP_DIR = 8;
const uint8_t PIN_SERVO = 9;
const uint8_t PIN_SONAR_PW = 10;  // must stay on D10: the PCINT0 handler reads PB2
const uint8_t PIN_DC_ENA = 11;
const uint8_t PIN_DC_IN2 = 12;
const uint8_t PIN_DC_IN1 = 13;

// ===================== Protocol enums (match gui/ego2sim-panel/src/contract.ts) ======
enum : uint8_t { MOTOR_SERVO = 0, MOTOR_DC = 1, MOTOR_STEPPER = 2, MOTOR_COUNT = 3 };
enum : uint8_t { OP_POSITION_ABSOLUTE = 0, OP_MOVE_RELATIVE = 1, OP_VELOCITY = 2, OP_STOP = 3 };
enum : uint8_t { MODE_GUI = 0, MODE_SENSOR = 1 };

// ===================== Timing =====================
const unsigned long SAMPLE_PERIOD_MS = 20;      // analog sensors at 50 Hz
const unsigned long TOF_POLL_MS = 20;           // VL53L1X data-ready poll
const unsigned long SENSOR_CTRL_PERIOD_MS = 50; // sensor -> motor mapping at 20 Hz
const unsigned long SENSOR_TX_PERIOD_MS = 50;   // "S" line at 20 Hz
const unsigned long MOTOR_TX_PERIOD_MS = 100;   // "MS" lines at 10 Hz
const unsigned long HEALTH_TX_PERIOD_MS = 2000; // "D" health lines
const unsigned long DEBOUNCE_MS = 30;

// ===================== Sensor calibration =====================
const float ADC_REF_V = 5.0;
const float FLEX_FIXED_OHM = 10000.0;
// CALIBRATE: flex resistance when flat and when bent 90 deg (linear fit between them).
const float FLEX_R_FLAT_OHM = 10000.0;
const float FLEX_R_90_OHM = 30000.0;

// TEMT6000 with its 10 k load: 0.5 uA per lux -> lux = V / 10k * 1e6 * 2 = 200 * V.
const float LIGHT_LUX_PER_V = 200.0;

// Sharp IR power-law fit: distance_cm = IR_K * V^IR_EXP.
// Set to 0 if the kit part is the short-range GP2Y0A21YK (10-80 cm).
#define IR_IS_GP2Y0A02YK 1
#if IR_IS_GP2Y0A02YK
const float IR_K = 60.374, IR_EXP = -1.16, IR_MIN_M = 0.20, IR_MAX_M = 1.50;
#else
const float IR_K = 29.988, IR_EXP = -1.173, IR_MIN_M = 0.10, IR_MAX_M = 0.80;
#endif

// LV-MaxSonar-EZ PW output: 147 us per inch. No pulse for this long = no reading.
const float SONAR_US_PER_INCH = 147.0;
const unsigned long SONAR_TIMEOUT_MS = 200;
const unsigned long TOF_STALE_MS = 500;

// Moving average + exponential moving average (the TEMT6000 filter, used on all analog sensors).
const uint8_t FILTER_N = 10;
const float FILTER_ALPHA = 0.2;

// ===================== Sensor -> motor mapping (SENSOR mode) =====================
// Flex 0-90 deg -> servo 0-180 deg.
const float SERVO_PER_FLEX = 2.0;
const float SERVO_SENSOR_DEADBAND_DEG = 2.0;
// Sharp IR (filtered): closer than 0.5 m reverse, farther than 0.9 m forward, stop between.
const float IR_REVERSE_BELOW_M = 0.5;
const float IR_FORWARD_ABOVE_M = 0.9;
const float DC_SENSOR_RPM = 50.0;
// VL53L1X 0.1-1.0 m -> stepper 0-360 deg (absolute from power-up), 5 deg deadband.
const float TOF_MIN_M = 0.1;
const float TOF_MAX_M = 1.0;
const float STEPPER_SENSOR_DEADBAND_DEG = 5.0;

// ===================== Servo (HS-311) =====================
const float SERVO_SLEW_DEG_S = 300.0;  // ~0.19 s / 60 deg, used to estimate position

// ===================== Stepper (SM-42BYG011-25 + EasyDriver, 1/8 microstep) =========
const float STEPPER_DEG_PER_STEP = 360.0 / (200 * 8);
const unsigned int STEP_INTERVAL_US = 1000;  // 225 deg/s
const float STEPPER_MAX_REL_DEG = 3600.0;

// ===================== DC motor + encoder (tuned in motor_test) =====================
// Counts per OUTPUT-SHAFT revolution with 4x decoding. Send K,<counts> to change at runtime.
float countsPerRev = 360.0;
#define ENC_DIR 1  // set to -1 if a forward move counts down

#define CTRL_DT_MS     40
#define POS_KP         2.0
#define POS_KD         0.5
#define POS_KI         2.0
#define POS_I_MAX      150
#define POS_MAX_PWM    200
#define POS_TOL_DEG    1.5
#define POS_TIMEOUT_MS 15000
#define VEL_KP         0.5
#define VEL_KI         3.0
#define lower_limit_dc 0
const float DC_MAX_RPM = 100.0;
const float DC_MAX_REL_DEG = 3600.0;

// =====================================================================================
// Non-blocking serial output. Serial.print blocks once the 64-byte hardware buffer is
// full, which would stall the stepper and PID loop, so lines are queued here and
// drained only as fast as the UART accepts them.
// =====================================================================================
const uint16_t TX_BUF = 320;
const uint8_t TX_LINE_MAX = 128;  // longest line we ever compose

class TxQueue : public Print {
 public:
  size_t write(uint8_t c) override {
    uint16_t next = (head_ + 1) % TX_BUF;
    if (next == tail_) return 0;  // full; callers check room() first so this is rare
    buf_[head_] = c;
    head_ = next;
    return 1;
  }
  using Print::write;
  uint16_t room() const { return (tail_ + TX_BUF - head_ - 1) % TX_BUF; }
  void pump() {
    int n = Serial.availableForWrite();
    while (n-- > 0 && tail_ != head_) {
      Serial.write(buf_[tail_]);
      tail_ = (tail_ + 1) % TX_BUF;
    }
  }
  void drain() {
    while (tail_ != head_) pump();
    Serial.flush();
  }

 private:
  uint8_t buf_[TX_BUF];
  uint16_t head_ = 0, tail_ = 0;
};
TxQueue tx;

// Print::print(float) prints "inf" for -inf too; ROS Range uses -inf for "too close".
void printNum(float v, uint8_t digits) {
  if (isnan(v)) tx.print(F("nan"));
  else if (isinf(v)) tx.print(v > 0 ? F("inf") : F("-inf"));
  else tx.print(v, digits);
}

// ===================== Global state =====================
uint8_t mode = MODE_GUI;

struct MotorInfo {
  uint8_t op;
  uint32_t lastCommandId;
  const __FlashStringHelper* status;
  float statusDeg;  // appended to status as "<value> deg" when not NaN
  bool held;        // STOP received in SENSOR mode: sensor control paused until mode changes
};
MotorInfo motors[MOTOR_COUNT];

void setStatus(uint8_t m, const __FlashStringHelper* s, float deg = NAN) {
  motors[m].status = s;
  motors[m].statusDeg = deg;
}

// Recent command IDs, so a resent command is not executed twice.
const uint8_t SEEN_IDS = 8;
uint32_t seenIds[SEEN_IDS];
uint8_t seenCount = 0, seenNext = 0;

// ===================== Sensors =====================
struct Smoother {
  int buf[FILTER_N];
  uint8_t idx = 0;
  long sum = 0;
  float ema = 0;
  bool primed = false;

  float update(int x) {
    if (!primed) {
      for (uint8_t i = 0; i < FILTER_N; i++) buf[i] = x;
      sum = (long)x * FILTER_N;
      ema = x;
      primed = true;
    }
    sum -= buf[idx];
    buf[idx] = x;
    sum += x;
    idx = (idx + 1) % FILTER_N;
    ema = FILTER_ALPHA * ((float)sum / FILTER_N) + (1.0 - FILTER_ALPHA) * ema;
    return ema;
  }
};

Smoother flexFilter, lightFilter, irFilter;

int flexAdc = 0, lightAdc = 0, irAdc = 0;
float flexRad = NAN, flexRadF = NAN;
float lightV = 0, lightLux = 0, lightLuxF = 0;
float irV = 0, irM = NAN, irMF = NAN;

SFEVL53L1X tof;
bool tofAvailable = false;
float tofM = NAN;
unsigned long tofLastMs = 0;

volatile unsigned long sonarRiseUs = 0;
volatile unsigned long sonarPulseUs = 0;
volatile bool sonarNew = false;
unsigned long sonarLastMs = 0;
float sonarUs = NAN, sonarM = NAN;

// ===================== Motors =====================
Servo servo;
float servoCmdDeg = 90;  // last commanded angle
float servoEstDeg = 90;  // estimated actual angle (servo has no feedback)

long stepPos = 0, stepTarget = 0, stepMoveStart = 0;
unsigned long lastStepUs = 0;
bool stepperEnabled = false;
bool stepperGuiMove = false;

volatile long encCount = 0;
volatile uint8_t encLast = 0;
// Quadrature state table: index = (oldState << 2) | newState
const int8_t QEM[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

enum : uint8_t { DC_IDLE, DC_POSITION, DC_VELOCITY };
uint8_t dcMode = DC_IDLE;
unsigned long dcLastCtrl = 0, dcModeStart = 0;
long dcLastCount = 0, dcStartCount = 0, dcTargetCount = 0;
float dcRpm = 0, dcTargetRpm = 0, dcVelIntegral = 0, dcPosIntegral = 0;
int dcSettledTicks = 0;
int8_t dcLastErrSign = 0;
float sensorDcRpm = 0;  // last velocity requested by sensor control

// ===================== Interrupts =====================
void encoderISR() {
  uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  encCount += ENC_DIR * QEM[(encLast << 2) | s];
  encLast = s;
}

long readEncoder() {
  noInterrupts();
  long c = encCount;
  interrupts();
  return c;
}

// MaxSonar pulse width without pulseIn() (which would block up to ~50 ms).
ISR(PCINT0_vect) {
  unsigned long t = micros();
  if (PINB & _BV(PB2)) {
    sonarRiseUs = t;
  } else {
    sonarPulseUs = t - sonarRiseUs;
    sonarNew = true;
  }
}

// ===================== Sensor reading =====================
// Read once to let the ADC mux settle after switching channel, then use the second read.
int readAdc(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

float adcToV(float adc) { return adc * ADC_REF_V / 1023.0; }

float flexAdcToRad(float adc) {
  if (adc <= 0.5) return NAN;  // open circuit
  float r = FLEX_FIXED_OHM * (1023.0 / adc - 1.0);
  float deg = (r - FLEX_R_FLAT_OHM) / (FLEX_R_90_OHM - FLEX_R_FLAT_OHM) * 90.0;
  return deg * DEG_TO_RAD;
}

// REP 117: +inf = nothing within range, -inf = closer than the minimum range.
float irVoltsToM(float v) {
  if (v <= 0.05) return INFINITY;
  float m = IR_K * pow(v, IR_EXP) / 100.0;
  if (m > IR_MAX_M) return INFINITY;
  if (m < IR_MIN_M) return -INFINITY;
  return m;
}

void sampleAnalog() {
  flexAdc = readAdc(PIN_FLEX);
  lightAdc = readAdc(PIN_LIGHT);
  irAdc = readAdc(PIN_IR);

  float flexF = flexFilter.update(flexAdc);
  flexRad = flexAdcToRad(flexAdc);
  flexRadF = flexAdcToRad(flexF);

  float lightF = lightFilter.update(lightAdc);
  lightV = adcToV(lightAdc);
  lightLux = lightV * LIGHT_LUX_PER_V;
  lightLuxF = adcToV(lightF) * LIGHT_LUX_PER_V;

  float irF = irFilter.update(irAdc);
  irV = adcToV(irAdc);
  irM = irVoltsToM(irV);
  irMF = irVoltsToM(adcToV(irF));
}

void pollTof(unsigned long now) {
  if (!tofAvailable) return;
  if (tof.checkForDataReady()) {
    int mm = tof.getDistance();
    uint8_t status = tof.getRangeStatus();
    tof.clearInterrupt();
    tofM = (status == 0) ? mm / 1000.0 : NAN;
    tofLastMs = now;
  } else if (now - tofLastMs > TOF_STALE_MS) {
    tofM = NAN;
  }
}

void updateSonar(unsigned long now) {
  if (sonarNew) {
    noInterrupts();
    unsigned long us = sonarPulseUs;
    sonarNew = false;
    interrupts();
    sonarUs = us;
    sonarM = us / SONAR_US_PER_INCH * 0.0254;
    sonarLastMs = now;
  } else if (now - sonarLastMs > SONAR_TIMEOUT_MS) {
    sonarUs = NAN;
    sonarM = NAN;
  }
}

// ===================== Servo =====================
void servoSet(float deg) {
  servoCmdDeg = constrain(round(deg), 0, 180);
  servo.write((int)servoCmdDeg);
}

void servoUpdate(unsigned long now) {
  static unsigned long last = 0;
  float dt = (now - last) / 1000.0;
  last = now;
  float maxStep = SERVO_SLEW_DEG_S * dt;
  float err = servoCmdDeg - servoEstDeg;
  servoEstDeg += constrain(err, -maxStep, maxStep);
}

bool servoMoving() { return fabs(servoCmdDeg - servoEstDeg) > 0.5; }

// ===================== Stepper =====================
void stepperUpdate() {
  if (stepPos == stepTarget) {
    if (stepperEnabled) {
      digitalWrite(PIN_STEP_EN, HIGH);  // release (as in motor_test) to keep the driver cool
      stepperEnabled = false;
      if (stepperGuiMove) {
        setStatus(MOTOR_STEPPER, F("moved"), (stepPos - stepMoveStart) * STEPPER_DEG_PER_STEP);
        stepperGuiMove = false;
      }
    }
    return;
  }
  unsigned long nowUs = micros();
  if (!stepperEnabled) {
    digitalWrite(PIN_STEP_EN, LOW);
    stepperEnabled = true;
    lastStepUs = nowUs;  // give the driver one interval to wake before the first step
    return;
  }
  if (nowUs - lastStepUs < STEP_INTERVAL_US) return;
  lastStepUs = nowUs;
  bool fwd = stepTarget > stepPos;
  digitalWrite(PIN_STEP_DIR, fwd ? LOW : HIGH);  // LOW = "forward", as in motor_test
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_STEP, LOW);
  stepPos += fwd ? 1 : -1;
}

void stepperStop() {
  stepTarget = stepPos;
  stepperGuiMove = false;
}

// ===================== DC motor =====================
void dcDrive(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(PIN_DC_IN1, pwm > 0 ? HIGH : LOW);
  digitalWrite(PIN_DC_IN2, pwm < 0 ? HIGH : LOW);
  analogWrite(PIN_DC_ENA, abs(pwm));
}

void dcStop() {  // coast
  dcMode = DC_IDLE;
  analogWrite(PIN_DC_ENA, 0);
  digitalWrite(PIN_DC_IN1, LOW);
  digitalWrite(PIN_DC_IN2, LOW);
}

void dcMoveDegrees(float degrees) {
  dcStartCount = readEncoder();
  dcTargetCount = dcStartCount + (long)(degrees * countsPerRev / 360.0);
  dcSettledTicks = 0;
  dcPosIntegral = 0;
  dcLastErrSign = 0;
  dcModeStart = millis();
  dcMode = DC_POSITION;
}

void dcSetVelocity(float rpm) {
  if (rpm == 0) {
    dcStop();
    return;
  }
  // Reset the integrator on a direction change; keep it when only the magnitude changes.
  if (dcMode != DC_VELOCITY || (rpm > 0) != (dcTargetRpm > 0)) dcVelIntegral = 0;
  dcTargetRpm = rpm;
  dcMode = DC_VELOCITY;
}

// Speed is measured every tick in every mode so state reports are always live.
void dcUpdate(unsigned long now) {
  if (now - dcLastCtrl < CTRL_DT_MS) return;
  float dt = (now - dcLastCtrl) / 1000.0;
  dcLastCtrl = now;

  long c = readEncoder();
  float rpmRaw = ((c - dcLastCount) / countsPerRev) / dt * 60.0;
  dcLastCount = c;
  dcRpm = 0.7 * dcRpm + 0.3 * rpmRaw;

  if (dcMode == DC_POSITION) {
    float errDeg = (dcTargetCount - c) * 360.0 / countsPerRev;
    float velDeg = dcRpm * 6.0;  // RPM -> deg/s

    if (fabs(errDeg) <= POS_TOL_DEG) dcSettledTicks++;
    else dcSettledTicks = 0;

    if (dcSettledTicks >= 3) {
      dcStop();
      if (mode == MODE_GUI) setStatus(MOTOR_DC, F("moved"), (c - dcStartCount) * 360.0 / countsPerRev);
      return;
    }
    if (now - dcModeStart > POS_TIMEOUT_MS) {
      dcStop();
      setStatus(MOTOR_DC, F("timeout, error"), errDeg);
      return;
    }

    // Clear the integral when crossing the target so it doesn't carry the push past it.
    int8_t sgn = (errDeg > 0) - (errDeg < 0);
    if (dcLastErrSign != 0 && sgn != 0 && sgn != dcLastErrSign) dcPosIntegral = 0;
    if (sgn != 0) dcLastErrSign = sgn;

    float pd = POS_KP * errDeg - POS_KD * velDeg;
    if (fabs(pd) < POS_MAX_PWM) dcPosIntegral += POS_KI * errDeg * dt;  // no windup when saturated
    dcPosIntegral = constrain(dcPosIntegral, -POS_I_MAX, POS_I_MAX);
    dcDrive((int)constrain(pd + dcPosIntegral, -POS_MAX_PWM, POS_MAX_PWM));
  } else if (dcMode == DC_VELOCITY) {
    float err = dcTargetRpm - dcRpm;
    dcVelIntegral += VEL_KI * err * dt;
    dcVelIntegral = constrain(dcVelIntegral, -255, 255);  // anti-windup
    float out = VEL_KP * err + dcVelIntegral;
    if (out > 0 && out < lower_limit_dc) out = lower_limit_dc;
    else if (out < 0 && out > -lower_limit_dc) out = -lower_limit_dc;
    dcDrive((int)out);
  }
}

// ===================== Mode / stop =====================
void stopMotor(uint8_t m) {
  if (m == MOTOR_SERVO) servoSet(servoEstDeg);  // hold where it is now
  else if (m == MOTOR_DC) dcStop();
  else stepperStop();
  motors[m].op = OP_STOP;
}

void sendDiag(uint8_t level, const __FlashStringHelper* name, const __FlashStringHelper* msg) {
  if (tx.room() < TX_LINE_MAX) return;
  tx.print(F("D,"));
  tx.print(level);
  tx.print(',');
  tx.print(name);
  tx.print(',');
  tx.println(msg);
}

void setMode(uint8_t newMode) {
  if (newMode == mode) return;
  mode = newMode;
  for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
    stopMotor(m);
    motors[m].held = false;
    setStatus(m, F("mode changed"));
  }
  sensorDcRpm = 0;
  sendDiag(0, F("control_mode"), mode == MODE_SENSOR ? F("SENSOR") : F("GUI"));
}

void updateButton(unsigned long now) {
  static bool lastReading = HIGH, stable = HIGH;
  static unsigned long lastChange = 0;
  bool reading = digitalRead(PIN_BUTTON);
  if (reading != lastReading) {
    lastReading = reading;
    lastChange = now;
  }
  if (now - lastChange >= DEBOUNCE_MS && reading != stable) {
    stable = reading;
    if (stable == LOW) setMode(mode == MODE_GUI ? MODE_SENSOR : MODE_GUI);  // on press
  }
}

// ===================== Sensor control =====================
void applySensorControl() {
  if (!motors[MOTOR_SERVO].held && isfinite(flexRadF)) {
    float goal = constrain(flexRadF * RAD_TO_DEG * SERVO_PER_FLEX, 0, 180);
    if (fabs(goal - servoCmdDeg) >= SERVO_SENSOR_DEADBAND_DEG) servoSet(goal);
    motors[MOTOR_SERVO].op = OP_POSITION_ABSOLUTE;
    setStatus(MOTOR_SERVO, F("sensor control"));
  }

  if (!motors[MOTOR_DC].held) {
    // NaN compares false both ways, so a bad reading stops the motor.
    float rpm = irMF < IR_REVERSE_BELOW_M ? -DC_SENSOR_RPM
              : irMF > IR_FORWARD_ABOVE_M ? DC_SENSOR_RPM
              : 0;
    if (rpm != sensorDcRpm) {
      sensorDcRpm = rpm;
      dcSetVelocity(rpm);
    }
    motors[MOTOR_DC].op = OP_VELOCITY;
    setStatus(MOTOR_DC, F("sensor control"));
  }

  if (!motors[MOTOR_STEPPER].held && isfinite(tofM)) {
    float frac = constrain((tofM - TOF_MIN_M) / (TOF_MAX_M - TOF_MIN_M), 0, 1);
    long goal = lround(frac * 360.0 / STEPPER_DEG_PER_STEP);
    if (labs(goal - stepTarget) * STEPPER_DEG_PER_STEP > STEPPER_SENSOR_DEADBAND_DEG) stepTarget = goal;
    motors[MOTOR_STEPPER].op = OP_MOVE_RELATIVE;
    setStatus(MOTOR_STEPPER, F("sensor control"));
  }
}

// ===================== Commands from the laptop =====================
bool seenBefore(uint32_t id) {
  for (uint8_t i = 0; i < seenCount; i++)
    if (seenIds[i] == id) return true;
  seenIds[seenNext] = id;
  seenNext = (seenNext + 1) % SEEN_IDS;
  if (seenCount < SEEN_IDS) seenCount++;
  return false;
}

void reject(uint32_t id, const __FlashStringHelper* reason) {
  if (tx.room() < TX_LINE_MAX) return;
  tx.print(F("D,2,motor_command,Rejected command #"));
  tx.print(id);
  tx.print(F(": "));
  tx.println(reason);
}

void handleMotorCommand(uint32_t id, uint8_t m, uint8_t op, float target) {
  if (seenBefore(id)) return;
  if (m >= MOTOR_COUNT) return reject(id, F("unknown motor"));
  if (!isfinite(target)) return reject(id, F("target is not finite"));

  if (op == OP_STOP) {  // STOP works in either mode
    stopMotor(m);
    motors[m].lastCommandId = id;
    if (mode == MODE_SENSOR) {
      motors[m].held = true;
      if (m == MOTOR_DC) sensorDcRpm = 0;
      setStatus(m, F("stopped, sensor control paused"));
    } else {
      setStatus(m, F("stopped by GUI"));
    }
    return;
  }
  if (mode != MODE_GUI) return reject(id, F("motion commands need GUI mode"));

  float deg = target * RAD_TO_DEG;
  if (m == MOTOR_SERVO) {
    if (op != OP_POSITION_ABSOLUTE) return reject(id, F("servo accepts absolute position only"));
    if (deg < -0.01 || deg > 180.01) return reject(id, F("servo target outside 0-180 deg"));
    servoSet(deg);
    setStatus(MOTOR_SERVO, F("moving to"), servoCmdDeg);
  } else if (m == MOTOR_DC) {
    if (op == OP_MOVE_RELATIVE) {
      if (fabs(deg) > DC_MAX_REL_DEG) return reject(id, F("relative move above limit"));
      dcMoveDegrees(deg);
      setStatus(MOTOR_DC, F("moving"), deg);
    } else if (op == OP_VELOCITY) {
      float rpm = target * 60.0 / TWO_PI;
      if (fabs(rpm) > DC_MAX_RPM) return reject(id, F("velocity above limit"));
      dcSetVelocity(rpm);
      setStatus(MOTOR_DC, rpm == 0 ? F("stopped") : F("velocity control"));
    } else {
      return reject(id, F("DC accepts relative or velocity only"));
    }
  } else {
    if (op != OP_MOVE_RELATIVE) return reject(id, F("stepper accepts relative move only"));
    if (fabs(deg) > STEPPER_MAX_REL_DEG) return reject(id, F("relative move above limit"));
    long steps = lround(deg / STEPPER_DEG_PER_STEP);  // round to nearest 1/8 microstep
    stepMoveStart = stepPos;
    stepTarget = stepPos + steps;
    stepperGuiMove = true;
    setStatus(MOTOR_STEPPER, F("moving"), steps * STEPPER_DEG_PER_STEP);
  }
  motors[m].op = op;
  motors[m].lastCommandId = id;
}

// Lines: "C,<id>,<motor>,<op>,<target>"  "M,<mode>"  "Z"  "K,<counts>"  "P"
void handleLine(char* line) {
  char* fields[5];
  uint8_t n = 0;
  for (char* tok = strtok(line, ","); tok && n < 5; tok = strtok(nullptr, ",")) fields[n++] = tok;
  if (n == 0) return;

  switch (fields[0][0]) {
    case 'C':
      if (n == 5) {
        handleMotorCommand(strtoul(fields[1], nullptr, 10), atoi(fields[2]), atoi(fields[3]),
                           atof(fields[4]));
      }
      break;
    case 'M':
      if (n == 2) setMode(atoi(fields[1]) == MODE_SENSOR ? MODE_SENSOR : MODE_GUI);
      break;
    case 'Z':
      noInterrupts();
      encCount = 0;
      interrupts();
      dcLastCount = 0;
      dcStop();
      break;
    case 'K':
      if (n == 2 && atof(fields[1]) > 0) countsPerRev = atof(fields[1]);
      break;
    case 'P':
      if (tx.room() >= TX_LINE_MAX) {
        tx.print(F("I,enc_count="));
        tx.print(readEncoder());
        tx.print(F(" counts_per_rev="));
        tx.println(countsPerRev, 1);
      }
      break;
  }
}

void readSerial() {
  static char buf[48];
  static uint8_t len = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0 && !overflow) {
        buf[len] = '\0';
        handleLine(buf);
      }
      len = 0;
      overflow = false;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      overflow = true;  // drop over-long lines entirely
    }
  }
}

// ===================== Telemetry =====================
// S,<ms>,<flex_adc>,<flex_rad>,<flex_rad_f>,<light_v>,<lux>,<lux_f>,<ir_v>,<ir_m>,<ir_m_f>,
//   <tof_m>,<sonar_pulse_us>,<sonar_m>
// Telemetry waits until the queue has room for it plus a command reply; returns false to retry.
bool sendSensors(unsigned long now) {
  if (tx.room() < 2 * TX_LINE_MAX) return false;
  tx.print(F("S,"));
  tx.print(now);
  tx.print(',');
  tx.print(flexAdc);
  tx.print(',');
  printNum(flexRad, 4);
  tx.print(',');
  printNum(flexRadF, 4);
  tx.print(',');
  printNum(lightV, 3);
  tx.print(',');
  printNum(lightLux, 1);
  tx.print(',');
  printNum(lightLuxF, 1);
  tx.print(',');
  printNum(irV, 3);
  tx.print(',');
  printNum(irM, 4);
  tx.print(',');
  printNum(irMF, 4);
  tx.print(',');
  printNum(tofM, 4);
  tx.print(',');
  printNum(sonarUs, 0);
  tx.print(',');
  printNum(sonarM, 4);
  tx.println();
  return true;
}

// MS,<ms>,<motor>,<mode>,<op>,<enabled>,<moving>,<pos_rad>,<vel_rad_s>,<target>,
//    <pos_measured>,<vel_measured>,<last_command_id>,<status>
bool sendMotor(unsigned long now, uint8_t m) {
  if (tx.room() < 2 * TX_LINE_MAX) return false;
  bool moving;
  float pos, vel, target;
  bool measured = false;

  if (m == MOTOR_SERVO) {
    moving = servoMoving();
    pos = servoEstDeg * DEG_TO_RAD;
    vel = moving ? copysign(SERVO_SLEW_DEG_S, servoCmdDeg - servoEstDeg) * DEG_TO_RAD : 0;
    target = servoCmdDeg * DEG_TO_RAD;
  } else if (m == MOTOR_DC) {
    measured = true;
    long c = readEncoder();
    pos = c * TWO_PI / countsPerRev;
    vel = dcRpm * TWO_PI / 60.0;
    moving = dcMode != DC_IDLE || fabs(dcRpm) > 1.0;
    if (dcMode == DC_POSITION) target = dcTargetCount * TWO_PI / countsPerRev;
    else if (dcMode == DC_VELOCITY) target = dcTargetRpm * TWO_PI / 60.0;
    else target = pos;
  } else {
    moving = stepPos != stepTarget;
    pos = stepPos * STEPPER_DEG_PER_STEP * DEG_TO_RAD;
    vel = moving ? (stepTarget > stepPos ? 1 : -1) * (1e6 / STEP_INTERVAL_US) *
                       STEPPER_DEG_PER_STEP * DEG_TO_RAD
                 : 0;
    target = stepTarget * STEPPER_DEG_PER_STEP * DEG_TO_RAD;
  }

  tx.print(F("MS,"));
  tx.print(now);
  tx.print(',');
  tx.print(m);
  tx.print(',');
  tx.print(mode);
  tx.print(',');
  tx.print(motors[m].op);
  tx.print(',');
  tx.print(motors[m].held ? 0 : 1);
  tx.print(',');
  tx.print(moving ? 1 : 0);
  tx.print(',');
  printNum(pos, 4);
  tx.print(',');
  printNum(vel, 4);
  tx.print(',');
  printNum(target, 4);
  tx.print(',');
  tx.print(measured ? 1 : 0);
  tx.print(',');
  tx.print(measured ? 1 : 0);
  tx.print(',');
  tx.print(motors[m].lastCommandId);
  tx.print(',');
  tx.print(motors[m].status);
  if (!isnan(motors[m].statusDeg)) {
    tx.print(' ');
    tx.print(motors[m].statusDeg, 3);
    tx.print(F(" deg"));
  }
  tx.println();
  return true;
}

void sendHealth() {
  if (!tofAvailable) sendDiag(2, F("tof"), F("VL53L1X not detected"));
  else if (isnan(tofM)) sendDiag(1, F("tof"), F("no valid measurement"));
  else sendDiag(0, F("tof"), F("ok"));

  if (isnan(sonarUs)) sendDiag(1, F("ultrasonic"), F("no pulse (check wiring on D10)"));
  else sendDiag(0, F("ultrasonic"), F("ok"));
}

// ===================== setup / loop =====================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_FLEX, INPUT);
  pinMode(PIN_LIGHT, INPUT);
  pinMode(PIN_IR, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Stepper: 1/8 microstep (MS1 = MS2 = HIGH), driver released until a move.
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_STEP_DIR, OUTPUT);
  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_STEP_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_STEP_DIR, LOW);
  digitalWrite(PIN_MS1, HIGH);
  digitalWrite(PIN_MS2, HIGH);
  digitalWrite(PIN_STEP_EN, HIGH);

  // DC motor + encoder
  pinMode(PIN_DC_IN1, OUTPUT);
  pinMode(PIN_DC_IN2, OUTPUT);
  pinMode(PIN_DC_ENA, OUTPUT);
  dcStop();
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encLast = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  servo.attach(PIN_SERVO);
  servoSet(90);
  servoEstDeg = 90;

  // MaxSonar PW on D10 = PB2 = PCINT2 (pin-change interrupt group 0).
  pinMode(PIN_SONAR_PW, INPUT);
  PCMSK0 |= _BV(PCINT2);
  PCICR |= _BV(PCIE0);

  for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
    motors[m].op = OP_STOP;
    motors[m].lastCommandId = 0;
    motors[m].held = false;
    setStatus(m, F("idle"));
  }

  Wire.begin();
  Wire.setClock(100000);
  tofAvailable = (tof.begin() == 0);
  if (tofAvailable) {
    tof.setDistanceModeShort();
    tof.setTimingBudgetInMs(50);
    tof.setIntermeasurementPeriod(100);
    tof.startRanging();
  }

  tx.println(F("I,ego2sim firmware ready"));
  sendHealth();
  tx.drain();

  unsigned long now = millis();
  dcLastCtrl = now;
  tofLastMs = now;
  sonarLastMs = now;
}

void loop() {
  static unsigned long lastSample = 0, lastTof = 0, lastSensorCtrl = 0;
  static unsigned long lastSensorTx = 0, lastMotorTx = 0, lastHealth = 0;
  static uint8_t nextMotorTx = MOTOR_COUNT;

  unsigned long now = millis();

  readSerial();
  updateButton(now);

  stepperUpdate();
  servoUpdate(now);
  dcUpdate(now);

  if (now - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = now;
    sampleAnalog();
  }
  if (now - lastTof >= TOF_POLL_MS) {
    lastTof = now;
    pollTof(now);
  }
  updateSonar(now);

  if (mode == MODE_SENSOR && now - lastSensorCtrl >= SENSOR_CTRL_PERIOD_MS) {
    lastSensorCtrl = now;
    applySensorControl();
  }

  // Telemetry: one line per loop at most, so no single loop pass does much printing.
  if (now - lastSensorTx >= SENSOR_TX_PERIOD_MS) {
    if (sendSensors(now)) lastSensorTx = now;
  } else if (nextMotorTx < MOTOR_COUNT) {
    if (sendMotor(now, nextMotorTx)) nextMotorTx++;
  } else if (now - lastMotorTx >= MOTOR_TX_PERIOD_MS) {
    lastMotorTx = now;
    nextMotorTx = 0;
  } else if (now - lastHealth >= HEALTH_TX_PERIOD_MS && tx.room() >= 2 * TX_LINE_MAX) {
    lastHealth = now;
    sendHealth();
  }

  stepperUpdate();
  tx.pump();
}
