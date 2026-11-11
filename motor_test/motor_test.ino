//Declare pin functions on RedBoard
#include <Servo.h>

// --- Easy Driver (stepper) ---
#define stp 7
#define dir 8
#define MS1 4
#define MS2 5
#define EN  6

// --- L298N (DC motor) ---
#define IN1 13   // L298N IN1 (L1)
#define IN2 12   // L298N IN2 (L2)
#define ENA 11   // L298N ENA (enable / PWM speed) - remove the ENA jumper on the board

// --- Encoder (quadrature: channel A and channel B) ---
#define ENC1 2   // Channel A (interrupt pin)
#define ENC2 3   // Channel B (interrupt pin)

// --- Servo ---
#define SERVO_PIN 9   // Servo signal wire

// ===================== ENCODER SETTINGS =====================
// Counts per OUTPUT-SHAFT revolution, counted on both edges of both channels (4x decoding).
//   countsPerRev = 4 * (encoder pulses per motor-shaft rev) * (gearbox ratio)
// If you don't know it, use option 'r' in the menu to measure it by hand,
// or set it directly with k<number>, e.g. k1920
float countsPerRev = 360.0;   // PLACEHOLDER - calibrate this!

// If "forward" (option 5) gives NEGATIVE counts, change this to -1
// (or swap the two encoder wires).
#define ENC_DIR 1

// ===================== CONTROL TUNING =====================
#define CTRL_DT_MS     40      // Control loop period (ms)

// Position control (PD, working in degrees)
#define POS_KP         2.0     // PWM per degree of error
#define POS_KD         0.5    // PWM per (degree/second) of speed (damping)
#define POS_KI         2.0    // PWM per (degree*second) of accumulated error. Breaks the motor
                               // free on small moves. Raise if small moves are slow to start,
                               // lower if it overshoots/oscillates at the end.
#define POS_I_MAX      150     // Cap on the integral's PWM contribution (anti-windup)
#define POS_MAX_PWM    200     // Max PWM during position moves
#define POS_TOL_DEG    1.5     // Done when within this many degrees
#define POS_TIMEOUT_MS 15000   // Safety timeout

// Velocity control (PI, working in RPM of the output shaft)
#define VEL_KP         0.5     // PWM per RPM of error
#define VEL_KI         3.0     // PWM per (RPM * second) of accumulated error

// Lowest PWM velocity mode (v) will send the motor. (Position mode uses the Ki term instead,
// since a fixed floor makes it overshoot and oscillate at the end of a move.)
#define lower_limit_dc 0

//Declare variables for functions
char user_input;
int x;
int y;
int state;
int dcSpeed = 200;   // DC motor PWM speed, 0-255 (open loop)

Servo servoMotor;
int servoAngle = 90; // Current servo position in degrees

// --- Encoder / control state ---
volatile long encCount = 0;
volatile uint8_t encLast = 0;
// Quadrature state table: index = (oldState << 2) | newState
const int8_t QEM[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

enum CtrlMode { MODE_IDLE, MODE_OPEN, MODE_POSITION, MODE_VELOCITY };
CtrlMode ctrlMode = MODE_IDLE;

unsigned long lastCtrl = 0;
unsigned long modeStart = 0;
unsigned long lastPrint = 0;
long lastCount = 0;
float rpmFilt = 0;
long targetCounts = 0;
float targetDeg = 0;
float targetRPM = 0;
float velIntegral = 0;
int settledTicks = 0;
float posIntegral = 0;
int lastErrSign = 0;

void setup() {
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(ENC1, INPUT_PULLUP);  // pullups are harmless for push-pull encoders
  pinMode(ENC2, INPUT_PULLUP);  // and needed for open-collector ones

  resetEDPins(); //Set step, direction, microstep and enable pins to default states
  DCStop();      //Make sure the DC motor starts off

  // Encoder interrupts on both channels, both edges
  encLast = (digitalRead(ENC1) << 1) | digitalRead(ENC2);
  attachInterrupt(digitalPinToInterrupt(ENC1), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2), encoderISR, CHANGE);

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(servoAngle); //Start the servo centered

  Serial.begin(9600); //Open Serial connection for debugging
  Serial.println(F("Begin motor control"));
  Serial.println();
  printMenu();
}

//Encoder interrupt: update count from the A/B state change
void encoderISR()
{
  uint8_t s = (digitalRead(ENC1) << 1) | digitalRead(ENC2);
  encCount += ENC_DIR * QEM[(encLast << 2) | s];
  encLast = s;
}

//Safely read the 32-bit count (interrupts could change it mid-read)
long readEncoder()
{
  noInterrupts();
  long c = encCount;
  interrupts();
  return c;
}

//Print function list for user selection
void printMenu()
{
  Serial.println(F("Enter number for control option:"));
  Serial.println(F("--- Stepper motor (Easy Driver) ---"));
  Serial.println(F("1. Turn at default microstep mode."));
  Serial.println(F("2. Reverse direction at default microstep mode."));
  Serial.println(F("3. Turn at 1/8th microstep mode."));
  Serial.println(F("4. Step forward and reverse directions."));
  Serial.println(F("--- DC motor (L298N) ---"));
  Serial.println(F("5. DC motor forward."));
  Serial.println(F("6. DC motor reverse."));
  Serial.println(F("7. DC motor stop (coast)."));
  Serial.println(F("8. DC motor brake."));
  Serial.println(F("9. DC motor ramp speed up and down."));
  Serial.println(F("+ / -  Increase / decrease DC motor speed (open loop)."));
  Serial.println(F("--- DC motor with encoder ---"));
  Serial.println(F("g<degrees>  Move by that many degrees, e.g. g90 or g-180"));
  Serial.println(F("v<rpm>      Hold a speed in RPM, e.g. v60 or v-60 (v0 or 7 to stop)"));
  Serial.println(F("p           Print encoder count and angle"));
  Serial.println(F("z           Zero the encoder count"));
  Serial.println(F("r           Calibrate counts per revolution (turn shaft by hand)"));
  Serial.println(F("k<counts>   Set counts per revolution manually, e.g. k1920"));
  Serial.println(F("--- Servo (pin 9) ---"));
  Serial.println(F("a. Servo to 0 degrees."));
  Serial.println(F("b. Servo to 90 degrees (center)."));
  Serial.println(F("c. Servo to 180 degrees."));
  Serial.println(F("d. Servo sweep 0-180-0."));
  Serial.println(F("s<angle>  Servo to a specific angle, e.g. s45"));
  Serial.println(F("< / >  Nudge servo -10 / +10 degrees."));
  Serial.println();
}

 //Main loop
void loop() {
  while(Serial.available()){
      user_input = Serial.read(); //Read user input and trigger appropriate function

      // Ignore line-ending characters sent by the Serial Monitor
      if (user_input == '\n' || user_input == '\r' || user_input == ' ')
      {
        continue;
      }

      if (user_input =='1')
      {
         digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
         StepForwardDefault();
         resetEDPins();
      }
      else if(user_input =='2')
      {
        digitalWrite(EN, LOW);
        ReverseStepDefault();
        resetEDPins();
      }
      else if(user_input =='3')
      {
        digitalWrite(EN, LOW);
        SmallStepMode();
        resetEDPins();
      }
      else if(user_input =='4')
      {
        digitalWrite(EN, LOW);
        ForwardBackwardStep();
        resetEDPins();
      }
      else if(user_input =='5')
      {
        DCForward();
      }
      else if(user_input =='6')
      {
        DCReverse();
      }
      else if(user_input =='7')
      {
        DCStop();
        Serial.println(F("DC motor stopped (coasting)."));
        Serial.println(F("Enter new option"));
        Serial.println();
      }
      else if(user_input =='8')
      {
        DCBrake();
      }
      else if(user_input =='9')
      {
        DCRamp();
      }
      else if(user_input =='+')
      {
        DCChangeSpeed(25);
      }
      else if(user_input =='-')
      {
        DCChangeSpeed(-25);
      }
      else if(user_input =='g' || user_input =='G')
      {
        DCMoveDegrees(Serial.parseFloat()); //Read the number typed after 'g'
      }
      else if(user_input =='v' || user_input =='V')
      {
        DCSetVelocity(Serial.parseFloat()); //Read the number typed after 'v'
      }
      else if(user_input =='p' || user_input =='P')
      {
        EncoderPrint();
      }
      else if(user_input =='z' || user_input =='Z')
      {
        noInterrupts();
        encCount = 0;
        interrupts();
        Serial.println(F("Encoder zeroed."));
        Serial.println();
      }
      else if(user_input =='r' || user_input =='R')
      {
        EncoderCalibrate();
      }
      else if(user_input =='k' || user_input =='K')
      {
        float k = Serial.parseFloat();
        if (k > 0)
        {
          countsPerRev = k;
          Serial.print(F("Counts per revolution set to "));
          Serial.println(countsPerRev);
        }
        else
        {
          Serial.println(F("Enter a positive number, e.g. k1920"));
        }
        Serial.println();
      }
      else if(user_input =='a' || user_input =='A')
      {
        ServoMoveTo(0);
      }
      else if(user_input =='b' || user_input =='B')
      {
        ServoMoveTo(90);
      }
      else if(user_input =='c' || user_input =='C')
      {
        ServoMoveTo(180);
      }
      else if(user_input =='d' || user_input =='D')
      {
        ServoSweep();
      }
      else if(user_input =='s' || user_input =='S')
      {
        ServoMoveTo(Serial.parseInt()); //Read the number typed after 's'
      }
      else if(user_input =='<')
      {
        ServoMoveTo(servoAngle - 10);
      }
      else if(user_input =='>')
      {
        ServoMoveTo(servoAngle + 10);
      }
      else
      {
        Serial.println(F("Invalid option entered."));
        printMenu();
      }
  }

  updateControl(); //Run the closed-loop position / velocity controller
}

// ===================== Closed-loop control =====================

//Drive the motor with a signed PWM value (-255..255)
void DCDrive(int pwm)
{
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (pwm < 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(pwm));
}

//Move the DC motor by a number of degrees relative to where it is now
void DCMoveDegrees(float degrees)
{
  if (degrees == 0)
  {
    Serial.println(F("Enter a non-zero angle, e.g. g90 or g-90"));
    Serial.println();
    return;
  }
  targetDeg = degrees;
  targetCounts = readEncoder() + (long)(degrees * countsPerRev / 360.0);
  lastCount = readEncoder();
  rpmFilt = 0;
  settledTicks = 0;
  posIntegral = 0;
  lastErrSign = 0;
  modeStart = millis();
  lastCtrl = millis();
  ctrlMode = MODE_POSITION;
  Serial.print(F("Moving "));
  Serial.print(degrees);
  Serial.println(F(" degrees..."));
}

//Hold the DC motor at a target speed in RPM (negative = reverse)
void DCSetVelocity(float rpm)
{
  if (rpm == 0)
  {
    DCStop();
    Serial.println(F("DC motor stopped."));
    Serial.println(F("Enter new option"));
    Serial.println();
    return;
  }
  targetRPM = rpm;
  velIntegral = 0;
  lastCount = readEncoder();
  rpmFilt = 0;
  lastCtrl = millis();
  lastPrint = millis();
  ctrlMode = MODE_VELOCITY;
  Serial.print(F("Holding "));
  Serial.print(rpm);
  Serial.println(F(" RPM. Send 7 to stop."));
}

//Runs every loop; does the actual work once per CTRL_DT_MS
void updateControl()
{
  if (ctrlMode != MODE_POSITION && ctrlMode != MODE_VELOCITY) return;

  unsigned long now = millis();
  if (now - lastCtrl < CTRL_DT_MS) return;
  float dt = (now - lastCtrl) / 1000.0;
  lastCtrl = now;

  // Measure speed (RPM of output shaft), lightly filtered
  long c = readEncoder();
  float rpmRaw = ((c - lastCount) / countsPerRev) / dt * 60.0;
  lastCount = c;
  rpmFilt = 0.7 * rpmFilt + 0.3 * rpmRaw;

  if (ctrlMode == MODE_POSITION)
  {
    float errDeg = (targetCounts - c) * 360.0 / countsPerRev;
    float velDeg = rpmFilt * 6.0;  // RPM -> degrees per second

    if (fabs(errDeg) <= POS_TOL_DEG) settledTicks++; else settledTicks = 0;

    if (settledTicks >= 3)
    {
      DCStop();
      Serial.print(F("Done. Moved "));
      Serial.print((c - (targetCounts - (long)(targetDeg * countsPerRev / 360.0))) * 360.0 / countsPerRev);
      Serial.println(F(" degrees."));
      Serial.println(F("Enter new option"));
      Serial.println();
      return;
    }
    if (now - modeStart > POS_TIMEOUT_MS)
    {
      DCStop();
      Serial.println(F("Position move timed out (motor stalled or encoder not counting?)."));
      Serial.print(F("Remaining error (degrees): "));
      Serial.println(errDeg);
      Serial.println();
      return;
    }

    // Clear the integral when the motor crosses the target so it doesn't carry the push past it
    int sgn = (errDeg > 0) - (errDeg < 0);
    if (lastErrSign != 0 && sgn != 0 && sgn != lastErrSign) posIntegral = 0;
    if (sgn != 0) lastErrSign = sgn;

    float pd = POS_KP * errDeg - POS_KD * velDeg;
    if (fabs(pd) < POS_MAX_PWM) posIntegral += POS_KI * errDeg * dt;  // don't wind up while saturated
    posIntegral = constrain(posIntegral, -POS_I_MAX, POS_I_MAX);
    float out = constrain(pd + posIntegral, -POS_MAX_PWM, POS_MAX_PWM);
    DCDrive((int)out);
  }
  else // MODE_VELOCITY
  {
    float err = targetRPM - rpmFilt;
    velIntegral += VEL_KI * err * dt;
    velIntegral = constrain(velIntegral, -255, 255);  // anti-windup
    float out = VEL_KP * err + velIntegral;
    if (out > 0 && out < lower_limit_dc) out = lower_limit_dc;
    else if (out < 0 && out > -lower_limit_dc) out = -lower_limit_dc;
    DCDrive((int)out);

    if (now - lastPrint >= 250)
    {
      lastPrint = now;
      Serial.print(F("RPM: "));
      Serial.print(rpmFilt);
      Serial.print(F("  (target "));
      Serial.print(targetRPM);
      Serial.println(F(")"));
    }
  }
}

//Print raw count and the angle it represents
void EncoderPrint()
{
  long c = readEncoder();
  Serial.print(F("Count: "));
  Serial.print(c);
  Serial.print(F("  Angle: "));
  Serial.print(c * 360.0 / countsPerRev);
  Serial.println(F(" degrees"));
  Serial.println();
}

//Measure counts per output-shaft revolution by turning the shaft by hand
void EncoderCalibrate()
{
  DCStop();
  Serial.println(F("Calibration: mark the output shaft so you can see one full turn."));
  Serial.println(F("Turn it by hand exactly ONE full revolution in the forward direction,"));
  Serial.println(F("then send any character (e.g. x)."));
  noInterrupts();
  encCount = 0;
  interrupts();

  while (Serial.available()) Serial.read();  // clear leftover characters
  while (!Serial.available()) { }            // wait for the user
  while (Serial.available()) Serial.read();

  long c = readEncoder();
  Serial.print(F("Counted "));
  Serial.println(c);
  if (c == 0)
  {
    Serial.println(F("No counts seen - check encoder power, GND and wiring."));
  }
  else
  {
    if (c < 0) Serial.println(F("Count was negative: set ENC_DIR to -1 (or swap the A/B wires)."));
    countsPerRev = abs(c);
    Serial.print(F("Counts per revolution set to "));
    Serial.println(countsPerRev);
    Serial.println(F("(Repeat a couple of times; for best accuracy turn several revs and use k<total/revs>.)"));
  }
  Serial.println();
}

// ===================== Stepper functions =====================

//Default microstep mode function
void StepForwardDefault()
{
  Serial.println(F("Moving forward at default step mode."));
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  for(x= 0; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println(F("Enter new option"));
  Serial.println();
}

//Reverse default microstep mode function
void ReverseStepDefault()
{
  Serial.println(F("Moving in reverse at default step mode."));
  digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
  for(x= 0; x<1000; x++)  //Loop the stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println(F("Enter new option"));
  Serial.println();
}

// 1/8th microstep foward mode function
void SmallStepMode()
{
  Serial.println(F("Stepping at 1/8th microstep mode."));
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  digitalWrite(MS1, HIGH); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
  digitalWrite(MS2, HIGH);
  for(x= 0; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println(F("Enter new option"));
  Serial.println();
}

//Forward/reverse stepping function
void ForwardBackwardStep()
{
  Serial.println(F("Alternate between stepping forward and reverse."));
  for(x= 1; x<5; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    //Read direction pin state and change it
    state=digitalRead(dir);
    if(state == HIGH)
    {
      digitalWrite(dir, LOW);
    }
    else if(state ==LOW)
    {
      digitalWrite(dir,HIGH);
    }

    for(y=0; y<1000; y++)
    {
      digitalWrite(stp,HIGH); //Trigger one step
      delay(1);
      digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
      delay(1);
    }
  }
  Serial.println(F("Enter new option:"));
  Serial.println();
}

//Reset Easy Driver pins to default states
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN, HIGH);
}

// ===================== DC motor functions (L298N) =====================

//Run DC motor forward at current speed (keeps running until stopped)
void DCForward()
{
  ctrlMode = MODE_OPEN;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, dcSpeed);
  Serial.print(F("DC motor forward at speed "));
  Serial.println(dcSpeed);
  Serial.println(F("Enter new option (7 to stop)"));
  Serial.println();
}

//Run DC motor in reverse at current speed (keeps running until stopped)
void DCReverse()
{
  ctrlMode = MODE_OPEN;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, dcSpeed);
  Serial.print(F("DC motor reverse at speed "));
  Serial.println(dcSpeed);
  Serial.println(F("Enter new option (7 to stop)"));
  Serial.println();
}

//Stop DC motor and let it coast (bridge disabled)
void DCStop()
{
  ctrlMode = MODE_IDLE;
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

//Actively brake the DC motor (both motor terminals shorted together)
void DCBrake()
{
  ctrlMode = MODE_IDLE;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);
  delay(300);          //Hold the brake briefly
  DCStop();
  Serial.println(F("DC motor braked."));
  Serial.println(F("Enter new option"));
  Serial.println();
}

//Ramp the DC motor up to full speed and back down, forward then reverse
void DCRamp()
{
  ctrlMode = MODE_IDLE;
  Serial.println(F("Ramping DC motor forward..."));
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  for(x = 0; x <= 255; x += 5)
  {
    analogWrite(ENA, x);
    delay(30);
  }
  for(x = 255; x >= 0; x -= 5)
  {
    analogWrite(ENA, x);
    delay(30);
  }

  Serial.println(F("Ramping DC motor reverse..."));
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  for(x = 0; x <= 255; x += 5)
  {
    analogWrite(ENA, x);
    delay(30);
  }
  for(x = 255; x >= 0; x -= 5)
  {
    analogWrite(ENA, x);
    delay(30);
  }

  DCStop();
  Serial.println(F("Ramp complete."));
  Serial.println(F("Enter new option"));
  Serial.println();
}

//Change DC motor speed; applies immediately if the motor is running (open loop only)
void DCChangeSpeed(int delta)
{
  dcSpeed = constrain(dcSpeed + delta, 0, 255);
  if (ctrlMode == MODE_OPEN)
  {
    analogWrite(ENA, dcSpeed);
  }
  Serial.print(F("DC motor speed set to "));
  Serial.println(dcSpeed);
  Serial.println();
}

// ===================== Servo functions =====================

//Move the servo to an angle (clamped to 0-180)
void ServoMoveTo(int angle)
{
  servoAngle = constrain(angle, 0, 180);
  servoMotor.write(servoAngle);
  Serial.print(F("Servo moved to "));
  Serial.print(servoAngle);
  Serial.println(F(" degrees."));
  Serial.println(F("Enter new option"));
  Serial.println();
}

//Sweep the servo from 0 to 180 and back, then return to its previous angle
void ServoSweep()
{
  Serial.println(F("Sweeping servo 0 -> 180 -> 0..."));
  for(x = 0; x <= 180; x++)
  {
    servoMotor.write(x);
    delay(10);
  }
  for(x = 180; x >= 0; x--)
  {
    servoMotor.write(x);
    delay(10);
  }
  servoMotor.write(servoAngle); //Return to where it was before the sweep
  Serial.println(F("Sweep complete."));
  Serial.println(F("Enter new option"));
  Serial.println();
}
