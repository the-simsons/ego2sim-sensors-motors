//Declare pin functions on RedBoard
#include <Servo.h>

// --- Easy Driver (stepper) ---
#define stp 2
#define dir 3
#define MS1 4
#define MS2 5
#define EN  6

// --- L298N (DC motor) ---
#define IN1 13   // L298N IN1 (L1)
#define IN2 12   // L298N IN2 (L2)
#define ENA 11   // L298N ENA (enable / PWM speed) - remove the ENA jumper on the board

// --- Servo ---
#define SERVO_PIN 9   // Servo signal wire

//Declare variables for functions
char user_input;
int x;
int y;
int state;
int dcSpeed = 200;   // DC motor PWM speed, 0-255

Servo servoMotor;
int servoAngle = 90; // Current servo position in degrees

void setup() {
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  resetEDPins(); //Set step, direction, microstep and enable pins to default states
  DCStop();      //Make sure the DC motor starts off

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(servoAngle); //Start the servo centered

  Serial.begin(9600); //Open Serial connection for debugging
  Serial.println("Begin motor control");
  Serial.println();
  printMenu();
}

//Print function list for user selection
void printMenu()
{
  Serial.println("Enter number for control option:");
  Serial.println("--- Stepper motor (Easy Driver) ---");
  Serial.println("1. Turn at default microstep mode.");
  Serial.println("2. Reverse direction at default microstep mode.");
  Serial.println("3. Turn at 1/8th microstep mode.");
  Serial.println("4. Step forward and reverse directions.");
  Serial.println("--- DC motor (L298N) ---");
  Serial.println("5. DC motor forward.");
  Serial.println("6. DC motor reverse.");
  Serial.println("7. DC motor stop (coast).");
  Serial.println("8. DC motor brake.");
  Serial.println("9. DC motor ramp speed up and down.");
  Serial.println("+ / -  Increase / decrease DC motor speed.");
  Serial.println("--- Servo (pin 9) ---");
  Serial.println("a. Servo to 0 degrees.");
  Serial.println("b. Servo to 90 degrees (center).");
  Serial.println("c. Servo to 180 degrees.");
  Serial.println("d. Servo sweep 0-180-0.");
  Serial.println("s<angle>  Servo to a specific angle, e.g. s45");
  Serial.println("< / >  Nudge servo -10 / +10 degrees.");
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
        Serial.println("DC motor stopped (coasting).");
        Serial.println("Enter new option");
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
        Serial.println("Invalid option entered.");
        printMenu();
      }
  }
}

// ===================== Stepper functions =====================

//Default microstep mode function
void StepForwardDefault()
{
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, LOW); //Pull direction pin low to move "forward"
  for(x= 0; x<1000; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println("Enter new option");
  Serial.println();
}

//Reverse default microstep mode function
void ReverseStepDefault()
{
  Serial.println("Moving in reverse at default step mode.");
  digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
  for(x= 0; x<1000; x++)  //Loop the stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  Serial.println("Enter new option");
  Serial.println();
}

// 1/8th microstep foward mode function
void SmallStepMode()
{
  Serial.println("Stepping at 1/8th microstep mode.");
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
  Serial.println("Enter new option");
  Serial.println();
}

//Forward/reverse stepping function
void ForwardBackwardStep()
{
  Serial.println("Alternate between stepping forward and reverse.");
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
  Serial.println("Enter new option:");
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
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, dcSpeed);
  Serial.print("DC motor forward at speed ");
  Serial.println(dcSpeed);
  Serial.println("Enter new option (7 to stop)");
  Serial.println();
}

//Run DC motor in reverse at current speed (keeps running until stopped)
void DCReverse()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, dcSpeed);
  Serial.print("DC motor reverse at speed ");
  Serial.println(dcSpeed);
  Serial.println("Enter new option (7 to stop)");
  Serial.println();
}

//Stop DC motor and let it coast (bridge disabled)
void DCStop()
{
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

//Actively brake the DC motor (both motor terminals shorted together)
void DCBrake()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);
  delay(300);          //Hold the brake briefly
  DCStop();
  Serial.println("DC motor braked.");
  Serial.println("Enter new option");
  Serial.println();
}

//Ramp the DC motor up to full speed and back down, forward then reverse
void DCRamp()
{
  Serial.println("Ramping DC motor forward...");
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

  Serial.println("Ramping DC motor reverse...");
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
  Serial.println("Ramp complete.");
  Serial.println("Enter new option");
  Serial.println();
}

//Change DC motor speed; applies immediately if the motor is running
void DCChangeSpeed(int delta)
{
  dcSpeed = constrain(dcSpeed + delta, 0, 255);
  // If the motor is currently driving in a direction, update speed live
  if (digitalRead(IN1) != digitalRead(IN2))
  {
    analogWrite(ENA, dcSpeed);
  }
  Serial.print("DC motor speed set to ");
  Serial.println(dcSpeed);
  Serial.println();
}

// ===================== Servo functions =====================

//Move the servo to an angle (clamped to 0-180)
void ServoMoveTo(int angle)
{
  servoAngle = constrain(angle, 0, 180);
  servoMotor.write(servoAngle);
  Serial.print("Servo moved to ");
  Serial.print(servoAngle);
  Serial.println(" degrees.");
  Serial.println("Enter new option");
  Serial.println();
}

//Sweep the servo from 0 to 180 and back, then return to its previous angle
void ServoSweep()
{
  Serial.println("Sweeping servo 0 -> 180 -> 0...");
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
  Serial.println("Sweep complete.");
  Serial.println("Enter new option");
  Serial.println();
}
