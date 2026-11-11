
const int PW_PIN = 6;

// LV-EZ scale: 147 microseconds per inch
const float US_PER_INCH = 147.0;

// Max pulse is ~37.5 ms (no target); 50 ms timeout keeps pulseIn from hanging
const unsigned long PULSE_TIMEOUT_US = 50000UL;

void setup() {
  Serial.begin(9600);
  pinMode(PW_PIN, INPUT);
  delay(250);  // sensor needs ~250 ms after power-up before the first reading
}

void loop() {
  unsigned long pulseUs = pulseIn(PW_PIN, HIGH, PULSE_TIMEOUT_US);

  if (pulseUs == 0) {
    Serial.println("No pulse (check wiring)");
  } else {
    float inches = pulseUs / US_PER_INCH;
    float cm = inches * 2.54;

    Serial.print("Pulse: ");
    Serial.print(pulseUs);
    Serial.print(" us   Range: ");
    Serial.print(inches, 1);
    Serial.print(" in   ");
    Serial.print(cm, 1);
    Serial.println(" cm");
  }

  delay(50);  // sensor updates every ~49 ms
}
