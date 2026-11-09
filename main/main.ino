const int FLEX_PIN = A1;
const float FIXED_RESISTOR = 10000.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  long total = 0;

  for (int i = 0; i < 10; i++) {
    total += analogRead(FLEX_PIN);
    delay(2);
  }

  float reading = total / 10.0;
  float voltage = reading * 5.0 / 1023.0;

  Serial.print("ADC=");
  Serial.print(reading, 1);
  Serial.print(" Voltage_V=");
  Serial.print(voltage, 2);

  if (reading > 0) {
    float resistance =
        FIXED_RESISTOR * (1023.0 / reading - 1.0);

    Serial.print(" Resistance_kOhm=");
    Serial.println(resistance / 1000.0, 1);
  } else {
    Serial.println(" ERROR=Check_wiring");
  }

  delay(100);
}