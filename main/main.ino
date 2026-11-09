const int FLEX_PIN = A1;
const float FIXED_RESISTOR = 10000.0;  // 10 kΩ
const float SUPPLY_VOLTAGE = 5.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Average several readings to reduce noise.
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(FLEX_PIN);
    delay(2);
  }

  float reading = total / 10.0;
  float voltage = reading * SUPPLY_VOLTAGE / 1023.0;

  Serial.print("ADC: ");
  Serial.print(reading, 1);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);

  if (reading > 0) {
    float resistance =
        FIXED_RESISTOR * (1023.0 / reading - 1.0);

    Serial.print(" V | Resistance: ");
    Serial.print(resistance / 1000.0, 1);
    Serial.println(" kOhm");
  } else {
    Serial.println(" V | Check sensor wiring");
  }

  delay(100);
}