#include <Wire.h>
#include <SparkFun_VL53L1X.h>

const int FLEX_PIN = A1;
const int IR_PIN = A2;

const float FIXED_RESISTOR = 10000.0;  // 10 kOhm
const float ADC_REFERENCE_V = 5.0;
const unsigned long READ_INTERVAL_MS = 100;

SFEVL53L1X tof;
bool tofAvailable = false;

unsigned long lastAnalogReading = 0;
unsigned long lastTofReading = 0;

// Average samples, discarding the first after switching channels.
float readAverage(int pin) {
  analogRead(pin);
  delayMicroseconds(200);

  long total = 0;

  for (int i = 0; i < 10; i++) {
    total += analogRead(pin);
    delay(1);
  }

  return total / 10.0;
}

void setup() {
  Serial.begin(115200);

  pinMode(FLEX_PIN, INPUT);
  pinMode(IR_PIN, INPUT);

  Wire.begin();
  Wire.setClock(100000);

  tofAvailable = (tof.begin() == 0);

  if (tofAvailable) {
    tof.setDistanceModeShort();
    tof.setTimingBudgetInMs(50);
    tof.setIntermeasurementPeriod(100);
    tof.startRanging();

    lastTofReading = millis();
    Serial.println("TOF ready");
  } else {
    Serial.println("ERROR: TOF not detected. Analog readings continue.");
  }

  Serial.println("FLEX=A1 IR=A2");
}

void loop() {
  unsigned long now = millis();

  if (now - lastAnalogReading >= READ_INTERVAL_MS) {
    lastAnalogReading = now;

    // Flex sensor on A1.
    float flexAdc = readAverage(FLEX_PIN);
    float flexVoltage = flexAdc * ADC_REFERENCE_V / 1023.0;

    Serial.print("FLEX ADC=");
    Serial.print(flexAdc, 1);
    Serial.print(" Voltage_V=");
    Serial.print(flexVoltage, 3);

    if (flexAdc > 0) {
      float flexResistance =
          FIXED_RESISTOR * (1023.0 / flexAdc - 1.0);

      Serial.print(" Resistance_kOhm=");
      Serial.println(flexResistance / 1000.0, 2);
    } else {
      Serial.println(" ERROR=Check_wiring");
    }

    // Sharp infrared distance sensor on A2.
    float irAdc = readAverage(IR_PIN);
    float irVoltage = irAdc * ADC_REFERENCE_V / 1023.0;

    Serial.print("IR ADC=");
    Serial.print(irAdc, 1);
    Serial.print(" Voltage_V=");
    Serial.println(irVoltage, 3);
  }

  // VL53L1X ToF sensor over I2C.
  if (tofAvailable) {
    if (tof.checkForDataReady()) {
      int distanceMm = tof.getDistance();
      byte status = tof.getRangeStatus();
      tof.clearInterrupt();

      Serial.print("TOF Status=");
      Serial.print(status);

      if (status == 0) {
        Serial.print(" Distance_mm=");
        Serial.println(distanceMm);
      } else {
        Serial.println(" Invalid_measurement");
      }

      lastTofReading = millis();
    }

    if (millis() - lastTofReading >= 2000) {
      Serial.println("TOF ERROR=No_new_measurement");
      lastTofReading = millis();
    }
  }
}
