#include <Wire.h>
#include <SparkFun_VL53L1X.h>

const int FLEX_PIN = A1;
const float FIXED_RESISTOR = 10000.0;  // Measured 10 kOhm
const unsigned long FLEX_INTERVAL_MS = 100;

SFEVL53L1X tof;
bool tofAvailable = false;

unsigned long lastFlexReading = 0;
unsigned long lastTofReading = 0;

void setup() {
  Serial.begin(115200);

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
    Serial.println("ERROR: TOF not detected. Flex readings continue.");
  }
}

void loop() {
  unsigned long now = millis();

  // Read the flex sensor approximately every 100 ms.
  if (now - lastFlexReading >= FLEX_INTERVAL_MS) {
    lastFlexReading = now;

    long total = 0;
    for (int i = 0; i < 10; i++) {
      total += analogRead(FLEX_PIN);
      delay(2);
    }

    float adc = total / 10.0;
    float voltage = adc * 5.0 / 1023.0;

    Serial.print("FLEX ADC=");
    Serial.print(adc, 1);
    Serial.print(" Voltage_V=");
    Serial.print(voltage, 2);

    if (adc > 0) {
      float resistance =
          FIXED_RESISTOR * (1023.0 / adc - 1.0);

      Serial.print(" Resistance_kOhm=");
      Serial.println(resistance / 1000.0, 2);
    } else {
      Serial.println(" ERROR=Check_wiring");
    }
  }

  // Read the ToF sensor whenever a measurement is ready.
  if (tofAvailable) {
    if (tof.checkForDataReady()) {
      int distance = tof.getDistance();
      byte status = tof.getRangeStatus();
      tof.clearInterrupt();

      Serial.print("TOF Status=");
      Serial.print(status);

      if (status == 0) {
        Serial.print(" Distance_mm=");
        Serial.println(distance);
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