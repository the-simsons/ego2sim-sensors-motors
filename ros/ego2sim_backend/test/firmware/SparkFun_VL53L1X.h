#pragma once
struct SFEVL53L1X {
 bool begin() { return 0; }
 void setDistanceModeShort() {} void setTimingBudgetInMs(int) {}
 void setIntermeasurementPeriod(int) {} void startRanging() {}
 bool checkForDataReady() { return false; }
 int getDistance() { return 600; } unsigned char getRangeStatus() { return 0; }
 void clearInterrupt() {}
};
