#include "Arduino.h"
#include MAIN_SKETCH
#include <cassert>
#include <iostream>
void receive(const std::string &line) {
  Serial.in += line;
  while (Serial.available()) readCommands();
}
int main() {
  setup();
  assert(Serial.out == "B,1\n");
  assert(configuredPins == std::vector<int>({A1,A2}));
  receive("X,1,123\n");
  assert(session == 123 && !watchdogExpired);
  Serial.out.clear();
  receive("C,1,123,7,1,1,1.0\n");
  assert(Serial.out == "A,1,123,7,0,MOTOR_NOT_IMPLEMENTED\n");
  Serial.out.clear();
  receive("C,1,123,8,0,0,4\n");
  assert(Serial.out == "A,1,123,8,0,LIMIT_OR_OPERATION\n");
  Serial.out.clear();
  receive("C,1,123,9,0,3,0\n");
  assert(Serial.out == "A,1,123,9,0,MOTOR_NOT_IMPLEMENTED\n");
  Serial.out.clear();
  receive("C,1,123,10,0,0,nan\n");
  assert(Serial.out == "A,1,123,10,0,INVALID\n");
  Serial.out.clear();
  receive("X,1,124\n"); Serial.out.clear();
  receive("C,1,123,11,0,0,1\n");
  assert(Serial.out.empty());
  receive("M,1,124,1\n");
  assert(Serial.out.find(",0,0\n") != std::string::npos);
  Serial.out.clear();
  receive(std::string(200,'x')+"\nP,1,124\n");
  assert(Serial.out.find("H,1,124,") == 0);
  fakeMillis += 1600; loop(); assert(watchdogExpired);
  assert(Serial.out.find(",light,1,nan\n") != std::string::npos);
  assert(Serial.out.find(",ultrasonic,1,nan\n") != std::string::npos);
  unsigned long x;
  assert(!uint32Value("4294967296",x));
  assert(uint32Value("4294967295",x));
  std::cout << "firmware protocol, bounds, disabled pins and watchdog passed\n";
}
