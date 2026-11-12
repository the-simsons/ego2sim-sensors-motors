#include "Arduino.h"
#include MAIN_SKETCH
#include <iostream>
#include <cassert>
void receive(const std::string &line){
 Serial.in+=line+"\n";
 readSerial();
}
int main(){
 setup();
 sampleAnalog();
 tofM=.650;sonarUs=3528;sonarM=.6096;
 sendSensors(1234);tx.drain();
 receive("C,42,1,1,1.57079633");
 assert(motors[1].lastCommandId==42);
 long target=dcTargetCount;
 receive("C,42,1,1,1.57079633");
 assert(dcTargetCount==target);
 sendMotor(1235,1);tx.drain();
 receive("C,43,2,1,0.2");
 long steps=stepTarget;
 assert(steps==51);
 for(int n=0;n<60;n++){fakeMicros+=1001;stepperUpdate();}
 assert(stepPos==51);
 sendMotor(1236,2);tx.drain();
 receive("C,44,1,2,20");tx.drain();
 receive("M,1");tx.drain();
 receive("C,45,1,1,1");tx.drain();
 receive("C,46,1,3,0");
 assert(motors[1].held && dcMode==DC_IDLE);
 sendMotor(1237,1);tx.drain();
 receive("M,0");tx.drain();
 assert(!motors[1].held);
 receive("C,47,0,0,1.57079633");
 assert(servoCmdDeg==90);
 sendMotor(1238,0);tx.drain();
 std::cout<<Serial.out;
}
