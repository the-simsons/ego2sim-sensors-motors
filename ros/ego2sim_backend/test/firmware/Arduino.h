#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
using byte = unsigned char;
struct __FlashStringHelper {};
#define F(x) reinterpret_cast<const __FlashStringHelper *>(x)
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define CHANGE 1
#define PI 3.14159265358979323846
#define TWO_PI (2*PI)
#define DEG_TO_RAD (PI/180.)
#define RAD_TO_DEG (180./PI)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define ISR(vector) void vector()
#define _BV(x) (1<<(x))
#define PB2 2
#define PCINT2 2
#define PCIE0 0
inline int PINB=0,PCMSK0=0,PCICR=0;
inline unsigned long fakeMicros=0;
inline unsigned long millis(){return fakeMicros/1000;}
inline unsigned long micros(){return fakeMicros;}
inline void delayMicroseconds(unsigned int us){fakeMicros+=us;}
inline void noInterrupts(){} inline void interrupts(){}
inline void pinMode(int,int){} inline void digitalWrite(int,int){}
inline int digitalRead(int){return 0;}
inline int analogRead(int){return 500;}
inline void analogWrite(int,int){}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int,void(*)(),int){}
class Print {
 public:
  virtual size_t write(uint8_t c)=0;
  size_t write(const char *s){size_t n=0;while(*s)n+=write((uint8_t)*s++);return n;}
  void print(const __FlashStringHelper *s){write(reinterpret_cast<const char *>(s));}
  void print(const char *s){write(s);}
  void print(char c){write((uint8_t)c);}
  void print(unsigned char c){print((unsigned int)c);}
  template<class T>void print(T v){std::ostringstream s;s<<v;write(s.str().c_str());}
  void print(float v,int n){std::ostringstream s;s<<std::fixed<<std::setprecision(n)<<v;write(s.str().c_str());}
  template<class T>void println(T v){print(v);println();}
  void println(float v,int n){print(v,n);println();}
  void println(){write((uint8_t)'\n');}
};
struct FakeSerial:Print {
 std::string in,out;
 void begin(int baud){if(baud!=115200)throw 1;}
 int available(){return in.size();}
 int availableForWrite(){return 64;}
 void flush(){}
 char read(){char c=in[0];in.erase(0,1);return c;}
 size_t write(uint8_t c)override{out+=(char)c;return 1;}
};
inline FakeSerial Serial;
