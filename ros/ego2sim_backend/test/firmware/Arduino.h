#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
using byte = unsigned char;
struct __FlashStringHelper {};
#define F(x) reinterpret_cast<const __FlashStringHelper *>(x)
#define A1 15
#define A2 16
#define INPUT 0
#define PI 3.14159265358979323846
inline unsigned long fakeMillis = 0;
inline unsigned long millis() { return fakeMillis; }
inline void delay(unsigned long ms) { fakeMillis += ms; }
inline void delayMicroseconds(unsigned int) {}
inline std::vector<int> configuredPins;
inline void pinMode(int p, int) { configuredPins.push_back(p); }
inline int analogRead(int) { return 500; }
struct FakeSerial {
  std::string in, out;
  void begin(int baud) { if (baud != 115200) throw 1; }
  int available() { return in.size(); }
  char read() { char c=in[0]; in.erase(0,1); return c; }
  void print(const __FlashStringHelper *s) { out += reinterpret_cast<const char *>(s); }
  void print(const char *s) { out += s; }
  void print(char c) { out += c; }
  template<class T> void print(T v) { std::ostringstream s; s << v; out += s.str(); }
  template<class T> void println(T v) { print(v); out += '\n'; }
  void println(float v, int n) { std::ostringstream s; s << std::fixed << std::setprecision(n) << v; out += s.str()+'\n'; }
};
inline FakeSerial Serial;
