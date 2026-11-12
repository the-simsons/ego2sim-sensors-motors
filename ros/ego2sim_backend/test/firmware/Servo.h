#pragma once
struct Servo {
  bool on = false;
  void attach(int) { on = true; }
  void detach() { on = false; }
  bool attached() const { return on; }
  void write(int) {}
};
