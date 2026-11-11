#pragma once
struct FakeWire { void begin() {} void setClock(int) {} };
inline FakeWire Wire;
