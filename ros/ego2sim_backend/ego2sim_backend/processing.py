"""ROS-independent calibration and filtering. No factory curves are invented."""
import math
NAN = float('nan')

class Calibration:
    def __init__(self, x=(), y=()):
        self.x, self.y = tuple(x), tuple(y)
        if len(x) != len(y) or (x and len(x) < 2):
            raise ValueError('Calibration needs at least two paired points, or empty arrays')
        if any(not math.isfinite(v) for v in (*x, *y)):
            raise ValueError('Calibration points must be finite')
        if any(a >= b for a, b in zip(x, x[1:])):
            raise ValueError('Calibration input points must be strictly increasing')

    def convert(self, value):
        if not self.x or not math.isfinite(value) or not self.x[0] <= value <= self.x[-1]:
            return NAN
        for i in range(1, len(self.x)):
            if value <= self.x[i]:
                t = (value-self.x[i-1])/(self.x[i]-self.x[i-1])
                return self.y[i-1] + t*(self.y[i]-self.y[i-1])
        return NAN

class LowPass:
    def __init__(self, tau=0.2, stale=1.0):
        if not math.isfinite(tau) or tau < 0 or not math.isfinite(stale) or stale <= 0:
            raise ValueError('Invalid filter timing')
        self.tau, self.stale = tau, stale
        self.value, self.stamp = NAN, None

    def update(self, value, stamp):
        if not math.isfinite(value):
            self.value, self.stamp = NAN, None
            return NAN
        dt = None if self.stamp is None else stamp-self.stamp
        if dt is None or dt <= 0 or dt > self.stale or not math.isfinite(self.value):
            self.value = value
        else:
            a = 1.0 if self.tau == 0 else -math.expm1(-dt/self.tau)
            self.value += a*(value-self.value)
        self.stamp = stamp
        return self.value

def sonar(voltage, supply):
    if not (math.isfinite(supply) and supply > 0):
        raise ValueError('Sonar supply must be positive')
    inches = voltage*512.0/supply
    # Six-inch output also represents all objects closer than six inches.
    if not math.isfinite(inches) or inches < 6-1e-9 or inches > 254+1e-9:
        return NAN
    return min(254., max(6., inches))*0.0254
