"""Projection of firmware sensor packets into the public ROS topics (no ROS imports)."""
import math
from .protocol import NAN

# topic -> (message kind, wire field, calibration group or None)
SENSOR_TOPICS = {
 'sensors/flex/raw': ('scalar','flex_adc',None),
 'sensors/flex/angle': ('scalar','flex_rad','flex'),
 'sensors/flex/angle_filtered': ('scalar','flex_rad_f','flex'),
 'sensors/light/raw_voltage': ('scalar','light_v',None),
 'sensors/light/illuminance': ('lux','lux','light'),
 'sensors/light/illuminance_filtered': ('lux','lux_f','light'),
 'sensors/ir/raw_voltage': ('scalar','ir_v',None),
 'sensors/ir/range': ('ir','ir_m','ir'),
 'sensors/ir/range_filtered': ('ir','ir_m_f','ir'),
 'sensors/tof/range': ('tof','tof_m',None),
 'sensors/ultrasonic/raw_pulse_width': ('scalar','sonar_pulse_us',None),
 'sensors/ultrasonic/range': ('sonar','sonar_m',None),
}
RAW_TOPICS = {name for name,(kind,field,cal) in SENSOR_TOPICS.items()
              if cal is None and kind == 'scalar'}


def sensor_values(record, calibration_verified):
    values = {}
    for topic,(kind,field,cal) in SENSOR_TOPICS.items():
        value = record[field]
        if cal and not calibration_verified.get(cal,False):
            value = NAN
        if kind == 'lux' and value < 0:
            value = NAN
        values[topic] = value
    return values
