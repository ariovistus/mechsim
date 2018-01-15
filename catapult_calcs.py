import math
from frc3223_azurite.conversions import inch_to_meter, g, meter_to_inch, radps_to_rpm

angle = math.radians(45)
arm_length_in = 25
base_height_in = 7
assert base_height_in + arm_length_in < 55
h_displ = inch_to_meter(72-arm_length_in)
v_displ = inch_to_meter(79 - (base_height_in + arm_length_in + (arm_length_in - 6.5) * math.sin(angle)))

print ('horizontal displacement: %s m' % h_displ)
print ('vertical displacement: %s m' % (v_displ))

# t = h_displ / (v cos(angle))

vx = math.sqrt(0.5 * g * h_displ ** 2 / (math.tan(angle) * h_displ - v_displ))
v = vx / math.cos(angle)
print ('linear velocity: %s m/s' % v)
w = v / inch_to_meter(arm_length_in)
print ('rotational velocity: %s rad/s (%s deg/s) (%s rpm)' % (w, math.degrees(w), radps_to_rpm(w)))
print ('time to travel: %s s' %( (angle + math.radians(90)) / w))
