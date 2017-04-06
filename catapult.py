import math

motor_stall_torque = 1.789 # ft-lbs
motor_free_speed = 556.1 # rad per sec
radius = 2.33 # ft
gearing_ratio = 20

arm_stall_torque = motor_stall_torque * gearing_ratio
arm_free_speed = motor_free_speed / gearing_ratio
inertia = 0.193 # slug ft ** 2

dt = 0.001

def _torque(_arm_speed):
    return arm_stall_torque * (arm_free_speed  - _arm_speed) / arm_free_speed

def _accel(_torque):
    return _torque / inertia


time = 0.
arm_speed = 0.
arm_position = 0.
accel = 0.
torque = _torque(arm_speed)
launch_velocity = 0.

while arm_position < math.pi / 4:
    time += dt
    accel = _accel(torque)
    arm_speed += accel * dt
    arm_position += arm_speed * dt
    launch_velocity = arm_speed * radius
    torque = _torque(arm_speed)
    print("T: %.3f   torque: %4.2f   accel: %6.2f   vel: %6.2f   position: %3d   launch: %3.1f" % (
        time, torque, accel, arm_speed, math.degrees(arm_position), launch_velocity))
