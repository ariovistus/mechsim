import math
from motors import *
from utils import *

def run_sim(radius, motor_system):
    arm_stall_torque = motor_system.stall_torque # N m
    arm_free_speed = rpm_to_radps(motor_system.free_speed) # rad/s
    inertia = 0.193 * 14.5939 * 0.3048 ** 2 # kg m ** 2

    def _torque(arm_spd):
        return arm_stall_torque * (arm_free_speed  - arm_spd) / arm_free_speed

    def _accel(_torque):
        return _torque / inertia

    dt = 0.001
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
        launch_velocity = arm_speed * meter_to_inch(radius ) / 12.
        torque = _torque(arm_speed)
        print("T: %.3f   torque: %4.2f   accel: %6.2f   vel: %6.2f   position: %3d   launch: %3.1f" % (
            time, torque, accel, arm_speed, math.degrees(arm_position), launch_velocity))


if __name__ == '__main__':
    run_sim(
        radius=inch_to_meter(12*2.33), 
        motor_system=MotorSystem(motor=cim, motor_count=1, gearing_ratio=20))
