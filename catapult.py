import math
from frc3223_azurite.motors import *
from frc3223_azurite.conversions import *
from frc3223_azurite.inertials import (
    rod_about_end,
    solid_sphere,
    parallel_axis,
)

class CatapultSimulation:
    def __init__(self, **kwargs):
        self.ball_mass_kg = kwargs.get('ball_mass_kg', lbs_to_kg(0.65))
        self.ball_radius_m = kwargs.get('ball_radius_m', inch_to_meter(5))
        self.arm_length_m = kwargs.get('arm_length_m', inch_to_meter(24))
        self.arm_mass_kg = kwargs.get('arm_mass_kg', lbs_to_kg(1))
        self.motor_system = kwargs['motor_system']

    def inertial(self):
        return (
                rod_about_end(self.arm_mass_kg, self.arm_length_m) + 
                solid_sphere(self.ball_mass_kg, self.ball_radius_m) +
                parallel_axis(self.ball_mass_kg, self.arm_length_m)
                )

    def torque(self):
        return self.motor_system.torque_at_speed(self.arm_speed)

    def accel(self):
        return self.torque() / self.inertial()

    def run_sim(self, dt=0.001):
        time = 0. # s
        self.arm_speed = 0. # rad/s
        arm_position = 0. # rad
        accel = 0. # rad/s^2
        torque = self.torque() # Nm
        launch_velocity = 0. # ft/s

        while arm_position < math.pi / 4:
            time += dt
            accel = self.accel()
            self.arm_speed += accel * dt
            arm_position += self.arm_speed * dt
            launch_velocity = self.arm_speed * meter_to_feet(self.arm_length_m)
            torque = self.torque()
            print("T: %.3f   torque: %4.2f   accel: %6.2f   vel: %6.2f   position: %3d   launch: %3.1f" % (
                time, torque, accel, self.arm_speed, math.degrees(arm_position), launch_velocity))


if __name__ == '__main__':
    sim = CatapultSimulation(
        arm_length_m=feet_to_meter(2.33),
        arm_mass_kg=lbs_to_kg(1.5),
        motor_system=MotorSystem(motor=cim, motor_count=1, gearing_ratio=25),
    )
    sim.run_sim()
