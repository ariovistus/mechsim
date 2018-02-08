import math
import numpy
from motors import *
from utils import *


class SpinSimulator:
    """
    spinning an unconstrained wheel
    """

    def __init__(self, **kwargs):
        self.flywheel_radius_m = kwargs.get('flywheel_radius_m', inch_to_meter(1.5))
        self.flywheel_initial_velocity_radps = kwargs.get('flywheel_initial_velocity_radps', 0)
        self.flywheel_mass_kg = kwargs.get('flywheel_mass_kg', lbs_to_kg(0.22))
        self.motor_system = kwargs['motor_system']

    def flywheel_moment_of_inertia(self):
        return 0.5 * self.flywheel_mass_kg * self.flywheel_radius_m ** 2

    def accel(self):
        torque_Nm = self.motor_system.torque_at_speed(radps_to_rpm(self.v))
        return torque_Nm / self.flywheel_moment_of_inertia()

    def current(self):
        torque_Nm = self.motor_system.torque_at_speed(radps_to_rpm(self.v))
        return self.motor_system.motor_current_at_torque(torque_Nm)

    def run_sim(self, dt=0.01, timeout=10):
        t = 0.0 # seconds
        self.v = 0.0 # rad/sec
        a = 0.0 # rad/sec^2
        i = 0.0 # Amps
        n = (int(timeout/dt) + 1)
        def buf():
            b = numpy.zeros(shape=(n,))
            b.fill(numpy.nan)
            return b
        self.ts = buf() # seconds
        self.vs = buf() # rad/sec
        self.a_s = buf() # rad/sec^2
        self.i_s = buf() # Amps

        j = 0
        def log():
            nonlocal j, t, self, a, i
            self.ts[j] = t
            self.vs[j] = self.v
            self.a_s[j] = a
            self.i_s[j] = i
            j += 1

        log()

        while True:
            a = self.accel()
            self.v += a * dt
            i = self.current()

            t += dt
            if t >= timeout:
                break
            log()

