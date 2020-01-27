import math
import numpy
from frc3223_azurite.motors import *
from utils import *

class FlywheelSimulation: 
    def __init__(self, **kwargs):
        self.ball_radius_m = inch_to_meter(2.5)
        self.flywheel_radius_m = kwargs.get('flywheel_radius_m', inch_to_meter(1.5))
        self.ball_force = kwargs.get('ball_force', lbs_to_N(4))
        self.flywheel_initial_velocity_radps = kwargs.get('flywheel_initial_velocity_radps', 0)
        self.flywheel_mass_kg = kwargs.get('flywheel_mass_kg', lbs_to_kg(0.22))
        self.entry_time_s = kwargs.get('entry_time', 3)
        self.exit_time_s = kwargs.get('exit_time', 4)
        self.battery_resistance_ohms = kwargs.pop('battery_resistance_ohms', 0.015)
        self.battery_voltage = kwargs.pop('battery_voltage', 12.7)
        self.fuse_resistance_ohms = kwargs.pop('fuse_resistance_ohms', 0.002)
        self.init = kwargs.pop('init', lambda *args, **kwargs: None)
        self.periodic_period = kwargs.pop('periodic_period', 0.02) # default on 20 ms intervals
        self.periodic = kwargs.pop('periodic', lambda *args, **kwargs: None)
        self.motor_system = kwargs['motor_system']

    def flywheel_tangent_force(self):
        torque_Nm = self.motor_system.torque_at_motor_current((self.current))
        f = torque_Nm / self.flywheel_radius_m 
        return f

    def flywheel_moment_of_inertia(self):
        return 0.5 * self.flywheel_mass_kg * self.flywheel_radius_m ** 2

    def flywheel_acceleration(self):
        f1 = self.flywheel_tangent_force() 
        f2 = 0
        if self.ball_in_channel:
            f2 = self.ball_force
            if f1 < 0:
                f2 = -f2
        f = f1 - f2
        
        torque = f * self.flywheel_radius_m
        J = self.flywheel_moment_of_inertia()
        return torque / J

    def current_at_motor(self, state):
        vrot = self.wv
        backemf = self.motor_system.motor_back_emf(vrot)
        voltage = state._voltage_p * self.battery_voltage - backemf
        motor_count = self.motor_system.motor_count
        r1 = self.battery_resistance_ohms
        r2 = self.fuse_resistance_ohms
        rm = self.motor_system.motor.resistance()
        r = r1 * motor_count + r2 + rm

        return voltage / r

    def voltage_at_motor(self, motor_current):
        r1 = self.battery_resistance_ohms
        r2 = self.fuse_resistance_ohms
        vb = self.battery_voltage
        motor_count = self.motor_system.motor_count

        return vb - r1 * motor_count * motor_current - r2 * motor_current

    def run_simulation(self, dt=0.0001, timeout=3):
        with RobotState() as state:
            t = 0.0
            t_last_periodic = 0. # time of last periodic call (s)
            self.ball_in_channel = False
            self.ball_left_channel = False
            self.current = 0
            dx = 0 # change in ball position since t=0, rad
            self.wv = self.flywheel_initial_velocity_radps # flywheel velocity rad/s
            self.wa = self.flywheel_acceleration() # flywheel acceleration rad/s^2
            self.current = self.current_at_motor(state)
            self.voltage = self.voltage_at_motor(self.current)
            i = 0
            n = (int(timeout/dt) + 2)
            def buf():
                b = numpy.zeros(shape=(n,))
                b.fill(numpy.nan)
                return b
            self.ts = buf()
            self.wvs = buf()
            self.was = buf()
            self.fslips = buf()
            self.currents = buf()
            self.voltages = buf()
            self.voltageps = buf()

            def log():
                nonlocal i, t, dx
                self.ts[i] = t
                self.wvs[i] = self.wv
                self.was[i] = self.wa
                self.currents[i] = self.current
                self.voltages[i] = self.voltage
                self.voltageps[i] = state._voltage_p
                self.fslips[i] = 1.0 if self.ball_in_channel else 0.0
                i += 1

            state._update(t, self.wa, self.wv, self.current, self.voltage)
            self.init(state)
            log()

            while t < timeout:
                self.wv += self.wa * dt
                self.wa = self.flywheel_acceleration()
                self.current = self.current_at_motor(state)
                self.voltage = self.voltage_at_motor(self.current)
                t += dt
                state._update(t, self.wa, self.wv, self.current, self.voltage)
                log()

                if t - t_last_periodic > self.periodic_period:
                    t_last_periodic = t
                    self.periodic(state)

                if self.ball_in_channel and t > self.exit_time_s:
                    self.ball_in_channel = False
                    self.ball_left_channel = True
                    print ("exited channel, t=%f" % (t))
                if t > self.entry_time_s and not self.ball_in_channel and not self.ball_left_channel:
                    self.ball_in_channel = True
                    print ("entered channel, t=%f" % (t))


class RobotState:
    def __init__(self):
        self.motor_current = 0.0
        self._update(0.0, 0.0, 0.0, 0.0, 0.0)
        self._voltage_p = 0.0
        self.motor = None
        self.stop = False

    def __enter__(self):
        #hal_impl.functions.hooks = SimHooks(self)
        #hal_impl.functions.reset_hal()
        self.motor = SimVictor(self)
        return self

    def __exit__(self, type, value, traceback):
        self.motor = None

    def _update(self, t, wa, wv, current, voltage):
        self.time_from_start_s = t
        self.acceleration_radps2 = wa
        self.velocity_radps = wv
        self.motor_current = current
        self.voltage = voltage


class SimVictor:
    def __init__(self, state):
        '''
        :type state: RobotState
        '''
        self.state = state

    def set(self, voltage_p):
        '''
        :type voltage_p: float
        '''
        self.state._voltage_p = voltage_p

