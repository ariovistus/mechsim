import math 
import wpilib
import numpy
import hal
from hal_impl.sim_hooks import SimHooks as BaseSimHooks
import hal_impl.functions
from frc3223_azurite.conversions import (
    g, 
    lbs_to_kg, 
    lbs_to_N,
    inch_to_meter,
    radps_to_rpm,
)


class IntakeSimulation:
    """
    a claw mechanism driven by springs and cables that I'm really glad the frc team isn't using.

    unfinished
    """

    def __init__(self, periodic, motor_system, **kwargs):
        self.dt_s = kwargs.pop('dt_s', 0.0001)
        self.nominal_voltage = kwargs.pop('nominal_voltage', 12.)
        self.starting_position_rad = kwargs.pop('starting_position_rad', 0.)
        self.spool_radius_m = kwargs.pop('spool_radius_m', inch_to_meter(0.5))
        self.spur_radius_m = kwargs.pop('spur_radius_m', inch_to_meter(2))
        self.spur_offset_rad = kwargs.pop('spur_offset_rad', math.radians(120))
        self.theta_eqbm = kwargs.pop('theta_eqbm', math.radians(90))
        self.x1_m = kwargs.pop('x1_m', inch_to_meter(0.5))
        self.x2_m = kwargs.pop('x2_m', inch_to_meter(1.5))
        self.intake_inertial = kwargs.pop('intake_inertial', inch_to_meter(5) ** 2 * lbs_to_kg(5))
        self.spring_constant = kwargs.pop('spring_constant', 50)
        self.gearbox_efficiency = kwargs.pop('gearbox_efficiency', 0.65)
        self.battery_resistance_ohms = kwargs.pop('battery_resistance_ohms', 0.015)
        self.battery_voltage = kwargs.pop('battery_voltage', 12.7)
        self.fuse_resistance_ohms = kwargs.pop('fuse_resistance_ohms', 0.002)
        self.pid_sample_rate= kwargs.pop('pid_sample_rate_s', 0.05)
        self.init = kwargs.pop('init', lambda *args, **kwargs: None)
        assert 0 <= self.gearbox_efficiency <= 1
        assert len(kwargs) == 0, 'Unknown parameters: ' + ', '.join(kwargs.keys())
        self.periodic = periodic
        self.periodic_period = 0.02 # called on 20 ms intervals
        self.motor_system = motor_system

        self.ts = None
        self.a_s = None
        self.vs = None
        self.xs = None
        self.voltages = None
        self.voltageps = None
        self.currents = None

    def displacement(self, theta_rad):
        x1 = self.x1_m
        x2 = self.x2_m
        return x1 ** 2 + x2 ** 2 - 2 * x1 * x2 * numpy.cos(theta_rad)

    def spring_torque(self, theta_rad):
        x = self.displacement(theta_rad)
        x0 = self.displacement(self.theta_eqbm)
        x1 = self.x1_m
        x2 = self.x2_m
        k = self.spring_constant

        torque = k * abs(x - x0) * x1 * numpy.sqrt(1 - (numpy.sin(theta_rad) * x2 / x) ** 2)
        return torque

    def spool_torque(self, motor_torque, theta_rad):
        a = self.spur_offset_rad - theta_rad
        r2 = self.spur_radius_m
        r1 = self.spool_radius_m
        torque = 0.5 * motor_torque / r1 * r2 * numpy.sin(a)
        return torque

    def calc_acceleration(self, state, current):
        motor_torque = self.gearbox_efficiency * self.motor_system.torque_at_motor_current(current)
        torque = (
            self.spool_torque(motor_torque, state.theta_rad) -
            self.spring_torque(state.theta_rad)
        )

        a = torque / self.intake_inertial

        return a

    def current_at_motor(self, state):
        vrot = self.v * self.spur_radius_m / self.spool_radius_m
        backemf = self.motor_system.motor_back_emf(vrot)
        voltage = state.voltage_p * self.battery_voltage - backemf
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

    def init_log_buffers(self, max_size):
        def make_buffer():
            return numpy.empty(shape=(max_size,),dtype='float')
        self.ts = make_buffer()
        self.a_s = make_buffer()
        self.vs = make_buffer()
        self.thetas = make_buffer()
        self.voltages = make_buffer()
        self.voltageps = make_buffer()
        self.currents = make_buffer()

    def log(self, state):
        self.ts[self.i] = state.time_from_start_s
        self.a_s[self.i] = state.acceleration_radps2
        self.vs[self.i] = state.velocity_radps
        self.thetas[self.i] = state.theta_rad
        self.voltageps[self.i] = state.voltage_p
        self.voltages[self.i] = state.voltage
        self.currents[self.i] = state.motor_current
        self.i += 1

    def run_sim(self, timeout=10., sample_rate=None):
        if sample_rate is None:
            sample_rate = self.dt_s
        with RobotState() as state:
            dt = self.dt_s # time delta (s)
            t = 0. # time (s)
            t_last_periodic = 0. # time of last periodic call (s)
            t_last_measurement = 0. # time of last data log (s)
            t_last_pid = 0. # time of last pid calculation (s)
            self.v = 0. # velocity (rad/s)
            theta = self.starting_position_rad # position (rad)
            self.init_log_buffers(int(timeout / sample_rate) + 1)
            self.i = 0
            current = self.current_at_motor(state)
            voltage = self.voltage_at_motor(current)
            state._update(t, 0.0, self.v, theta, current, voltage)
            self.init(state)
            a = self.calc_acceleration(state, current) # acceleration (rad/s^2)
            state._update(t, a, self.v, theta, current, voltage)
            self.log(state)

            while t < timeout and not state.stop:
                self.v += a * dt
                theta += self.v * dt
                t += dt
                current = self.current_at_motor(state)
                voltage = self.voltage_at_motor(current)
                a = self.calc_acceleration(state, current)
                state._update(t, a, self.v, theta, current, voltage)
                if t - t_last_periodic > self.periodic_period:
                    t_last_periodic = t
                    self.periodic(state)
                if t - t_last_measurement > sample_rate:
                    self.log(state)
                    t_last_measurement = t
                if t - t_last_pid > self.pid_sample_rate and state.pid is not None:
                    state.pid._calculate()
                    t_last_pid = t

            self.trim_buffers()

    def trim_buffers(self):
        buffers = ['ts', 'a_s', 'vs', 'thetas', 'voltages', 'voltageps', 'currents']
        for buf in buffers:
            arr = getattr(self,buf)
            setattr(self, buf, arr[:self.i])


class RobotState:
    def __init__(self):
        self.pid = None
        self.motor_current = 0.0
        self._update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._voltage_p = 0.0
        self.motor = None
        self.stop = False

    def __enter__(self):
        hal_impl.functions.hooks = SimHooks(self)
        hal_impl.functions.reset_hal()
        self.motor = SimVictor(self)
        return self

    def __exit__(self, type, value, traceback):
        self.motor.free()
        self.motor = None

    def _update(self, t, a, v, theta, current, voltage):
        self.theta_rad = theta
        self.acceleration_radps2 = a
        self.velocity_radps = v
        self.time_from_start_s = t
        self.motor_current = current
        self.voltage = voltage
        if self.pid:
            self.pid.setpointTimer.time_seconds = t

    @property
    def velocity_rpm(self):
        return radps_to_rpm(self.velocity_radps)

    def set_voltage_p(self, voltage):
        '''
        :type voltage: float
        '''
        self._voltage_p = max(-1.0, min(1.0, voltage))
    
    def get_voltage_p(self):
        return self._voltage_p

    voltage_p = property(get_voltage_p, set_voltage_p)


class SimVictor(wpilib.Victor):
    def __init__(self, state):
        '''
        :type state: RobotState
        '''
        super().__init__(1)
        self.state = state

    def set(self, voltage_p):
        '''
        :type voltage_p: float
        '''
        super().set(voltage_p)
        self.state.voltage_p = voltage_p

class SimHooks(BaseSimHooks):
    def __init__(self, state):
        '''
        :type state: RobotState
        '''
        super().__init__()
        self.state = state

    def getTime(self):
        return self.state.time_from_start_s

    def getFPGATime(self):
        from hal_impl.data import hal_data
        # post commit a518 this doesn't need to be implemented
        return int((self.getTime() - hal_data['time']['program_start']) * 1000000)


if __name__ == '__main__':
    from frc3223_azurite.motors import *

    def init(state):
        state.voltage_p = 1.0
        
    def periodic(state):
        state.voltage_p = 0.0

    IntakeSimulation(
        dt_s=0.0001,
        periodic = periodic,
        init = init,
        motor_system=MotorSystem(motor=bag, motor_count=1, gearing_ratio=64),
    ).run_sim()
