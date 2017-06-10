import math 
import numpy
from utils import (
    g, 
    lbs_to_kg, 
    inch_to_meter,
    radps_to_rpm,
)
from pidcontroller import NoThreadingPIDController


class ArmSimulation:
    def __init__(self, periodic, motor_system, **kwargs):
        self.dt_s = kwargs.get('dt_s', 0.0001)
        self.nominal_voltage = kwargs.get('nominal_voltage', 12.)
        self.starting_position_rad = kwargs.get('starting_position_rad', 0.)
        self.end_mass_kg = kwargs.get('end_mass_kg', lbs_to_kg(5.0))
        self.arm_mass_kg = kwargs.get('arm_mass_kg', lbs_to_kg(2.0))
        self.arm_length_m = kwargs.get('arm_length_m', inch_to_meter(12.))
        self.periodic = periodic
        self.periodic_period = 0.02 # called on 20 ms intervals
        self.motor_system = motor_system
        self.init = kwargs.get('init', lambda *args, **kwargs: None)

    def calc_acceleration(self, state):
        multiplier = state.voltage_p * self.nominal_voltage / 12.
        motor_torque = self.motor_system.torque_at_speed(state.velocity_rpm) * multiplier
        gravity_torque = (
            self.end_mass_kg * g * self.arm_length_m * math.cos(state.theta_rad) +
            self.arm_mass_kg * g * self.arm_length_m / 2. * math.cos(state.theta_rad)
        )

        J_arm = self.arm_mass_kg * self.arm_length_m ** 2 / 3.
        J_end = self.end_mass_kg * self.arm_length_m ** 2
        moment_of_inertia = J_arm + J_end
        return (motor_torque - gravity_torque) / moment_of_inertia

    def run_sim(self, timeout=10., sample_rate=None, pid_sample_rate=0.05):
        if sample_rate is None:
            sample_rate = self.dt_s
        state = RobotState()
        dt = self.dt_s
        t = 0.
        t_last_periodic = 0.
        t_last_measurement = 0.
        t_last_pid = 0.
        v = 0.
        theta = self.starting_position_rad
        def make_buffer():
            return numpy.empty(shape=(int(timeout / sample_rate)+1,),dtype='float')
        ts = make_buffer()
        a_s = make_buffer()
        vs = make_buffer()
        thetas = make_buffer()
        voltages = make_buffer()
        i = 0
        state._update(t, v, theta)
        def pid_source():
            return theta
        def pid_output(v):
            state.voltage_p = v
        state.pid = NoThreadingPIDController(Kp = 1.0, Ki = 0.3, Kd = 0.4, source = pid_source, output = pid_output)
        self.init(state)
        def log():
            nonlocal i, t, a, v, theta, state
            ts[i] = t
            a_s[i] = a
            vs[i] = v
            thetas[i] = theta
            voltages[i] = state.voltage_p
            i += 1
        a = self.calc_acceleration(state)
        log()

        while t < timeout:
            v += a * dt
            theta += v * dt
            t += dt
            a = self.calc_acceleration(state)
            state._update(t, v, theta)
            if t - t_last_periodic > self.periodic_period:
                t_last_periodic = t
                self.periodic(state)
            if t - t_last_measurement > sample_rate:
                log()
            if t - t_last_pid > pid_sample_rate:
                state.pid._calculate()

        return ts, a_s, vs, thetas, voltages


class RobotState:
    def __init__(self):
        self.pid = None
        self._update(0.0, 0.0, 0.0)
        self._voltage_p = 0.0

    def _update(self, t, v, theta):
        self.theta_rad = theta
        self.velocity_radps = v
        self.time_from_start_s = t
        if self.pid:
            self.pid.setpointTimer.time_seconds = t

    @property
    def velocity_rpm(self):
        return radps_to_rpm(self.velocity_radps)

    def set_voltage_p(self, voltage):
        self._voltage_p = max(-1.0, min(1.0, voltage))
    
    def get_voltage_p(self):
        return self._voltage_p

    voltage_p = property(get_voltage_p, set_voltage_p)


if __name__ == '__main__':
    from motors import *

    def init(state):
        state.voltage_p = 1.0
        
    def periodic(state):
        theta = state.theta_rad
        err = math.radians(45) - theta
        #p = 2 / math.radians(45)
        #voltage = err * p
        theta_deg = math.degrees(theta)
        if theta_deg > 60:
            voltage = -0.1
        elif theta_deg > 50:
            voltage = 0.0
        elif theta_deg > 35:
            voltage = 0.2
        else:
            voltage = 0.4
        state.voltage_p = voltage

    ts, a_s, vs, thetas, voltages = ArmSimulation(
        dt_s=0.0001,
        starting_position_rad=math.radians(0),
        end_mass_kg=lbs_to_kg(5.0),
        arm_mass_kg=lbs_to_kg(2.0),
        arm_length_m=inch_to_meter(15),
        nominal_voltage=12.,
        periodic = periodic,
        init = init,
        motor_system=MotorSystem(motor=am9015, motor_count=1, gearing_ratio=64),
    ).run_sim()