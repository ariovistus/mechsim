import math 
import wpilib
import numpy
import hal
from hal_impl.sim_hooks import SimHooks as BaseSimHooks
import hal_impl.functions
from frc3223_azurite.conversions import (
    g, 
    lbs_to_kg, 
    inch_to_meter,
    radps_to_rpm,
)


class CatapultSimulation:
    def __init__(self, periodic, motor_system, **kwargs):
        self.dt_s = kwargs.pop('dt_s', 0.0001)
        self.nominal_voltage = kwargs.pop('nominal_voltage', 12.)
        self.starting_position_rad = kwargs.pop('starting_position_rad', 0.)
        self.release_position_rad = kwargs.pop('release_position_rad', math.radians(45))
        self.end_mass_kg = kwargs.pop('end_mass_kg', lbs_to_kg(5.0))
        self.arm_mass_kg = kwargs.pop('arm_mass_kg', lbs_to_kg(2.0))
        self.arm_length_m = kwargs.pop('arm_length_m', inch_to_meter(12.))
        self.damping = kwargs.pop('damping', 0.1) # kg/s
        self.pid_sample_rate= kwargs.pop('pid_sample_rate_s', 0.05)
        self.periodic = periodic
        self.pid_periodic = kwargs.pop('pid_periodic', lambda state: 1)
        self.periodic_period = 0.02 # called on 20 ms intervals
        self.motor_system = motor_system
        self.init = kwargs.pop('init', lambda *args, **kwargs: None)
        assert len(kwargs) == 0, 'Unknown parameters: ' + ', '.join(kwargs.keys())

        self.ts = None
        self.a_s = None
        self.vs = None
        self.thetas = None
        self.voltages = None
        self.currents = None

    def calc_acceleration(self, state):
        torque = self.motor_system.torque_at_speed_and_voltage(state.velocity_radps, state.voltage_p * self.nominal_voltage)
        state.motor_current = self.motor_system.motor_current_at_torque(torque)
        gravity_torque = (
            self.end_mass_kg * g * self.arm_length_m * math.cos(state.theta_rad) +
            self.arm_mass_kg * g * self.arm_length_m / 2. * math.cos(state.theta_rad)
        )

        J_arm = self.arm_mass_kg * self.arm_length_m ** 2 / 3.
        J_end = self.end_mass_kg * self.arm_length_m ** 2
        moment_of_inertia = J_arm + J_end
        damping_torque = self.damping * self.rot_v
        return (torque - gravity_torque - damping_torque) / moment_of_inertia

    def init_log_buffers(self, max_size):
        def make_buffer():
            return numpy.empty(shape=(max_size,),dtype='float')
        self.ts = make_buffer()
        self.a_s = make_buffer()
        self.vs = make_buffer()
        self.thetas = make_buffer()
        self.voltages = make_buffer()
        self.currents = make_buffer()

    def log(self, state):
        self.ts[self.i] = state.time_from_start_s
        self.a_s[self.i] = state.acceleration_radps2
        self.vs[self.i] = self.rot_v
        self.thetas[self.i] = state.theta_rad
        self.voltages[self.i] = state.voltage_p
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
            self.rot_v = 0. # rotational velocity (rad/s)
            theta = self.starting_position_rad # position (rad)
            self.init_log_buffers(int(timeout / sample_rate) + 1)
            self.i = 0
            state._update(t, 0.0, self.rot_v, theta)
            self.init(state)
            a = self.calc_acceleration(state) # acceleration (rad/s^2)
            state._update(t, a, self.rot_v, theta)
            self.log(state)

            while t < timeout and theta < self.release_position_rad and not state.stop:
                self.rot_v += a * dt
                theta += self.rot_v * dt
                t += dt
                a = self.calc_acceleration(state)
                state._update(t, a, self.rot_v, theta)
                if t - t_last_periodic > self.periodic_period:
                    t_last_periodic = t
                    self.periodic(state)
                if (t - t_last_measurement) > sample_rate:
                    self.log(state)
                    t_last_measurement = t
                if t - t_last_pid > self.pid_sample_rate:
                    self.pid_periodic(state)
                    t_last_pid = t

            if theta >= self.release_position_rad:
                self.release_velocity = self.rot_v
            self.trim_buffers()

    def trim_buffers(self):
        buffers = ['ts', 'a_s', 'vs', 'thetas', 'voltages', 'currents']
        for buf in buffers:
            arr = getattr(self,buf)
            setattr(self, buf, arr[:self.i])

    def write_csv(self, nom):
        import csv
        with open(nom, 'w') as f:
            writer = csv.writer(f)

            writer.writerow([
                'time', 'accel (rad/s2)','velocity (rad/s)', 
                'position (rad)', 'current (A)',
                'motor percent voltage'])

            for i in range(len(self.ts)):
                writer.writerow([
                    self.ts[i], self.a_s[i], self.vs[i],
                    self.thetas[i], self.currents[i], self.voltages[i],
                    ])

    @property
    def tangential_release_velocity_m(self):
        return self.release_velocity * self.arm_length_m
    
    def gen_trajectory(self, xinit_m=0, yinit_m=0, xmax_m=3.05, exit_velocity_mps=None, theta_rad=None):
        def x_pos_m(t_s, v0_x_mps):
            return xinit_m + t_s * v0_x_mps

        def y_pos_m(t_s, v0_y_mps):
            return yinit_m + v0_y_mps * t_s - 0.5 * g * t_s ** 2

        if theta_rad is None:
            theta_rad = math.radians(90) - self.release_position_rad
        if exit_velocity_mps is None:
            exit_velocity_mps = self.tangential_release_velocity_m
        print ('exit velocity mps: ', exit_velocity_mps)

        t_max = xmax_m / math.cos(theta_rad) / exit_velocity_mps 
        v_x = exit_velocity_mps * math.cos(theta_rad)
        v_y = exit_velocity_mps * math.sin(theta_rad)
        ts = numpy.linspace(0, t_max, 60)
        x_func = numpy.vectorize(lambda t: x_pos_m(t, v_x))
        y_func = numpy.vectorize(lambda t: y_pos_m(t, v_y))

        return ts, x_func(ts), y_func(ts)


class RobotState:
    def __init__(self):
        self.pid = None
        self.motor_current = 0.0
        self._update(0.0, 0.0, 0.0, 0.0)
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

    def _update(self, t, a, v, theta):
        self.theta_rad = theta
        self.acceleration_radps2 = a
        self.velocity_radps = v
        self.time_from_start_s = t
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

    ArmSimulation(
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
