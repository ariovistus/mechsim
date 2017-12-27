import numpy
from frc3223_azurite.conversions import *
from frc3223_azurite import motors 
from frc3223_azurite.inertials import solid_cylinder

class DriveSim:
    def __init__(self, motor_system, **kwargs):
        self.dt_s = kwargs.get('dt_s', 0.0001)
        self.robot_mass_kg = kwargs.get('robot_mass_kg', lbs_to_kg(150))
        self.wheel_radius_m = kwargs.get('wheel_radius_m', inch_to_meter(12.))
        self.wheel_inertial = kwargs.get('wheel_inertial', solid_cylinder(lbs_to_kg(0.3), self.wheel_radius_m))
        self.gearbox_efficiency = kwargs.get('gearbox_efficiency', 0.9)
        self.u_static = kwargs.get('u_static', 0.9)
        self.u_kinetic = kwargs.get('u_kinetic', 0.7)
        self.rolling0 = kwargs.get('rolling0', 0)
        self.rolling1 = kwargs.get('rolling1', 0)
        assert 0 <= self.gearbox_efficiency <= 1
        # assume 1 motor system per wheel
        self.motor_system = motor_system

        self.ts = None
        self.a_s = None
        self.vs = None
        self.arots = None
        self.vrots = None
        self.voltages = None
        self.currents = None
        self.pushforces = None
        self.slips = None

    def init_log_buffers(self, max_size):
        def make_buffer():
            return numpy.empty(shape=(max_size,),dtype='float')
        self.ts = make_buffer()
        self.a_s = make_buffer()
        self.vs = make_buffer()
        self.arots = make_buffer()
        self.vrots = make_buffer()
        self.voltages = make_buffer()
        self.currents = make_buffer()
        self.pushforces = make_buffer()
        self.slips = make_buffer()

    def log(self, state):
        self.ts[self.i] = state.time_from_start_s
        self.a_s[self.i] = state.acceleration_mps2
        self.vs[self.i] = state.velocity_mps
        self.arots[self.i] = state.acceleration_radps2
        self.vrots[self.i] = state.velocity_radps
        self.voltages[self.i] = 1.0 #state.voltage_p
        self.currents[self.i] = state.motor_current
        self.pushforces[self.i] = state.linear_force
        self.slips[self.i] = state.slip
        self.i += 1

    def run_sim(self, timeout_s=3, sample_rate=None):
        if sample_rate is None:
            sample_rate = self.dt_s
        t = 0.0 # sec
        v = 0.0 # m/s
        a = 0.0 # m/s^2
        vrot = 0.0 # rad/s
        arot = 0.0 # rad/s^2
        voltage = 0.0
        linear_force = 0 # N
        current = 0.0
        slip = 0
        wheel_torque = None # Nm
        wheel_torque = self.gearbox_efficiency * self.motor_system.torque_at_speed(vrot)
        wheel_normal_force = self.robot_mass_kg * g / 4

        self.i = 0
        state = RobotState()
        state._update(t, a, arot, v, vrot, voltage, current, linear_force, slip)
        self.init_log_buffers(int(timeout_s / sample_rate) + 1)
        self.log(state)


        while self.i < len(self.ts):
            wheel_on_ground_force = wheel_torque / self.wheel_radius_m
            if wheel_on_ground_force > self.u_static * wheel_normal_force:
                slip = 1
            elif slip and vrot * self.wheel_radius_m < v:
                slip = 0
                
            if slip:
                ground_on_wheel_force = self.u_kinetic * wheel_normal_force
                arot = (wheel_torque - ground_on_wheel_force * self.wheel_radius_m) / self.wheel_inertial
            else:
                # no slip
                ground_on_wheel_force = min(wheel_on_ground_force, self.u_static * wheel_normal_force)
            rolling_resistance = self.rolling0 + self.rolling1 * v

            linear_force = ground_on_wheel_force * 4 - rolling_resistance

            a = linear_force / self.robot_mass_kg

            v = v + a * self.dt_s
            if slip:
                vrot = vrot + arot * self.dt_s
            else:
                vrot = v / self.wheel_radius_m
            t += self.dt_s
            state._update(t, a, arot, v, vrot, voltage, current, linear_force, slip)
            self.log(state)
            wheel_torque = self.gearbox_efficiency * self.motor_system.torque_at_speed(vrot)


class RobotState:
    def __init__(self):
        self.pid = None
        self.motor_current = 0.0
        self._update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
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

    def _update(self, t, a, arot, v, vrot, voltage, current, pushforce, slip):
        self.acceleration_radps2 = arot
        self.acceleration_mps2 = a
        self.velocity_radps = vrot
        self.velocity_mps = v
        self.time_from_start_s = t
        self.linear_force = pushforce
        self.current = current
        self.voltage = voltage
        self.slip = slip
        if self.pid:
            self.pid.setpointTimer.time_seconds = t
        

if __name__ == '__main__':
    sim = DriveSim(motor_system=motors.MotorSystem(motor=motors.cim, motor_count=1, gearing_ratio=10))
    sim.run_sim()

