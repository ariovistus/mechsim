import numpy
from frc3223_azurite.conversions import *
from frc3223_azurite import motors 
from frc3223_azurite.inertials import solid_cylinder

class DriveSim:
    def __init__(self, gearbox, **kwargs):
        self.dt_s = kwargs.pop('dt_s', 0.0001)
        self.robot_mass_kg = kwargs.pop('robot_mass_kg', lbs_to_kg(150))
        self.wheel_radius_m = kwargs.pop('wheel_radius_m', inch_to_meter(3.))
        self.wheel_inertial = kwargs.pop('wheel_inertial', solid_cylinder(lbs_to_kg(0.3), self.wheel_radius_m))
        self.gearbox_efficiency = kwargs.pop('gearbox_efficiency', 0.65)
        self.battery_resistance_ohms = kwargs.pop('battery_resistance_ohms', 0.015)
        self.fuse_resistance_ohms = kwargs.pop('fuse_resistance_ohms', 0.002)
        self.battery_current_max = kwargs.pop('battery_current_max', 250)
        self.u_static = kwargs.pop('u_static', 0.9)
        self.u_kinetic = kwargs.pop('u_kinetic', 0.7)
        self.rolling0 = kwargs.pop('rolling0', 30)
        self.rolling1 = kwargs.pop('rolling1', 5)
        self.battery_voltage = kwargs.pop('battery_voltage', 12.7)
        self.gearbox_count = kwargs.pop('gearbox_count', 2)
        assert 0 <= self.gearbox_efficiency <= 1
        assert len(kwargs) == 0, 'Unknown parameters: ' + ', '.join(kwargs.keys())
        self.gearbox = gearbox

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
        self.voltages[self.i] = state.voltage
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
        voltage = self.battery_voltage
        linear_force = 0 # N
        current = 0.0
        slip = 0
        gbox_torque = None # Nm
        
        current = self.current_at_motor(vrot)
        gbox_torque = self.gearbox_efficiency * \
            self.gearbox.torque_at_motor_current(current)
        voltage = self.voltage_at_motor(current)
        gbox_normal_force = self.robot_mass_kg * g / self.gearbox_count

        self.i = 0
        state = RobotState()
        state._update(t, a, arot, v, vrot, voltage, current, linear_force, slip)
        self.init_log_buffers(int(timeout_s / sample_rate) + 1)
        self.log(state)


        while self.i < len(self.ts):
            gbox_on_ground_force = gbox_torque / self.wheel_radius_m
            if gbox_on_ground_force > self.u_static * gbox_normal_force:
                slip = 1
            elif slip and vrot * self.wheel_radius_m < v:
                slip = 0
                
            if slip:
                ground_on_gbox_force = self.u_kinetic * gbox_normal_force
                arot = (gbox_torque - ground_on_gbox_force * self.wheel_radius_m) / self.wheel_inertial
            else:
                # no slip
                ground_on_gbox_force = min(gbox_on_ground_force, self.u_static * gbox_normal_force)
            rolling_resistance = self.rolling0 + self.rolling1 * v

            linear_force = max(0, ground_on_gbox_force * self.gearbox_count - rolling_resistance)

            a = linear_force / self.robot_mass_kg

            v = v + a * self.dt_s
            if slip:
                vrot = vrot + arot * self.dt_s
            else:
                vrot = v / self.wheel_radius_m
            t += self.dt_s
            state._update(t, a, arot, v, vrot, voltage, current, linear_force, slip)
            self.log(state)
            current = self.current_at_motor(vrot)
            gbox_torque = self.gearbox_efficiency * \
                self.gearbox.torque_at_motor_current(current)
            voltage = self.voltage_at_motor(current)

    def current_at_motor(self, vrot):
        backemf = self.gearbox.motor_back_emf(vrot)
        voltage = self.battery_voltage - backemf
        motor_count = self.gearbox_count * self.gearbox.motor_count
        r1 = self.battery_resistance_ohms
        r2 = self.fuse_resistance_ohms
        rm = self.gearbox.motor.resistance()
        r = r1 * motor_count + r2 + rm

        return voltage / r

    def voltage_at_motor(self, motor_current):
        r1 = self.battery_resistance_ohms
        r2 = self.fuse_resistance_ohms
        vb = self.battery_voltage
        motor_count = self.gearbox_count * self.gearbox.motor_count

        return vb - r1 * motor_count * motor_current - r2 * motor_current


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
    sim = DriveSim(gearbox=motors.MotorSystem(motor=motors.cim, motor_count=1, gearing_ratio=10))
    sim.run_sim()

