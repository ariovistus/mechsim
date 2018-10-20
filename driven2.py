import numpy
from frc3223_azurite.conversions import *
from frc3223_azurite import motors 
from frc3223_azurite.inertials import solid_cylinder

class DriveSim:
    def __init__(self, gearbox, **kwargs):
        self.dt_s = kwargs.pop('dt_s', 0.0001)
        self.battery_resistance_ohms = kwargs.pop('battery_resistance_ohms', 0.015)
        self.fuse_resistance_ohms = kwargs.pop('fuse_resistance_ohms', 0.002)
        self.battery_voltage = kwargs.pop('battery_voltage', 12.7)
        self.periodic = kwargs.pop('periodic', lambda *args, **kwargs: None)
        self.periodic_period = kwargs.pop('periodic_period', 0.01) # default called on 10 ms intervals
        self.gearbox_count = kwargs.pop('gearbox_count', 2)
        self.ks = kwargs.pop('ks', 0) # V
        self.kv = kwargs.pop('kv', 0) # V*s/rad
        self.ka = kwargs.pop('ka', 0) # V*s^2/rad
        self.init = kwargs.pop('init', lambda *args, **kwargs: None)
        assert len(kwargs) == 0, 'Unknown parameters: ' + ', '.join(kwargs.keys())
        self.gearbox = gearbox

        self.ts = None
        self.arots = None
        self.vrots = None
        self.goal_vrots = None
        self.thetas = None
        self.voltages = None
        self.bus_voltages = None
        self.currents = None
        self.pushforces = None
        self.slips = None

    def init_log_buffers(self, max_size):
        def make_buffer():
            return numpy.empty(shape=(max_size,),dtype='float')
        self.ts = make_buffer()
        self.arots = make_buffer()
        self.vrots = make_buffer()
        self.goal_vrots = make_buffer()
        self.thetas = make_buffer()
        self.voltages = make_buffer()
        self.bus_voltages = make_buffer()
        self.voltage_ps = make_buffer()
        self.currents = make_buffer()

    def log(self, state):
        self.ts[self.i] = state.time_from_start_s
        self.arots[self.i] = state.acceleration_radps2
        self.vrots[self.i] = state.velocity_radps
        self.goal_vrots[self.i] = state.goal_velocity_radps
        self.thetas[self.i] = state.position_rad
        self.voltages[self.i] = state.voltage
        self.bus_voltages[self.i] = state.available_voltage
        self.voltage_ps[self.i] = state._voltage_p
        self.currents[self.i] = state.motor_current
        self.i += 1

    def calculate_acceleration(self, v, voltage):
        if abs(voltage) < 0.1: voltage = 0
        ks = self.ks * numpy.sign(voltage)
        return (voltage - ks - self.kv * v) / self.ka

    def run_sim(self, timeout_s=3, sample_rate=None):
        if sample_rate is None:
            sample_rate = self.dt_s
        t_last_periodic = 0. # time of last periodic call (s)
        t = 0.0 # sec
        vrot = 0.0 # rad/s
        arot = 0.0 # rad/s^2
        x = 0.0 # rad
        self.state = state = RobotState()
        voltage = self.voltage()
        current = 0.0
        
        current = self.current_at_motor(vrot)
        voltage = self.voltage_at_motor(current)
        av_voltage = self.available_voltage(current)

        self.i = 0
        state._update(t, arot, x, vrot, voltage, av_voltage, current)
        self.init_log_buffers(int(timeout_s / sample_rate) + 1)
        self.log(state)
        self.init(state)


        while self.i < len(self.ts):
            #self.print_stuff()
            arot = self.calculate_acceleration(vrot, voltage)

            vrot = vrot + arot * self.dt_s
            x = x + vrot * self.dt_s
            t += self.dt_s
            state._update(t, arot, vrot, x, voltage, av_voltage, current)
            self.log(state)
            current = self.current_at_motor(vrot)
            if t - t_last_periodic > self.periodic_period:
                t_last_periodic = t
                self.periodic(state)
            voltage = self.voltage_at_motor(current)
            av_voltage = self.available_voltage(current)

    def print_stuff(self):
        print("t=%f, a=%f, v=%f, vg=%f, vmax=%f, V=%f, Vp=%f, i=%f" % (
            self.state.time_from_start_s, 
            self.state.acceleration_radps2,
            self.state.velocity_radps, 
            self.state.goal_velocity_radps, 
            self.gearbox.free_speed,
            self.state.voltage, 
            self.state._voltage_p, 
            self.state.motor_current))

    def current_at_motor(self, vrot):
        backemf = self.gearbox.motor_back_emf(vrot / self.gearbox.gearing_ratio)
        #print ('vv: %f, backemf: %f' % (self.voltage(), backemf))
        voltage = self.voltage() - backemf
        motor_count = self.gearbox_count * self.gearbox.motor_count
        r1 = self.battery_resistance_ohms
        r2 = self.fuse_resistance_ohms
        rm = self.gearbox.motor.resistance()
        r = rm * motor_count + r2 + r1

        return voltage / r

    def voltage_at_motor(self, motor_current):
        r1 = self.battery_resistance_ohms
        r2 = self.fuse_resistance_ohms
        vb = self.voltage()
        motor_count = self.gearbox_count * self.gearbox.motor_count

        return vb - r1 * motor_count * motor_current - r2 * motor_current

    def available_voltage(self, motor_current):
        r1 = self.battery_resistance_ohms
        r2 = self.fuse_resistance_ohms
        vb = self.battery_voltage
        motor_count = self.gearbox_count * self.gearbox.motor_count

        return vb - r1 * motor_count * motor_current - r2 * motor_current

    def voltage(self):
        p = min(1.0, max(-1.0, self.state._voltage_p))
        return self.battery_voltage * p



class RobotState:
    def __init__(self):
        self.pid = None
        self.motor_current = 0.0
        self._update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._voltage_p = 0.0
        self.goal_velocity_radps = 0
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

    def _update(self, t, arot, vrot, x, motor_voltage, av_voltage, current):
        self.acceleration_radps2 = arot
        self.velocity_radps = vrot
        self.position_rad = x
        self.time_from_start_s = t
        self.motor_current = current
        self.voltage = motor_voltage
        self.available_voltage = av_voltage
        if self.pid:
            self.pid.setpointTimer.time_seconds = t
        

if __name__ == '__main__':
    import pytest
    sim = DriveSim(
            ks=5,
            kv=2,
            ka=0.3,
            gearbox=motors.MotorSystem(motor=motors.cim, motor_count=1, gearing_ratio=10))

    assert sim.calculate_acceleration(v=0, voltage=0) == 0
    assert sim.calculate_acceleration(v=100, voltage=0) == pytest.approx(-666.66, 0.1)
    assert sim.calculate_acceleration(v=-100, voltage=0) == pytest.approx(666.66, 0.1)

