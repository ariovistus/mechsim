import math
import numpy
from motors import *
from utils import *



class ShooterSimulation:
    def __init__(self, **kwargs):
        self.fuel_radius_m = inch_to_meter(2.5)
        self.hole_radius_m = inch_to_meter(10.75)
        self.hole_y_pos_m = inch_to_meter(97)
        self.shooter_y_pos_m = kwargs.get('shooter_y_pos_m', inch_to_meter(15))

        # shooter is assumed to be at position x=0. 
        # this is the distance between the shooter and the near edge of the hole
        self.hole_x_pos_m = kwargs.get('hole_x_pos_m', inch_to_meter(100))

    def x_pos_m(self, t_s, exit_velocity_x_mps):
        return t_s * exit_velocity_x_mps

    def y_pos_m(self, t_s, exit_velocity_y_mps):
        return self.shooter_y_pos_m + exit_velocity_y_mps * t_s - 0.5 * g * t_s ** 2
    
    def velocity_mps(self, theta_rad, x_m):
        h = self.hole_y_pos_m - self.shooter_y_pos_m
        rf = self.fuel_radius_m
        return numpy.sqrt(g/2) * x_m / numpy.cos(theta_rad) / numpy.sqrt(x_m * numpy.tan(theta_rad) - (h + rf))

    def min_velocity_mps(self, theta_rad):
        v = self.velocity_mps(theta_rad, self.hole_x_pos_m)
        t = self.hole_x_pos_m / v / numpy.cos(theta_rad)
        if v * numpy.sin(theta_rad) - g * t >= 0.0:
            # trajectory isn't descending
            v = float('nan')
        return v

    def max_velocity_mps(self, theta_rad):
        v = self.velocity_mps(theta_rad, self.hole_x_pos_m + 2 * self.hole_radius_m)
        t1 = self.hole_x_pos_m / v / numpy.cos(theta_rad)
        t2 = (self.hole_x_pos_m + 2 * self.hole_radius_m) / v / numpy.cos(theta_rad)
        ypos1 = self.y_pos_m(t1, v * math.sin(theta_rad))
        if ypos1 < self.hole_y_pos_m + self.fuel_radius_m:
            # didn't clear boiler
            v = float('nan')
        elif v * numpy.sin(theta_rad) - g * t2 >= 0.0:
            # trajectory isn't descending
            v = float('nan')
        return v

    def min_theta_rad(self):
        return math.atan(((self.hole_y_pos_m - self.shooter_y_pos_m) + self.hole_radius_m) / self.hole_x_pos_m)

    def gen_min_max_velocities(self, min_theta_rad, max_theta_rad):
        if min_theta_rad < self.min_theta_rad():
            print ('min theta = ', self.min_theta_rad(), 'adjusting theta range')
            min_theta_rad = self.min_theta_rad()
        thetas = numpy.linspace(min_theta_rad, max_theta_rad, 60)
        min_func = numpy.vectorize(self.min_velocity_mps) 
        max_func = numpy.vectorize(self.max_velocity_mps) 

        min_velocities = min_func(thetas)
        max_velocities = max_func(thetas)
        return thetas, min_velocities, max_velocities

    def gen_trajectory(self, theta_rad, exit_velocity_mps):
        t_max = (self.hole_x_pos_m + 2 * self.hole_radius_m) / math.cos(theta_rad) / exit_velocity_mps 
        v_x = exit_velocity_mps * math.cos(theta_rad)
        v_y = exit_velocity_mps * math.sin(theta_rad)
        ts = numpy.linspace(0, t_max, 60)
        x_func = numpy.vectorize(lambda t: self.x_pos_m(t, v_x))
        y_func = numpy.vectorize(lambda t: self.y_pos_m(t, v_y))

        return ts, x_func(ts), y_func(ts)


class FlywheelSimulation: 
    def __init__(self, **kwargs):
        self.fuel_radius_m = inch_to_meter(2.5)
        self.flywheel_radius_m = kwargs.get('flywheel_radius_m', inch_to_meter(1.5))
        self.fuel_initial_velocity_mps = kwargs.get('fuel_initial_velocity_mps', 0)
        self.fuel_mass_kg = kwargs.get('fuel_mass_kg', lbs_to_kg(2.6 / 16))
        self.flywheel_initial_velocity_radps = kwargs.get('flywheel_initial_velocity_radps', 0)
        self.flywheel_mass_kg = kwargs.get('flywheel_mass_kg', lbs_to_kg(0.22))
        self.c_flywheel_fuel_static_friction = kwargs.get('flywheel_fuel_static_friction', 0.1)
        self.c_flywheel_fuel_kinetic_friction = kwargs.get('flywheel_fuel_kinetic_friction', 0.075)
        self.c_channel_fuel_static_friction = kwargs.get('channel_fuel_static_friction', 0.7)
        self.c_channel_fuel_kinetic_friction = kwargs.get('channel_fuel_kinetic_friction', 0.053)
        self.compression_force = kwargs.get('compression_force', lbs_to_N(2.4))
        self.entry_position_rad = kwargs.get('entry_position_rad', math.radians(155))
        self.exit_position_rad = kwargs.get('exit_position_rad', math.radians(12))
        self.motor_system = kwargs['motor_system']

    def channel_vdiff(self):
        rf = self.fuel_radius_m
        rw = self.flywheel_radius_m

        b = self.vcf * rf * (rf + rw) / (2 * rf + rw)

        return (self.v - b)

    def is_channel_slipping(self):
        return abs(self.channel_vdiff()) > 0.0005

    def flywheel_vdiff(self):
        rf = self.fuel_radius_m
        rw = self.flywheel_radius_m

        v1 = self.v * rw / (rw + rf)
        vw = rw * self.wv
        vf = v1 - rf * self.vwf
        return vw - vf

    def is_flywheel_slipping(self):
        return abs(self.flywheel_vdiff()) > 0.0005

    def flywheel_tangent_force(self):
        torque_Nm = self.motor_system.torque_at_speed(radps_to_rpm(self.wv))
        f = torque_Nm / self.flywheel_radius_m 
        return f

    def max_flywheel_fuel_static_friction(self):
        return self.c_flywheel_fuel_static_friction * self.compression_force

    def flywheel_fuel_kinetic_friction(self):
        return self.c_flywheel_fuel_kinetic_friction * self.compression_force

    def max_channel_fuel_static_friction(self):
        return self.c_channel_fuel_static_friction * self.compression_force

    def channel_fuel_kinetic_friction(self):
        return self.c_channel_fuel_kinetic_friction * self.compression_force

    def friction_flywheel_on_fuel(self):
        vdiff = self.flywheel_vdiff()

        if not self.is_flywheel_slipping():
            # static friction
            f = self.flywheel_tangent_force()
            f = min(f, self.max_flywheel_fuel_static_friction())
            return f
        elif vdiff < 0:
            # kinetic friction opposing direction of movement
            f = -self.flywheel_fuel_kinetic_friction()
            return f
        elif vdiff > 0:
            # kinetic friction in direction of movement
            f = self.flywheel_fuel_kinetic_friction()
            return f

    def friction_fuel_on_channel(self):
        vdiff = self.channel_vdiff()

        if not self.is_channel_slipping():
            # static friction
            f = self.friction_flywheel_on_fuel() + self.gravity_tangent_component()
            f = min(f, self.max_channel_fuel_static_friction())
            return f
        elif vdiff < 0:
            return -self.channel_fuel_kinetic_friction()
        elif vdiff > 0:
            return self.channel_fuel_kinetic_friction()

    def gravity_tangent_component(self):
        return self.fuel_mass_kg * g * math.cos(self.x - math.pi)

    def flywheel_moment_of_inertia(self):
        return 0.5 * self.flywheel_mass_kg * self.flywheel_radius_m ** 2

    def flywheel_acceleration(self):
        f = self.flywheel_tangent_force() - self.friction_flywheel_on_fuel()
        torque = f * self.flywheel_radius_m
        J = self.flywheel_moment_of_inertia()
        return torque / J

    def fuel_moment_of_inertia_about_flywheel(self):
        mf = self.fuel_mass_kg
        rf = self.fuel_radius_m
        rw = self.flywheel_radius_m
        return (2./3) * mf * rf ** 2 + mf * (rf + rw) ** 2

    def fuel_tangent_acceleration(self):
        rf = self.fuel_radius_m
        rw = self.flywheel_radius_m
        J = self.fuel_moment_of_inertia_about_flywheel()
        forces = self.friction_flywheel_on_fuel() + self.gravity_tangent_component() + self.friction_fuel_on_channel()
        moment = (rf+rw) * forces
        return moment / J

    def fuel_acceleration_pcf(self):
        """
        rotational acceleration about point of contact with channel
        """
        rf = self.fuel_radius_m
        moment = -2 * rf * self.friction_flywheel_on_fuel() - rf * self.gravity_tangent_component()
        J = (5./3) * self.fuel_mass_kg * rf ** 2
        return moment / J

    def fuel_acceleration_pwf(self):
        """
        rotational acceleration about point of contact with flywheel
        """
        rf = self.fuel_radius_m
        moment = 2 * rf * self.friction_fuel_on_channel() + rf * self.gravity_tangent_component()
        J = (5./3) * self.fuel_mass_kg * rf ** 2
        return moment / J

    def run_simulation(self, dt=0.0001, timeout=3):
        t = 0.0
        dpos = self.exit_position_rad - self.entry_position_rad
        if dpos < 0:
            dpos += math.pi * 2
        self.vcf = 0.0 # fuel rotational velocity about point of contact with channel rad/s
        self.vwf = 0.0 # fuel rotational velocity about point of contact with flywheel rad/s^2
        self.v = self.fuel_initial_velocity_mps # fuel translational velocity along arc path m/s
        self.x = self.entry_position_rad # fuel position rad
        dx = 0 # change in fuel position since t=0, rad
        self.wv = self.flywheel_initial_velocity_radps # flywheel velocity rad/s
        self.a = self.fuel_tangent_acceleration() # fuel translational acceleration along arc path m/s^2
        self.wa = self.flywheel_acceleration() # flywheel acceleration rad/s^2
        self.acf = self.fuel_acceleration_pcf() # fuel rotational acceleration about point of contact with channel rad/s^2
        self.awf = self.fuel_acceleration_pwf() # fuel rotational acceleration about point of contact with flywheel rad/s^2
        channel_slip = self.is_channel_slipping()
        flywheel_slip = self.is_flywheel_slipping()
        i = 0
        n = (int(timeout/dt) + 1)
        def buf():
            b = numpy.zeros(shape=(n,))
            b.fill(numpy.nan)
            return b
        self.ts = buf()
        self.vs = buf()
        self.a_s = buf()
        self.xs = buf()
        self.wvs = buf()
        self.was = buf()

        def log():
            nonlocal i, t
            self.ts[i] = t
            self.vs[i] = self.v
            self.a_s[i] = self.a
            self.xs[i] = self.x
            self.wvs[i] = self.wv
            self.was[i] = self.wa
            i += 1

        log()

        while t < timeout:
            self.wv += self.wa * dt
            self.v += self.a * dt
            self.x += self.v * dt
            dx += self.v * dt
            self.vcf += self.acf * dt
            self.a = self.fuel_tangent_acceleration()
            self.wa = self.flywheel_acceleration()
            self.acf = self.fuel_acceleration_pcf()
            self.awf = self.fuel_acceleration_pwf() 
            t += dt
            log()

            if dx >= dpos:
                break


if __name__ == '__main__':
    #sim = ShooterSimulation()

    #sim.gen_min_max_velocities(math.radians(45), math.radians(85))

    motor_system = MotorSystem(motor=cim, motor_count=1, gearing_ratio=1)
    sim = FlywheelSimulation(
            flywheel_initial_velocity_radps=rpm_to_radps(motor_system.free_speed),
            #flywheel_initial_velocity_radps=546.485,
            motor_system=motor_system)
    sim.run_simulation()
