import math
import numpy
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


if __name__ == '__main__':
    sim = ShooterSimulation()

    sim.gen_min_max_velocities(math.radians(45), math.radians(85))
