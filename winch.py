import math

class MotorParams:
    def __init__(self, name, stall_torque, stall_current, free_speed):
        self.name = name
        self.stall_torque = stall_torque # Nm
        self.stall_current = stall_current # A
        self.free_speed = free_speed # RPM

    def stall_at_40(self):
        if self.stall_current < 40.:
            return self.stall_torque

        k = self.stall_torque / self.stall_current
        torque_at_40 = k * 40.
        return torque_at_40

    def torque_at_speed(self, invel_rpm):
        out_torque = self.stall_torque 
        out_torque *= (self.free_speed - invel_rpm) / self.free_speed
        return out_torque 

    def speed_at_torque(self, intorque_Nm):
        out_vel_rpm = (1 - intorque_Nm / self.stall_torque) * self.free_speed
        return out_vel_rpm 

cim = MotorParams("cim", 2.41, 133., 5300.)
minicim = MotorParams("minicim", 1.4, 86., 6200.)
bag = MotorParams("bag", 0.4, 41., 14000.)
_775pro = MotorParams("775pro", 0.71, 134., 18730.)
rs775 = MotorParams("rs775", 0.247, 22., 5700.)

def Nm_to_in_lbs(nm):
    return  nm * 8.85

def inch_to_meter(inches):
    return inches * 0.0254

def meter_to_inch(meters):
    return meters / 0.0254

def lbs_to_kg(lbs):
    return lbs * 0.4536

def kg_to_lbs(kgs):
    return kgs / 0.4536

def circum(radius):
    return 2 * math.pi * radius

def rpm_to_mps(rpm, radius):
    return rpm * circum(radius) / 60.

touchpad_height_m = inch_to_meter(4*12 + 10)
g = 9.81 # m/s2

def run_sim(
        height_m, mass_kg, winch_radius_m, rope_radius_m, 
        motor, motor_count, gearing_ratio):

    print ("%s%s, gearing ratio=%s, %s inches from ground, %s lbs." % (
        motor.name, 
        "" if motor_count == 1 else " (" + str(motor_count) + "x)",
        gearing_ratio,
        meter_to_inch(height_m),
        kg_to_lbs(mass_kg),
    ))
    print (" stall torque: %.2f in-lbs" % (Nm_to_in_lbs(motor.stall_at_40()) * gearing_ratio * motor_count))
    travel_distance = touchpad_height_m - height_m 
    travel_distance += inch_to_meter(4) # account for stretchy rope 

    dt = 0.1
    time = 0
    time_fuse_burning = 0
    time_til_fuse_burnt = 1
    radius = winch_radius_m + rope_radius_m
    velocity = 0
    abs_position_m = 0
    mod_position_m = 0
    torque = 0
    overtorque = False
    print ("initial radius=%6.2f in" % meter_to_inch(radius))
    while True:
       torque = mass_kg * g * radius
       #print ('\t required torque: %6.2f Nm (%6.2f in-lbs)' % (torque, Nm_to_in_lbs(torque)))
       motor_torque = torque / motor_count / gearing_ratio
       if motor_torque > motor.stall_torque:
           print ("insufficent torque to climb at radius %4.2f in!" % (meter_to_inch(radius)))
           overtorque = True
           break
       if motor_torque > motor.stall_at_40():
            time_fuse_burning += dt
       if time_fuse_burning >= time_til_fuse_burnt:
            print ("ya blew the 40A breaker!")
            overtorque = True
            break
       velocity_rpm = motor.speed_at_torque(motor_torque) / gearing_ratio
       #print ("v=%6.2f rpm" % (velocity_rpm,))
       velocity_mps = rpm_to_mps(velocity_rpm, radius)
       mod_position_m += velocity_mps * dt

       if mod_position_m >= circum(radius):
           mod_position_m -= circum(radius)
           abs_position_m += circum(radius)
           radius += 2 * rope_radius_m

       if abs_position_m + mod_position_m >= travel_distance:
            break

       stall40 = motor.stall_at_40() * gearing_ratio
       """
       print ('\tstall at 40: %6.2f Nm (%6.2f in-lbs)' % (
           stall40,
           Nm_to_in_lbs(stall40),
           ))
        """
       #print ("\tradius=%6.2f   velocity=%6.2f RPM  " % (meter_to_inch(radius), velocity_rpm))
       time += dt
       #break


    if not overtorque:
        print ("time to climb: %6.2f seconds" % time)
        print (" final radius: %6.2f in" % meter_to_inch(radius))


run_sim(
        height_m=inch_to_meter(10), 
        mass_kg=lbs_to_kg(130), 
        winch_radius_m=inch_to_meter(1), 
        rope_radius_m=inch_to_meter(0.25), 
        motor=minicim, 
        motor_count=1,
        gearing_ratio=64)

run_sim(
        height_m=inch_to_meter(10), 
        mass_kg=lbs_to_kg(130), 
        winch_radius_m=inch_to_meter(1), 
        rope_radius_m=inch_to_meter(0.125), 
        motor=minicim, 
        motor_count=3,
        gearing_ratio=12)

