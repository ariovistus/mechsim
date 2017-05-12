import math
from motors import *
from utils import *


touchpad_height_m = inch_to_meter(4*12 + 10)


def run_sim(
        height_m, mass_kg, winch_radius_m, rope_radius_m, 
        motor_system, reporter=None):

    print ("%s%s, gearing ratio=%s, %s inches from ground, %s lbs." % (
        motor_system.name, 
        "" if motor_system.motor_count == 1 else " (" + str(motor_system.motor_count) + "x)",
        motor_system.gearing_ratio,
        meter_to_inch(height_m),
        kg_to_lbs(mass_kg),
    ))

    def radius_at_torque(winch_torque_nm):
        return winch_torque_nm / mass_kg / g

    winch_torque_at_40 = motor_system.torque_at_motor_current(40.)
    print (" max torque at 40A: %.2f in-lbs" % Nm_to_in_lbs(winch_torque_at_40))
    print ("  radius = %6.2f in" % (meter_to_inch(radius_at_torque( winch_torque_at_40))))
    winch_stall_torque = motor_system.stall_torque 
    print (" stall torque: %.2f in-lbs" % (Nm_to_in_lbs(winch_stall_torque)))
    print ("  radius = %6.2f in" % (meter_to_inch(radius_at_torque(winch_stall_torque))))
    travel_distance = touchpad_height_m - height_m 

    freezp = rpm_to_radps(motor_system.free_speed)
    xx = travel_distance / (winch_radius_m * freezp * (1 - (winch_radius_m * mass_kg * g) / winch_stall_torque))
    print ("r=%6.4f freesp=%6.2f F=%6.2f L=%6.2f stallt=%6.2f" % (
        winch_radius_m, freezp, mass_kg * g, travel_distance, winch_stall_torque))
    print ("xx: ", xx)
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
        motor_torque = motor_system._t_sys_to_motor(torque)
        if torque > motor_system.stall_torque:
            print ("insufficent torque to climb at radius %4.2f in!" % (meter_to_inch(radius)))
            overtorque = True
            break
        if torque > winch_torque_at_40:
            time_fuse_burning += dt
        if time_fuse_burning >= time_til_fuse_burnt:
            print ("ya blew the 40A breaker!")
            overtorque = True
            break
        velocity_rpm = motor_system.speed_at_torque(torque) 
        #print ("v=%6.2f rpm" % (velocity_rpm,))
        velocity_mps = rpm_to_mps(velocity_rpm, radius)
        mod_position_m += velocity_mps * dt

        if mod_position_m >= circum(radius):
            mod_position_m -= circum(radius)
            abs_position_m += circum(radius)
            radius += 2 * rope_radius_m

        if abs_position_m + mod_position_m >= travel_distance:
            break

        if reporter is not None:
            reporter(
                time=time, 
                radius=radius, 
                winch_torque=torque,
                motor_torque=motor_torque,
                motor_current=motor_system.motor_current_at_torque(torque),
                motor_stall_torque=motor_system.motor.stall_torque,
                motor_max_torque=motor_system.motor.stall_at_40(),
                time_fuse_burning=time_fuse_burning,
                velocity_rpm=velocity_rpm,
                position=abs_position_m+mod_position_m
                )
        """
        stall40 = motor.stall_at_40() * gearing_ratio
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
        print (" rope eaten: %6.2f in" % meter_to_inch(abs_position_m + mod_position_m))


# this is pretty close to post dcmp onyx
if __name__ == '__main__':
    run_sim(
            height_m=inch_to_meter(12), 
            mass_kg=lbs_to_kg(130), 
            winch_radius_m=inch_to_meter(0.75), 
            rope_radius_m=inch_to_meter(0.00), 
            motor_system=MotorSystem(motor=minicim, motor_count=1, gearing_ratio=64))

