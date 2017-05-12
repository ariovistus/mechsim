import math

class MotorParams:
    def __init__(self, name, stall_torque, stall_current, free_speed):
        self.name = name
        self.stall_torque = stall_torque # Nm
        self.stall_current = stall_current # A
        self.free_speed = free_speed # RPM

    def torque_at_current(self, current_a):
        k = self.ktorque()
        return current_a * k

    def current_at_torque(self, torque_nm):
        k = self.ktorque()
        return torque_nm / k

    def stall_at_40(self):
        if self.stall_current < 40.:
            return self.stall_torque

        return self.torque_at_current(40.)

    def ktorque(self):
        return self.stall_torque / self.stall_current

    def torque_at_speed(self, invel_rpm):
        out_torque = self.stall_torque 
        out_torque *= (self.free_speed - invel_rpm) / self.free_speed
        return out_torque 

    def speed_at_torque(self, intorque_Nm):
        out_vel_rpm = (1 - intorque_Nm / self.stall_torque) * self.free_speed
        return out_vel_rpm 


class MotorSystem:
    def __init__(self, motor, motor_count, gearing_ratio):
        self.motor = motor
        self.motor_count = motor_count
        self.gearing_ratio = gearing_ratio

    def _t_motor_to_sys(self, motor_torque_nm):
        return motor_torque_nm * self.motor_count * self.gearing_ratio

    def _t_sys_to_motor(self, torque_nm):
        return torque_nm / self.motor_count / self.gearing_ratio

    def _v_motor_to_sys(self, motor_vel_rpm):
        return motor_vel_rpm / self.gearing_ratio

    @property
    def name(self):
        return self.motor.name

    @property 
    def stall_torque(self):
        return self._t_motor_to_sys(self.motor.stall_torque)

    @property
    def free_speed(self):
        return self._v_motor_to_sys(self.motor.free_speed)

    def torque_at_motor_current(self, motor_current_a):
        k = self.motor.ktorque()
        motor_torque = motor_current_a * k
        return motor_torque * self.motor_count * self.gearing_ratio

    def motor_current_at_torque(self, torque_nm):
        motor_torque_nm = torque_nm / self.motor_count / self.gearing_ratio
        k = self.motor.ktorque()
        return motor_torque_nm / k

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
am9015 = MotorParams("am9015", 0.428, 63.8, 16000.)
