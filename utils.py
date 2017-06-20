import math 

g = 9.81 # m/s2


def lbs_to_N(lbs):
    return lbs * 4.448


def N_to_lbs(n):
    return n / 4.448


def Nm_to_in_lbs(nm):
    return  nm * 8.85


def in_lbs_to_Nm(inlbs):
    return  inlbs / 8.85


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


def rpm_to_radps(rpm):
    return rpm * 2 * math.pi / 60.

def radps_to_rpm(rpm):
    return rpm / (2 * math.pi) * 60.
