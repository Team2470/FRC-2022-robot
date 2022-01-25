#! /usr/bin/env python

import math

# Link: https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html#record-feedforward-gains

# A good test for this is to calculate the “theoretical” value of kV, which is 12 volts divided by the theoretical
# free speed of your drivetrain (which is, in turn, the free speed of the motor times the wheel circumference divided
# by the gear reduction). This value should agree very closely with the kV measured by the tool - if it does not, you
# have likely made an error somewhere.

wheel_diameter_inches = 6
wheel_diameter_meters = wheel_diameter_inches/39.3701
wheel_circumference_meters = wheel_diameter_meters*math.pi

gear_ratio = 26.04  # Low gear
motor_freespeed_rpm = 6380  # Falcon 500
gearbox_freespeed_rpm = motor_freespeed_rpm/gear_ratio

# Convert revolutions per minute (RPM) to revolutions per second (RPS)
gearbox_freespeed_rps = gearbox_freespeed_rpm/60

# Theoretical drive train freespeed in meters per second 
drivetrain_freespeed_mps = gearbox_freespeed_rps*wheel_circumference_meters

kv = 12 / drivetrain_freespeed_mps

print(f"kv {kv}")

