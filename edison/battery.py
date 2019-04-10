#!/usr/bin/env python3.7

from roboclaw.motorcontrol import MotorControl

# Edison - '/dev/ttyMFD1'
# MacBook - '/dev/tty.usbmodem14101'
# Office PC - '/dev/tty.usbmodem1D15101'

motors = MotorControl('/dev/ttyMFD1', False)

battery = motors.read_batt_voltage()
print('{}V'.format(battery))
