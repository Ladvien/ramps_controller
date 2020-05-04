#!/usr/bin/python3

"""
Created on Wed Sep 25 05:58:48 2019

@author: ladvien
"""

from ramps_controller.ramps_controller import RAMPS
from ramps_controller.serial_util import *

#################
# Open Serial
#################
port_name = get_write_port()
ser = serial.Serial(port_name, 115200)
ramps = RAMPS(ser)

###############
# Test #1
###############
""""
This tests the serial connection, commands, and reset of the RAMPS module.
"""
ramps.reset_ramps()

#                        0               1     2     3        4       5         6
#   MOTOR_PACKET = PACKET_TYPE_CHAR MOTOR_NUM DIR STEPS_1 STEPS_2 MILLI_BETWEEN \n
#   MOTOR_PACKET =    01                01    00    03     E8        05         0A
#   MOTOR_PACKET =    0x 01010003E8050A
#  
#   PACKAGE      = MOTOR_X + MOTOR_Y + MOTOR_Z + MOTOR_E1 + MOTOR_E2  

motor_packet_x = [ramps.DRIVE_CMD, ramps.DIR_CC, 0xFF, 0xFF, 0x05]

test_package = bytearray(motor_packet_x)
print(test_package)
ser.write(test_package)
read_available(ser)
