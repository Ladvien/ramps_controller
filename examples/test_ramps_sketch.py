
#!/usr/bin/python3

"""
Created on Wed Sep 25 05:58:48 2019

This script is meant to test the RAMPs Arduino sketch included in ./ramps_arduino

@author: ladvien
"""

DRIVE_CMD       = 0x01
HALT_CMD        = 0x0F
DIR_CC          = 0x00
DIR_CCW         = 0x01

COMPLETED_CMD   = 0x07
END_TX          = 0x0A
ACKNOWLEDGE     = 0x06
NEG_ACKNOWLEDGE = 0x15

SUCCESS         = 0x06
FAIL            = 0x15
    
MOTOR_X         = 0x01
MOTOR_Y         = 0x02
MOTOR_Z         = 0xFF
MOTOR_E1        = 0x04
MOTOR_E2        = 0x01

import serial
from time import sleep

#################
# Open Serial
#################
ser = serial.Serial('/dev/ttyUSB0', 115200)

def reset_ramps(ser, print_welcome = False):

        # Reset the Arduino Mega.
        ser.setDTR(False)
        sleep(0.4)
        ser.setDTR(True)
        sleep(2)   
        
        # Get welcome message.
        welcome_message = []
        
        while ser.in_waiting > 0:
            welcome_message.append(ser.readline().decode('utf-8') )
        
        print(''.join(welcome_message))

def read_available(ser, as_ascii = True):
    
    # 1. Get all available data.
    # 2. Unless buffer exceeded.
    # 3. Return a list of the data.
    
    incoming_data = []
    incoming_data_size = 0
    
    while ser.in_waiting > 0:
        incoming_data_size += 1

        if as_ascii:
            incoming_data.append(ser.readline().decode('utf-8'))
        else:
            incoming_data += ser.readline()

    print(incoming_data)

###############
# Test #1
###############
reset_ramps(ser)


#                        0               1     2     3        4       5         6
#   MOTOR_PACKET = PACKET_TYPE_CHAR MOTOR_NUM DIR STEPS_1 STEPS_2 MILLI_BETWEEN \n
#   MOTOR_PACKET =    01                01    00    03     E8        05         0A
#   MOTOR_PACKET =    0x 01010003E8050A
test_packet = bytearray([0x01, 0x00, 0xFF, 0xE8, 0x01, 0x01, 0x00, 0xFF, 0xE8, 0x01, 0x01, 0x00, 0xFF, 0xE8, 0x01, 0x01, 0x00, 0xFF, 0xE8, 0x01, 0x01, 0x00, 0xFF, 0xE8, 0x01 ])

ser.write(test_packet)
read_available(ser)

