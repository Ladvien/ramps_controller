#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 28 05:39:18 2019

@author: ladvien
"""
from time import sleep, time


"""
  MOTOR_NUM:
      X     = 0
      Y     = 1
      Z     = 2
      E1    = 3
      E2    = 4
      
  PACKET_TYPES
      0x01 = motor_write
      0x02 = motor_halt

  DIRECTION
      0x00 = CW
      0x01 = CCW

  MOTOR MOVE PROTOCOL:
                       0               1     2     3        4       5         6
  MOTOR_PACKET = PACKET_TYPE_CHAR MOTOR_NUM DIR STEPS_1 STEPS_2 MILLI_BETWEEN \n

"""

class RAMPS:
    
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
    MOTOR_Z         = 0x03
    MOTOR_E1        = 0x04
    MOTOR_E2        = 0x05
    
    def __init__(self, ser, debug = False):
        self.ser = ser
        self.toggle_debug = debug
        self.rx_buffer_size = 256
        self.serial_delay = 0.1
        
    def toggle_debug(self):
        self.debug = not self.debug
        
    def print_debug(self, message):
        if self.toggle_debug:
            print(message)
    
    """ 
            COMMUNICATION
    """
    
    # Prepare for a serial send.
    def encode_packet(self, values):
        return bytearray(values)
    
    # Prepare a packet the slave will understand
    def prepare_motor_packet(self, motor_num, direction, steps, milli_between):
        steps_1 = (steps >> 8) & 0xFF
        steps_2 = (steps) & 0xFF
        return [self.DRIVE_CMD, motor_num, direction, steps_1, steps_2, milli_between, self.END_TX]
    
    def read_available(self, as_ascii = False):
        
        self.print_debug(f'Reading available.')
        
        # 1. Get all available data.
        # 2. Unless buffer exceeded.
        # 3. Return a list of the data.
        
        incoming_data = []
        incoming_data_size = 0
        
        while self.ser.in_waiting > 0:
            incoming_data_size += 1
            
            if incoming_data_size > self.rx_buffer_size:
                self.print_debug(f'Buffer overflow.')
                return list('RX buffer overflow.')
            
            if as_ascii:
                incoming_data.append(self.ser.readline().decode('utf-8'))
            else:
                incoming_data += self.ser.readline()

        self.print_debug(f'Completed reading available.')
        return incoming_data


    def check_for_confirm(self, command_expected):
        confirmation = self.read_available()
        if len(confirmation) > 0:
            if confirmation[0] == command_expected:
                return True
        else:
            return False

    
    """ 
            RAMPS UTILITY
    """
    
    def reset_ramps(self, print_welcome = False):

        self.print_debug(f'Reseting Arduino.')
        # Reset the Arduino Mega.
        self.ser.setDTR(False)
        sleep(0.4)
        self.ser.setDTR(True)
        sleep(2)   
        
        # Get welcome message.
        welcome_message = []
        
        while self.ser.in_waiting > 0:
            welcome_message.append(self.ser.readline().decode('utf-8') )
        
        self.print_debug(f'Completed reset.')
        if print_welcome:
            # Print it for the user.
            print(''.join(welcome_message))
            return
        else:
            return

    """ 
            MOTOR COMMANDS
    """
    def move(self, motor, direction, steps, milli_secs_between_steps):
        
        # 1. Create a list containg RAMPs command.
        # 2. Encode it for serial writing.
        # 3. Write to serial port.
        # 4. Check for ACK or NACK.
        # 5. Poll serial for completed command.
        
        packet = self.prepare_motor_packet(motor,
                                           direction,
                                           steps,
                                           milli_secs_between_steps)
        packet = self.encode_packet(packet)
        
        self.print_debug(f'Created move packet: {packet}')
        
        self.write_move(packet)
        
        # Don't miss ACK to being in a hurry.
        sleep(self.serial_delay)
        confirmation = self.read_available()
        if confirmation[0] == self.ACKNOWLEDGE:
            self.print_debug(f'Move command acknowledged.')
        
        if(self.wait_for_complete(120)):
            return True
        
        return False
    
    def wait_for_complete(self, timeout):
        
        # 1. Wait for complete or timeout
        # 2. Return whether the move was successful.
        
        start_time = time()
        
        while True:
            now_time = time()
            duration = now_time - start_time
            self.print_debug(duration)
            if(duration > timeout):
                return False
            
            if self.check_for_confirm(self.COMPLETED_CMD):
                self.print_debug(f'Move command completed.')
                return True
            
            sleep(self.serial_delay)
    
    def write_move(self, packet):        
        self.ser.write(packet)
        self.print_debug(f'Executed move packet: {packet}')