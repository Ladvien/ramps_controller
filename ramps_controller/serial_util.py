from time import sleep
import os
import sys
import glob
import serial

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def get_write_port():

    # Get ports listed.    
    ports = serial_ports()

    # Check if any ports found.
    print('Select port: ')
    if len(ports) < 1:
        print('No connected serial devices found.')
        print('Are you in the dialout group?')
        quit()

    # Allow the user to select port.
    for i in range(len(ports)):
        print(f'{i}: {ports[i]}')
    
    selected_port = -1
    while selected_port < 0:
        selected_port_candidate = input(':')
        try:
            selected_port = int(selected_port_candidate)
        except:
            print('Invalid port selected.')

    write_port = ''
    try:
        write_port = ports[selected_port]
    except:
        print('Unable to parse port name.')

    print(f'Selected port {write_port}')
    return  write_port

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

def encode(value):
    return (value << 2) | 0x03

def decode(value):
    return (value >> 2) &~ 0xC0

