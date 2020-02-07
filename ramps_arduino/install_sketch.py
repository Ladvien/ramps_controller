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


if __name__ == '__main__':

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

os.system('arduino-cli compile -b arduino:avr:mega ramps_sketch')
command_str = f'arduino-cli -v upload -p {write_port} --fqbn arduino:avr:mega ramps_sketch'
print(command_str)
os.system(command_str)