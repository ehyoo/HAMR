import serial
import time

# intialize and return a serial object. This will NOT automatically connect to the given port. Call connect() for that
def initialize(port_, baudrate_, timeout_= 2, write_timeout_=2):
    ser = serial.Serial(baudrate=baudrate_, timeout=timeout_, write_timeout=write_timeout_)
    ser.port = port_
    return ser

# connect to the serial object. The serial object should already be instatiated with either initialize() or the serial.Serial() method

def connect(serial_device):
    try:
        serial_device.open()
    except serial.serialutil.SerialException as e:
        print_error(e, sys._getframe().f_code.co_name)
        return 0

    # read initial line to confirm connection. timeout after 2 second
    start_time = time.time()
    elapsed_time = 0
    while not serial_device.readline() and time < 2: # could i replace the timeout with a lambda expression?
        elapsed_time = time.time() - start_time

    return elapsed_time < 1

# Disconnect the given serial device iff it's open
def disconnect(serial_device):
    if serial_device.is_open:
        serial_device.close()


# def serial_readline(serial_device):
#     if serial_device.is_open:
#         try:
#             return serial_device.readline()
#         except serial.serialutil.SerialException as e:
#             print_error(e, sys._getframe().f_code.co_name)
        

# write to the serial device iff it's open
def write(serial_device, val):
    if serial_device.is_open:
        serial_device.write(val)
        return 1