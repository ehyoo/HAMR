
import time
import os
from time import gmtime, strftime
import threading
import platform
import signal

import serial

import sys
import pdb

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button


# add code to arduino to stop sending serial data if not connected! (change "send" to "stop")
# Important! Ensure these variables are correct when running
port = 'COM6'
baudrate = 250000

serial_first_line = 'st\n'
if platform.system() is 'Windows':
    serial_first_line = 'st\r\n'


# Open new file in the given directory. Create new directory if it does not exist
# save data in CSV format (works automatically in excel)
def open_file(name, dir):
    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    filename = '_'.join((name, strftime("[%Y-%m-%d]_[%H-%M-%S]") + '.csv'))
    folder_path = os.path.join(script_dir, dir)
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    file_path = os.path.join(folder_path, filename)
    new_file = open(file_path, 'w')
    return new_file

# start_time = time.time()

# read_finished = False
# plotting = True
# time_ptr = 0

# def read_loop():
#     global read_finished
#     global xdata
#     global ydata
#     global time_ptr
#     print "read loop\n"

#     while plotting: # time.time() - start_time < 4
#         line = device.readline()   # read a '\n' terminated line
#         print line
#         if line == 'start\r\n': # read the beginning of transmission
#             data = [0,0,0,0]
#             for j in range(4): # read 5 data values and one time value in millis
#                 data[j] = float(device.readline()) #convert to a 32 bit integer
#             # print data
#             # xdata[time_ptr] = data[3] 1000.0
#             # print xdata[time_ptr]
#             # ydata[time_ptr] = data[0]
#             # log.write(str(data[3]) + ', ' + str(data[0]) + "\n")
#             time_ptr = 0 if time_ptr == 999 else time_ptr + 1

#     read_finished = True



# t2 = threading.Thread(target=query_user) # to pass in parameters, need to subclass
# t2.start()


# read the serial port if it is open. should be run in a seperate thread

def print_error(e, method):
    print  "CAUGHT: " + e.__class__.__name__ + "\n" + str(e) + "\n" + "METHOD: " + method


def serial_initialize(port_, baudrate_, timeout_= 2, write_timeout_=2):
    ser = serial.Serial(baudrate=baudrate_, timeout=timeout_, write_timeout=write_timeout_)
    ser.port = port_
    return ser


def serial_connect(serial_device):
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


def serial_disconnect(serial_device):
    if serial_device.is_open:
        serial_device.close()


# def serial_readline(serial_device):
#     if serial_device.is_open:
#         try:
#             return serial_device.readline()
#         except serial.serialutil.SerialException as e:
#             print_error(e, sys._getframe().f_code.co_name)
        

def serial_write(serial_device, val):
    if serial_device.is_open:
        serial_device.write(val)
        return 1


def read_loop():
    global reading
    global continue_reading

    log = open_file('hamr', 'test_data')

    num_data_points = 4
    reading = True

    while device.is_open and continue_reading: # need to check if its open
        try:
            line = device.readline()    # read a \n terminated line
        except Exception as e:
            continue_reading = False
            print_error(e, sys._getframe().f_code.co_name)

        if line == serial_first_line:
            data = [-1 for x in range(0, num_data_points)]
            for j in range(0,num_data_points):
                try:
                    data[j] = float(device.readline())
                except: # TODO: add the other exception when serial port is closed
                    continue
            
            # do something with data
            print data
            # update graph data

            log.write(','.join(map(str, data)) + "\n")

    log.close()
    reading = False


# connect to com port 
def callback_connect(event):
    global continue_reading

    if button_connect.label.get_text() == 'Connect':
        # attempt to connect to the serial port unless it's already open
        if not serial_connect(device): return
        button_connect.label.set_text("Disconnect")
        print "Connected Succesfully"
    else:
        serial_write(device, b'stop\n')
        continue_reading = False  # tell thread_read to exit
        while reading: pass 
        serial_disconnect(device)

        button_connect.label.set_text("Connect")
        button_log_data.label.set_text("Log Data")
        print "Disonnected Succesfully"
    
    plt.draw()


def callback_log_data(event):
    global continue_reading
    global thread_read

    if not device.is_open:
        print "You're not connected\n"
        return

    device.reset_input_buffer()
    if button_log_data.label.get_text() == 'Log Data':
        while device.readline() != 'send\n':
            device.write(b'send\n')
            print "sent start signal\n"
        
        continue_reading = True
        thread_read = threading.Thread(target=read_loop)
        thread_read.start()

        button_log_data.label.set_text('Stop Logging')
    else:
        continue_reading = False
        while(reading): pass

        device.reset_input_buffer()
        while device.readline() != 'stop\n': 
            device.write(b'stop\n')
            print "sent stop signal\n"
        button_log_data.label.set_text('Log Data')
    
    plt.draw()


# GUI
button_connect = Button(plt.axes([0.1, 0.05, 0.15, 0.075]), 'Connect')
button_connect.on_clicked(callback_connect) 

button_log_data = Button(plt.axes([0.3, 0.05, 0.15, 0.075]), 'Log Data')
button_log_data.on_clicked(callback_log_data) 

def main():
    global device
    global reading  # while this is true, thread_read is still alive
    global continue_reading  # If this is true, then the thread_read thread will not exit

    reading = False 
    device = serial_initialize(port, baudrate, timeout_=2, write_timeout_=2)

    plt.show()

    continue_reading = False  # tell thread_read to exit
    while reading: pass  # wait until thread_read has finished
    device.close()
    print 'Exit Succesful'

if __name__ == '__main__': main()

# Debugging notes: 

# DON'T use CTRL + C

# if the serial is printing out spaces every second, 
# then the com port connected but the arduino is choosing not to send 
# any data. This is likely a logic error (i.e. not properly telling the
# arduino to start)