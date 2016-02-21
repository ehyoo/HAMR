
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
blit = False
if platform.system() is 'Windows':
    serial_first_line = 'st\r\n'
    blit = True


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

    global xdata
    global ydata0
    global ydata1

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

            print len(xdata)
            print len(ydata0)
            print len(ydata1)

            if len(xdata) < 1000:
                xdata = np.append(xdata, data[0] / 1000.0)
                ydata0 = np.append(ydata0, data[1] / 1000.0)
                ydata1 = np.append(ydata1, data[2] / 1000.0)

            else:
                xd = np.copy(xdata)
                xd[0]  = data[0] / 1000.0
                xdata = np.roll(xd, -1)

                yd0 = np.copy(ydata0)
                yd0[0] = data[1] / 1000.0
                ydata0 = np.roll(yd0, -1)

                yd1 = np.copy(ydata1)
                yd1[0] = data[2] / 1000.0
                ydata1 = np.roll(yd1, -1)


            log.write(','.join(map(str, data)) + "\n")

    log.close()
    reading = False

#### update graph even if xvalue is below 10
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



# PLOTTING
# GUI
plt.subplots_adjust(bottom=0.2)
button_connect = Button(plt.axes([0.1, 0.05, 0.15, 0.075]), 'Connect')
button_connect.on_clicked(callback_connect) 

button_log_data = Button(plt.axes([0.3, 0.05, 0.15, 0.075]), 'Log Data')
button_log_data.on_clicked(callback_log_data)

# initialize plots
numplots = 2
fig = plt.figure()
axes = [plt.subplot(201 + 10 * np.ceil(numplots/2.0) + x) for x in range(0, numplots)]

# data sets
data_size = 1000 # the max number of points to plot at one time
xdata = np.linspace(0, 9.99, num = data_size)
ydata0 = np.zeros(data_size) - 1
ydata1 = np.zeros(data_size) - 1

xdata = [0]
ydata0 = [0]
ydata1 = [0]

# set axis limits
lines = [axis.plot(ydata0, '.')[0] for axis in axes]
for axis in axes:
    axis.set_ylim(-2.0,2.0)

xmin = 0 
update_rate = 30




# initial state for blitting
def init():
    for j in range(0,numplots):
        lines[j].set_ydata([])
        lines[j].set_xdata([])
        # axes[j].set_xlim(xmin, xmin + 10)
    return tuple([line for line in lines])

def update(data):
    global xdata 
    global ydata0
    global ydata1

    # these should be the only lines needed for live update
    lines[0].set_xdata(xdata)
    lines[1].set_xdata(xdata)
    lines[0].set_ydata(ydata0)
    lines[1].set_ydata(ydata1)

    axes[0].set_xlim(0,10)
    axes[1].set_xlim(0,10)
    xmax = max(xdata)
    axes[0].set_xlim(xmax-10, xmax) # plot last ten seconds
    axes[1].set_xlim(xmax-10, xmax) # plot last ten seconds
    return tuple([line for line in lines])


def main():
    global device
    global reading  # while this is true, thread_read is still alive
    global continue_reading  # If this is true, then the thread_read thread will not exit

    reading = False 
    device = serial_initialize(port, baudrate, timeout_=2, write_timeout_=2)

    # ani = animation.FuncAnimation(fig, update, init_func = init, interval=update_rate, blit=blit)
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