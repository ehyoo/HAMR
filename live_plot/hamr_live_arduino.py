import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import sys
import pdb
from time import sleep
import time

import serial
import os
from time import gmtime, strftime
import threading

# Important! Ensure these variables are correct when running
port = 'COM4'
baudrate = 115200

# Open new file in the given directory. Create new directory if it does not exist
def open_file(name, dir):
    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    filename = '_'.join((name, strftime("[%Y-%m-%d]_[%H-%M-%S]") + '.txt'))
    folder_path = os.path.join(script_dir, dir)
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    file_path = os.path.join(folder_path, filename)
    new_file = open(file_path, 'w')
    return new_file

# setup log and serial port
log = open_file('hamr', 'test_data')
device = serial.Serial(port=port, baudrate=baudrate, timeout=1)

start_time = time.time()

read_finished = False
plotting = True
time_ptr = 0

def query_user():
    while(1):
        data = input("enter velocity value: ")
        device.write(str(data))




def read_loop():
    global read_finishedA
    global xdata
    global ydata
    global time_ptr

    while plotting: # time.time() - start_time < 4
        line = device.readline()   # read a '\n' terminated line
        # print line
        if line == 'start\r\n': # read the beginning of transmission
            data = [0,0,0,0]
            for j in range(4): # read 5 data values and one time value in millis
                data[j] = float(device.readline()) #convert to a 32 bit integer
            print data
            xdata[time_ptr] = data[3] / 1000.0
            # print xdata[time_ptr]
            ydata[time_ptr] = data[0]
            log.write(str(data[3]) + ', ' + str(data_sizeta[0]) + "\n")
            time_ptr = 0 if time_ptr == 999 else time_ptr + 1

    read_finished = True

# initialize plots
numplots = 4
fig = plt.figure()
axes = [plt.subplot(201 + 10 * np.ceil(numplots/2.0) + x) for x in range(0, numplots)]

# data sets
data_size = 1000 # the max number of points to plot at one time
xdata = np.linspace(0, 9.99, num = data_size)
ydata = np.zeros(data_size) - 1

# set axis limits
lines = [axis.plot(ydata, '.')[0] for axis in axes]
for axis in axes:
    axis.set_ylim(-2,2)

xmin = 0 

#update rate
update_rate = 30 # interval in ms betweeen updates. this is not that accurate so don't use it for precise timing
increment = update_rate / 1000.0 # won't be needed for live update

# initial state for blitting
def init():
    for j in range(0,numplots):
        lines[j].set_ydata([])
        lines[j].set_xdata([])
        # axes[j].set_xlim(xmin, xmin + 10)

    return tuple([line for line in lines])

def update(data):
    global xdata 
    global ydata
    global time_ptr

    # these should be the only lines needed for live update
    lines[0].set_xdata(xdata)
    lines[0].set_ydata(ydata)
    axes[0].set_xlim(0,10)
    xmax = max(xdata)
    axes[0].set_xlim(xmax-10, xmax) # plot last ten seconds
    return tuple([line for line in lines])


# start thread
t1 = threading.Thread(target=read_loop) # to pass in parameters, need to subclass
t1.start()

# t2 = threading.Thread(target=query_user) # to pass in parameters, need to subclass
# t2.start()

# redraw callback
def onresize(event):
    plt.draw()

fig.canvas.mpl_connect('button_press_event', onresize)
ani = animation.FuncAnimation(fig, update, init_func = init, interval=update_rate, blit=True)
plt.show()

plotting = False

# cleanup
# don't close IO devices until serial thread has stopped
while not read_finished:
   pass

device.close()
log.close()

print 'Exit successful'
t1.join()
t2.join()
