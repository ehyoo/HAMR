import time
import threading
import platform
import signal
import timeit

import sys
import pdb

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from matplotlib.widgets import Slider
from matplotlib.widgets import CheckButtons



import hamr_file_writer
import hamr_serial

# Debugging notes: 
# Avoid using CTRL + C

# if the serial is printing out spaces every second, 
# then the com port connected but the arduino is choosing not to send 
# any data. This is likely a logic error (i.e. not properly telling the
# arduino to start)

# Important! Ensure these variables are correct when running
port = 'COM6'
baudrate = 250000

serial_first_line = 'xx\n'
blit = False
if platform.system() is 'Windows':
    serial_first_line = 'xx\r\n'
    blit = True

# http://pyserial.readthedocs.org/en/latest/tools.html. Use this to automatically detect arduino

#Slider variables
min_r_motor = -1
max_r_motor = 1

min_l_motor = -1
max_l_motor = 1

min_t_motor = -1
max_t_motor = 1

min_P = 0
max_P = 1000

min_I = 0
max_I = 1000

min_D = 0
max_D = 1000

pid_round = 5

def print_error(e, method):
    print  "CAUGHT: " + e.__class__.__name__ + "\n" + str(e) + "\n" + "METHOD: " + method

def read_loop():
    global reading
    global continue_reading

    global xdata
    global ydata

    log = hamr_file_writer.open_file('hamr', 'test_data', folder_date = True)

    num_data_points = 5
    reading = True

    while device.is_open and continue_reading: # need to check if its open
        try:
            line = device.readline()    # read a \n terminated line
        except Exception as e:
            continue_reading = False
            print_error(e, sys._getframe().f_code.co_name)

        if line == serial_first_line:
            data = [-1 for x in range(0, num_data_points)]
            for j in range(num_data_points):
                try:
                    data[j] = float(device.readline())
                except: # TODO: add the other exception when serial port is closed
                    continue
            
            # do something with data
            # print len(xdata)
            # print len(ydata0)
            # print len(ydata1)

            xd = np.copy(xdata)
            xd[0]  = data[0] / 1000.0
            xd = np.roll(xd, -1)

            tempy = [0,0,0,0]
            for i in range(num_data_points-1):
                tempy[i] = np.copy(ydata[i])
                tempy[i][0] = data[i+1]
                tempy[i] = np.roll(tempy[i], -1)

            for i in range(num_data_points-1):
                ydata[i] = tempy[i]

            xdata = xd

            log.write(','.join(map(str, data)) + "\n")

    log.close()
    reading = False

#########################################################################################################
# CALLBACK FUNCTIONS
#########################################################################################################
#########################################################################################################
def callback_connect(event):
    global continue_reading

    if button_connect.label.get_text() == 'Connect':
        # attempt to connect to the serial port unless it's already open
        if not hamr_serial.connect(device): return
        button_connect.label.set_text("Disconnect")
        print "Connected Succesfully"
    else:
        hamr_serial.write(device, b't')
        continue_reading = False  # tell thread_read to exit
        while reading: pass 
        hamr_serial.disconnect(device)

        button_connect.label.set_text("Connect")
        button_log_data.label.set_text("Log Data")
        print "Disonnected Succesfully"
    
    plt.draw()


def callback_log_data(event):
    global continue_reading
    global thread_read
    global xdata
    global ydata

    if not device.is_open:
        print "You're not connected\n"
        return

    # device.reset_input_buffer()
    if button_log_data.label.get_text() == 'Log Data':

        # attempt to connect
        while device.read()!= 's':
            device.write(b's')
            print "sent start signal\n"
        
        continue_reading = True
        thread_read = threading.Thread(target=read_loop)
        thread_read.start()

        button_log_data.label.set_text('Stop Logging')
        for y in ydata: y[:] = None
    else:
        device.write(b't')
        continue_reading = False
        while(reading): pass

        # attempt to disconnect
        # device.reset_input_buffer()
        while device.read() != 't': 
            device.write(b't')
            print "sent stop signal\n"
        button_log_data.label.set_text('Log Data')
    plt.draw()

def callback_pause_graph(event):
    if not device.is_open:
        print "You're not connected\n"
        return
        
    global pause_graph
    if button_pause_graph.label.get_text() == 'Pause Graph':
        pause_graph = True
        button_pause_graph.label.set_text('Unpause Graph')
    else:
        pause_graph = False
        button_pause_graph.label.set_text('Pause Graph')
    plt.draw()

def callback_update_arduino(event):
    if not device.is_open: return
    write_r_motor(slider_r_motor.val)
    time.sleep(.015)
    write_l_motor(slider_l_motor.val)
    time.sleep(.015)
    write_t_motor(slider_t_motor.val)
    time.sleep(.015)
    write_P(slider_P.val)
    time.sleep(.015)
    write_I(slider_I.val)
    time.sleep(.015)
    write_D(slider_D.val)
    time.sleep(.015)

def callback_reset_arduino(event):
    write_r_motor(0)
    slider_r_motor.set_val(0)
    time.sleep(.015)
    write_l_motor(0)
    slider_l_motor.set_val(0)
    time.sleep(.015)
    write_t_motor(0)
    slider_t_motor.set_val(0)
    time.sleep(.015)

update_arduino_immediately = False
def callback_check(label):
    global update_arduino_immediately
    update_arduino_immediately = not update_arduino_immediately

def write_r_motor(val):
    device.write(b'r')
    device.write(str(val))

def write_l_motor(val):
    device.write(b'l')
    device.write(str(val))


def write_t_motor(val):
    device.write(b'u')
    device.write(str(val))


def write_P(val):
    device.write(b'p')
    device.write(str(val))


def write_I(val):
    device.write(b'i')
    device.write(str(val))


def write_D(val):
    device.write(b'd')
    device.write(str(val))


def callback_slider_r_motor(val):
    if not update_arduino_immediately: return
    write_r_motor(val)


def callback_slider_l_motor(val):
    if not update_arduino_immediately: return
    write_l_motor(val)


def callback_slider_t_motor(val):
    if not update_arduino_immediately: return
    write_t_motor(val)


def callback_slider_P(val):
    if not update_arduino_immediately: return
    rounded_val = int(val/pid_round * pid_round) 
    if val != rounded_val:
        slider_P.set_val(rounded_val)

def callback_slider_I(val):
    if not update_arduino_immediately: return
    rounded_val = int(val/pid_round * pid_round) 
    if val != rounded_val:
        slider_I.set_val(rounded_val)

def callback_slider_D(val):
    if not update_arduino_immediately: return
    rounded_val = int(val/pid_round * pid_round) 
    if val != rounded_val:
        slider_D.set_val(rounded_val)


# def callback


#########################################################################################################
# PLOTTING
#########################################################################################################
#########################################################################################################


# initialize plots
numplots = 4
fig = plt.figure(figsize=(20,8))
plt.subplots_adjust(left=.5, bottom=.2)

# create numplots subplots
height = 2
width = np.ceil(numplots/2.0)
m_axes = [plt.subplot(int(100 * height + 10 * width + x + 1)) for x in range(numplots)]

# initialize empty arrays of size 1000. These will not plot because the yvalue is set to None and should be overwritten
data_size = 1000 # the max number of points to plot at one time

xdata = np.zeros(1000)
ydata = [np.full(1000, None) for x in range(numplots)]

# set axis limits
lines = [m_axes[x].plot(ydata[x], '-')[0] for x in range(numplots)]
for x in range(numplots):
    m_axes[x].set_ylim(-1.5,1.5)
    m_axes[x].set_title('Plot ' + str(x))
    # m_axes[x].get_xaxis().set_visible(False)

xmin = 0 
update_rate = 50
pause_graph = False

# GUI

# BUTTONS
plt.subplots_adjust(left=.3, bottom=0.2)
button_connect = Button(plt.axes([0.01, 0.05, 0.05, 0.075]), 'Connect')
button_connect.on_clicked(callback_connect) 

button_log_data = Button(plt.axes([0.12, 0.05, 0.08, 0.075]), 'Log Data')
button_log_data.on_clicked(callback_log_data)

button_pause_graph = Button(plt.axes([0.21, 0.05, 0.08, 0.075]), 'Pause Graph')
button_pause_graph.on_clicked(callback_pause_graph)

button_update_arduino = Button(plt.axes([0.34, 0.05, 0.1, 0.075]), 'Update Arduino')
button_update_arduino.on_clicked(callback_update_arduino)

button_reset_arduino = Button(plt.axes([0.45, 0.05, 0.1, 0.075]), 'Reset Arduino')
button_reset_arduino.on_clicked(callback_reset_arduino)


# CHECKBOX
check = CheckButtons(plt.axes([0.01, 0.4, 0.1, 0.1]), ['Immediate\n Update'], [False])
check.on_clicked(callback_check)

# SLIDERS
slider_r_motor = Slider(plt.axes([0.06, 0.9, 0.2, 0.03]), 'R Motor Vel', min_r_motor, max_r_motor, valinit=0, valfmt='%1.1f')
slider_r_motor.on_changed(callback_slider_r_motor)

slider_l_motor = Slider(plt.axes([0.06, 0.85, 0.2, 0.03]), 'L Motor Vel', min_l_motor, max_l_motor, valinit=0, valfmt='%1.1f')
slider_l_motor.on_changed(callback_slider_l_motor)

slider_t_motor = Slider(plt.axes([0.06, 0.8, 0.2, 0.03]), 'T Motor Vel', min_t_motor, max_t_motor, valinit=0, valfmt='%1.1f')
slider_t_motor.on_changed(callback_slider_t_motor)

slider_P = Slider(plt.axes([0.06, 0.7, 0.2, 0.03]), 'Proportional', min_P, max_P, valinit=0, valfmt='%1.f')
slider_P.on_changed(callback_slider_P)

slider_I = Slider(plt.axes([0.06, 0.65, 0.2, 0.03]), 'Integral', min_I, max_I, valinit=0, valfmt='%1.f')
slider_I.on_changed(callback_slider_I)

slider_D = Slider(plt.axes([0.06, 0.6, 0.2, 0.03]), 'Derivative', min_D, max_D, valinit=0, valfmt='%1.f')
slider_D.on_changed(callback_slider_D)

# initial state for blitting
def init_blit():
    for j in range(numplots):
        lines[j].set_ydata([])
        lines[j].set_xdata([])
        m_axes[j].set_xlim(xmin, xmin + 10)
    return tuple([line for line in lines])


def update_animation(data):
    global xdata 
    global ydata
    # these should be the only lines needed for live update

    if pause_graph: return tuple([line for line in lines])
    for i in range(numplots):
        lines[i].set_xdata(xdata)
        lines[i].set_ydata(ydata[i])

        m_axes[i].set_xlim(0,10)
        xmax = max(xdata)
        m_axes[i].set_xlim(xmax-10, xmax)

    return tuple([line for line in lines])


def main():
    global device
    global reading  # while this is true, thread_read is still alive
    global continue_reading  # If this is true, then the thread_read thread will not exit

    reading = False 
    device = hamr_serial.initialize(port, baudrate, timeout_=2, write_timeout_=2)

    anim = animation.FuncAnimation(fig, update_animation, init_func=init_blit, interval=update_rate, blit=True)
    # anim._stop()
    # anim._start()

    plt.show()

    continue_reading = False  # tell thread_read to exit
    while reading: pass  # wait until thread_read has finished
    device.close()
    print 'Exit Succesful'

if __name__ == '__main__': main()
