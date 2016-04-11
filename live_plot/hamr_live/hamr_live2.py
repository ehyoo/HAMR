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

from matplotlib.widgets import Button, Slider, CheckButtons, MultiCursor, RadioButtons

#custom libraries
import hamr_file_writer
import hamr_serial
from matplotlib_subclassed_widgets import *
import time_me
from hamr_constants import *

import joystick

# Debugging notes: 
# If the Arduino disconnects, disconnect the software first. Then reconnect. Or restart the program.
# Avoid using CTRL + C. 

# IMPORTANT! Ensure these variables correspond to the Arduino
PORT = 'COM4'
BAUDRATE = 250000

SIG_STARTING_STRING = '$\n'
BLIT = False
if platform.system() is 'Windows':
    SIG_STARTING_STRING = '$\r\n'
    BLIT = True

WRITE_DELAY = .03

# http://pyserial.readthedocs.org/en/latest/tools.html. Use this to automatically detect arduino


#helper functions
def print_error(e, method):
    print  "CAUGHT: " + e.__class__.__name__ + "\n" + str(e) + "\n" + "METHOD: " + method

def precision(val, prec):
    return round(val / prec) * prec


#######################################################
# THREAD FOR READING ARDUINO
#######################################################

def read_arduino_update_graph(log):
    global xdata
    global ydata

    data = [-1 for x in range(NUM_DATA_POINTS)]
    for j in range(NUM_DATA_POINTS):
        try:
            data[j] = float(device.readline())
        except: # TODO: distinguish between types of exceptions(float conversion and serial)
            continue
    
    # if(data[0] < xdata[999]):
        # log.write(','.join(map(str, data)) + "\n")
        # return

    xd = np.copy(xdata)
    xd[0]  = data[0] / 1000.0
    xd = np.roll(xd, -1)

    tempy = [0 for x in range(NUM_DATA_POINTS)]
    for i in range(NUM_DATA_POINTS-1):
        tempy[i] = np.copy(ydata[i])
        tempy[i][0] = data[i+1]
        tempy[i] = np.roll(tempy[i], -1)

    for i in range(NUM_DATA_POINTS-1):
        ydata[i] = tempy[i]

    xdata = xd

    log.write(','.join(map(str, data)) + "\n")


def read_loop():
    global reading
    # global xdata
    # global ydatas

    log = hamr_file_writer.open_file('hamr', 'test_data', folder_date = True)

    reading = True

    while device.is_open and continue_reading: 
        # this try block exits the thread when device.is_open fails
        try: 
            line = device.readline()    # read a \n terminated line
        except Exception as e:
            print_error(e, sys._getframe().f_code.co_name)
            disconnect_arduino()

        # print the portion of line that does not contain SIG_STARTING_STRING
        if line.endswith(SIG_STARTING_STRING) and len(line) > len(SIG_STARTING_STRING):
            print line[0:-1 * len(SIG_STARTING_STRING)]

        if SIG_STARTING_STRING in line:
            read_arduino_update_graph(log)

    log.close()
    reading = False


#######################################################
# THREAD FOR READING AND SENDING JOYSTICK SIGNAL
#######################################################
def read_joystick():
    joystick.external_main(device) # pass in a label object 

#######################################################
# ARDUINO CONTROL FUNCTIONS
#######################################################
def connect_arduino():
    if not hamr_serial.connect(device): 
        print "Connection Failed"
        return

    button_connect.label.set_text("Disconnect")
    xdata = np.zeros(DATA_SIZE)
    for y in ydata: y[:] = None
    print "Connected Succesfully"


def disconnect_arduino():
    global continue_reading
     # tell thread_read to exit and wait for it to do so
    continue_reading = False
    while reading: pass 

    hamr_serial.write(device, SIG_STOP_LOG)
    hamr_serial.disconnect(device)

    button_connect.label.set_text("Connect")
    button_log_data.label.set_text("Log Data")
    print "Disonnected Succesfully"


def log_arduino():
    global continue_reading
    global thread_read
    global xdata
    global ydata

    # attempt to connect. wait 1000 ms before giving up
    device.write(SIG_START_LOG)
    print "Sent SIG_START_LOG\n"
    timer = time_me.TimeMe(1000)
    connected = True
    while device.read() != SIG_START_LOG:
        if timer.times_up(): # give up
            connected = False
            print "Failed to receive confirmation signal\n"
            break

    
    if connected:
        continue_reading = True
        thread_read = threading.Thread(target=read_loop)
        thread_read.start()

        button_log_data.label.set_text('Stop Logging')

        # reset graphing data arrays
        xdata = np.zeros(DATA_SIZE)
        for y in ydata: y[:] = None


def end_log_arduino():
    global continue_reading
    # tell thread read to exit and wait for it to do so
    continue_reading = False
    while(reading): pass

    # attempt to disconnect
    device.write(SIG_STOP_LOG) 
    print "Sent SIG_STOP_LOG\n"
    timer = time_me.TimeMe(2000)
    while device.read() != SIG_STOP_LOG:
        if timer.times_up(): #give up
            print "Failed to receive confirmation signal\n"
            break
    button_log_data.label.set_text('Log Data')


#######################################################
# CALLBACK FUNCTIONS
#######################################################
def callback_connect(event):
    if button_connect.label.get_text() == 'Connect':
        connect_arduino()
    else:
        disconnect_arduino()


def callback_log_data(event):
    if not device.is_open:
        print "You're not connected\n"
        return

    if button_log_data.label.get_text() == 'Log Data':
        log_arduino()
    else:
        end_log_arduino()


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


def callback_update_arduino(event):
    if not device.is_open: return
    write_r_motor(slider_input_1.val)
    time.sleep(WRITE_DELAY)
    # write_l_motor(slider_input_2.val)
    # time.sleep(WRITE_DELAY)
    write_t_motor(slider_input_3.val)
    time.sleep(WRITE_DELAY)
    write_P(slider_P.val)
    time.sleep(WRITE_DELAY)
    write_I(slider_I.val)
    time.sleep(WRITE_DELAY)
    write_D(slider_D.val)
    time.sleep(WRITE_DELAY)

def callback_reset_arduino(event):
    write_r_motor(0)
    slider_input_1.set_val(0)
    time.sleep(WRITE_DELAY)
    write_l_motor(0)
    # slider_input_2.set_val(0)
    # time.sleep(WRITE_DELAY)
    write_t_motor(0)
    slider_input_3.set_val(0)
    time.sleep(WRITE_DELAY)


update_arduino_immediately = False
def callback_check(label):
    global update_arduino_immediately
    update_arduino_immediately = not update_arduino_immediately


def write_r_motor(val):
    # global r_motor_prev
    val = precision(val, .1)
    # if r_motor_prev  != val:
    device.write(b'r')
    device.write(str(val))
    # r_motor_prev = val

def write_l_motor(val):
    # global l_motor_prev
    val = precision(val, .1)
    # if l_motor_prev  != val:
    device.write(b'l')
    device.write(str(val))
    # l_motor_prev = val

def write_t_motor(val):
    # global t_motor_prev
    val = precision(val, .1)
    # if t_motor_prev  != val:
    device.write(b't')
    device.write(str(val))
    # t_motor_prev = val

input_motors = [SIG_R_MOTOR, SIG_L_MOTOR, SIG_T_MOTOR]
input_holonomic = [SIG_HOLO_X, SIG_HOLO_Y, SIG_HOLO_R]
input_dd = [SIG_DD_V, SIG_DD_R, b'^'] # the third signal is a placeholder. it should never be used
current_input = input_holonomic

def write_input_1(val):
    print current_input[0]
    print str(val)
    val = precision(val, INPUT_PRECISION)
    device.write(current_input[0])
    device.write(str(val))

    

def write_input_2(val):
    print current_input[1]
    print str(val)
    val = precision(val, INPUT_PRECISION)
    device.write(current_input[1])
    device.write(str(val))

    

def write_input_3(val):
    print current_input[2]
    print str(val)
    val = precision(val, INPUT_PRECISION)
    device.write(current_input[2])
    device.write(str(val))

    




# map labels to control signals
current_mode = 'HOLO X'
P_map = {'R Motor': SIG_R_KP, 'L Motor': SIG_L_KP, 'T Motor': SIG_T_KP, 'HOLO X': SIG_HOLO_X_KP, 'HOLO Y': SIG_HOLO_Y_KP, 'HOLO R': SIG_HOLO_R_KP}
I_map = {'R Motor': SIG_R_KI, 'L Motor': SIG_L_KI, 'T Motor': SIG_T_KI, 'HOLO X': SIG_HOLO_X_KI, 'HOLO Y': SIG_HOLO_Y_KI, 'HOLO R': SIG_HOLO_R_KI}
D_map = {'R Motor': SIG_R_KD, 'L Motor': SIG_L_KD, 'T Motor': SIG_T_KD, 'HOLO X': SIG_HOLO_X_KD, 'HOLO Y': SIG_HOLO_Y_KD, 'HOLO R': SIG_HOLO_R_KD}

def write_P(val):
    # print P_map[current_mode]
    # print str(val)
    # print "---------"
    # print "---------"
    val = precision(val, PID_PRECISION)
    device.write(P_map[current_mode])
    device.write(str(val))
    time.sleep(WRITE_DELAY)


def write_I(val):
    # print I_map[current_mode]
    # print str(val)
    # print "---------"
    # print "---------"
    val = precision(val, PID_PRECISION)
    device.write(I_map[current_mode])
    device.write(str(val))
    time.sleep(WRITE_DELAY)


def write_D(val):
    # print D_map[current_mode]
    # print str(val)
    # print "---------"
    # print "---------"
    val = precision(val, PID_PRECISION)
    device.write(D_map[current_mode])
    device.write(str(val))
    time.sleep(WRITE_DELAY)


def callback_slider_input_1(val):
    if not update_arduino_immediately: return
    write_input_1(val)


def callback_slider_input_2(val):
    if not update_arduino_immediately: return
    write_input_2(val)


def callback_slider_input_3(val):
    if not update_arduino_immediately: return
    write_input_3(val)


# PID SLIDERS
def callback_slider_P(val):
    if not update_arduino_immediately: return
    write_P(val)


def callback_slider_I(val):
    if not update_arduino_immediately: return
    write_I(val)


def callback_slider_D(val):
    if not update_arduino_immediately: return
    write_D(val)

# RADIO BUTTONS FOR MODE SELECTION
def callback_radio_mode(label):
    global current_mode
    global current_input
    current_mode = label

    if 'Motor' in label:
        current_input = input_motors
        slider_input_1.label.set_text('M1 Velocity')
        slider_input_2.label.set_text('M2 Velocity')
        slider_input_3.label.set_text('MT Velocity')
    elif 'Holo' in label:
        current_input = input_holonomic
        slider_input_1.label.set_text('X dot')
        slider_input_2.label.set_text('Y dot')
        slider_input_3.label.set_text('Theta dot')
    else:
        current_input = input_dd
        slider_input_1.label.set_text('Forward V')
        slider_input_2.label.set_text('Theta dot')
        slider_input_3.label.set_text('N/A')
    plt.draw()


    # else if 'DD' in label:
        # current_input = ''


#######################################################
# PLOTTING                                    
#######################################################

fig = plt.figure(figsize=(21,8), facecolor='#E1E6E8')
plt.subplots_adjust(left=.3)

# create NUM_PLOTS subplots in 2 rows
graph_columns = np.ceil(NUM_PLOTS/float(GRAPH_ROWS))
m_axes = [plt.subplot(int(100 * GRAPH_ROWS + 10 * graph_columns + x + 1)) for x in range(NUM_PLOTS)]

xdata = np.zeros(DATA_SIZE)
ydata = [np.full(DATA_SIZE, None) for x in range(NUM_PLOTS)]

# set axis limits
lines = [m_axes[x].plot(ydata[x], '-')[0] for x in range(NUM_PLOTS)]
for x in range(NUM_PLOTS):
    m_axes[x].set_ylim(-1.1,1.1)
    # m_axes[x].set_title('Plot ' + str(x))
    # m_axes[x].get_xaxis().set_visible(False)
m_axes[0].set_title('Right Motor Velocity (m/s)')
m_axes[1].set_title('Left Motor Velocity (m/s)')
m_axes[2].set_title('Angular Velocity (deg/s)')
m_axes[2].set_ylim(-200,200)


m_axes[3].set_title('holonomic xdot (m/s)')
m_axes[4].set_title('holonomic ydot (m/s)')
m_axes[5].set_title('lower base angle (deg)')
m_axes[5].set_ylim(0,360)


m_axes[6].set_title('Input xdot (m/s)')
m_axes[7].set_title('Input ydot (m/s)')
m_axes[8].set_title('Input thetadot (deg/s)')
m_axes[8].set_ylim(-200,200)

# Note: to change the y limits of a particular graph, edit the following line
# m_axes['graph number'].set_ylim(-1.5,1.5)

pause_graph = False

# initial state for blitting
def init_blit():
    for j in range(NUM_PLOTS):
        lines[j].set_ydata([])
        lines[j].set_xdata([])
        m_axes[j].set_xlim(-10, 0)

    return tuple([line for line in lines])


def update_animation(data):
    global xdata 
    global ydata

    if pause_graph: return []
    for i in range(NUM_PLOTS):
        lines[i].set_xdata(xdata)
        lines[i].set_ydata(ydata[i])

        xmax = max(xdata)
        m_axes[i].set_xlim(xmax-10, xmax)
    return tuple([line for line in lines])

#######################################################
# GUI                                   
#######################################################

# BUTTONS
button_connect = MyButton(plt.axes([0.01, 0.05, 0.05, 0.075]), 'Connect', color='#FFFFFF')
button_connect.on_clicked(callback_connect) 

button_log_data = MyButton(plt.axes([0.08, 0.05, 0.08, 0.075]), 'Log Data', color='#FFFFFF')
button_log_data.on_clicked(callback_log_data)

button_pause_graph = MyButton(plt.axes([0.17, 0.05, 0.08, 0.075]), 'Pause Graph', color='#FFFFFF')
button_pause_graph.on_clicked(callback_pause_graph)

button_update_arduino = MyButton(plt.axes([0.01, 0.2, 0.1, 0.075]), 'Update Arduino', color='#FFFFFF')
button_update_arduino.on_clicked(callback_update_arduino)

button_reset_arduino = MyButton(plt.axes([0.12, 0.2, 0.1, 0.075]), 'Reset Arduino', color='#FFFFFF')
button_reset_arduino.on_clicked(callback_reset_arduino)

# CHECKBOX
check = MyCheckBox(plt.axes([0.01, 0.4, 0.1, 0.1]), ['Immediate\n Update'], [False])
check.on_clicked(callback_check)

# SLIDERS
slider_drag = True

slider_input_1 = MySlider(plt.axes([0.06, 0.9, 0.18, 0.03]), 'X dot', MIN_R_MOTOR, MAX_R_MOTOR, valinit=0, valfmt='%1.2f', dragging = slider_drag)
slider_input_1.on_changed(callback_slider_input_1)

slider_input_2 = MySlider(plt.axes([0.06, 0.85, 0.18, 0.03]), 'Y dot', MIN_L_MOTOR, MAX_L_MOTOR, valinit=0, valfmt='%1.2f', dragging = slider_drag)
slider_input_2.on_changed(callback_slider_input_2)

slider_input_3 = MySlider(plt.axes([0.06, 0.8, 0.18, 0.03]), 'Theta dot', MIN_T_MOTOR, MAX_T_MOTOR, valinit=0, valfmt='%1.2f', dragging = slider_drag)
slider_input_3.on_changed(callback_slider_input_3)

slider_P = MySlider(plt.axes([0.03, 0.7, 0.22, 0.03]), 'P', MIN_P, MAX_P, valinit=0, valfmt='%1.2f', dragging = slider_drag)
slider_P.on_changed(callback_slider_P)

slider_I = MySlider(plt.axes([0.03, 0.65, 0.22, 0.03]), 'I', MIN_I, MAX_I, valinit=0, valfmt='%1.2f', dragging = slider_drag)
slider_I.on_changed(callback_slider_I)

slider_D = MySlider(plt.axes([0.03, 0.6, 0.22, 0.03]), 'D', MIN_D, MAX_D, valinit=0, valfmt='%1.3f', dragging = slider_drag)
slider_D.on_changed(callback_slider_D)

#RADIO BUTTON
radio_mode = RadioButtons(plt.axes([0.13, 0.3, 0.12, 0.28]), ('Holo X', 'Holo Y', 'Holo R', 'R Motor', 'L Motor', 'T Motor', 'DD V', 'DD R'))
radio_mode.on_clicked(callback_radio_mode)


#######################################################
# MAIN                                 
#######################################################
def main():
    global device
    global reading  # while this is true, thread_read is still alive
    global continue_reading  # If this is true, then the thread_read thread will not exit

    # begin setup hamr serial connection
    reading = False     
    device = hamr_serial.initialize(PORT, BAUDRATE, timeout_=.3, write_timeout_=.3)

    # begin joystick thread
    thread_joystick = threading.Thread(target=read_joystick)
    thread_joystick.start()

    # begin animating plot
    anim = animation.FuncAnimation(fig, update_animation, init_func=init_blit, interval=GRAPH_UPDATE_DELAY, blit=BLIT)
    plt.show()

    # program was closed
    continue_reading = False  # tell thread_read to exit
    while reading: pass  # wait until thread_read has finished
    device.close()
    print 'Program Exitted'

if __name__ == '__main__': main()