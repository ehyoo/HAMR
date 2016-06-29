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

    tempy = [0 for x in range(NUM_PLOTS)]
    for i in range(NUM_PLOTS):
        tempy[i] = np.copy(ydata[i])
        tempy[i][0] = data[i+1]
        tempy[i] = np.roll(tempy[i], -1)

    for i in range(NUM_PLOTS):
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
# PLOTTING                                    
#######################################################



# m_axes[6].set_title('desired_M1_v')
# m_axes[7].set_title('desired_M2_v')
# m_axes[8].set_title('desired_MT_v')
# m_axes[8].set_ylim(-360,360)


# Note: to change the y limits of a particular graph, edit the following line
# m_axes['graph number'].set_ylim(-1.5,1.5)

## AYY LMAO

## AYYY

# initial state for blitting


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

button_update_arduino = MyButton(plt.axes([0.01, 0.2, 0.1, 0.075]), 'Update', color='#FFFFFF')
button_update_arduino.on_clicked(callback_update_arduino)

# button_reset_arduino = MyButton(plt.axes([0.12, 0.2, 0.1, 0.075]), 'Reset', color='#FFFFFF')
# button_reset_arduino.on_clicked(callback_reset_arduino)

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
radio_mode = RadioButtons(plt.axes([0.13, 0.3, 0.12, 0.28]), ('HOLO X', 'HOLO Y', 'HOLO R', 'R Motor', 'L Motor', 'T Motor', 'DD V', 'DD R'))
radio_mode.on_clicked(callback_radio_mode)

#######################################################
# MAIN                                 
#######################################################
def main():
    gv.device
    gv.reading 
    gv.continue_reading

    # begin setup hamr serial connection
    gv.reading = False     
    gv.device = hamr_serial.initialize(
        PORT, BAUDRATE, timeout_=.3, write_timeout_=.3)

    # begin joystick thread
    thread_joystick = threading.Thread(target=read_joystick)
    thread_joystick.start()

    # begin animating plot
    anim = animation.FuncAnimation(fig, update_animation, init_func=init_blit, interval=GRAPH_UPDATE_DELAY, blit=BLIT)
    plt.show()

    # program was closed
    gv.continue_reading = False  # tell thread_read to exit
    while reading: pass  # wait until thread_read has finished
    device.close()
    print 'Program Exitted'

if __name__ == '__main__': main()