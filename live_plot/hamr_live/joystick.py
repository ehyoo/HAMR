import os
import sys
import platform

import pygame
#from pygame.locals import *
import time
import hamr_serial
from hamr_constants import *

# set precision values for output from joystick
Y_PRECISION = .1
X_PRECISION = .1
THETA_PRECISION = .1


THETA_POSITION = 2
if platform.system() is 'Windows':
    THETA_POSITION = 3


#update this to the HAMR port
port = 'COM4'
baudrate = 250000


def precision(val, prec):
    return round(val / prec) * prec


def init_joystick():
    pygame.init()

    # Set up the joystick
    pygame.joystick.init()

    my_joystick = None
    joystick_names = []

    # Enumerate joysticks
    for i in range(0, pygame.joystick.get_count()):
        joystick_names.append(pygame.joystick.Joystick(i).get_name())

    print joystick_names

    # By default, load the first available joystick.
    if (len(joystick_names) > 0):
        my_joystick = pygame.joystick.Joystick(0)
        my_joystick.init()

    max_joy = max(my_joystick.get_numaxes(), 
                  my_joystick.get_numbuttons(), 
                  my_joystick.get_numhats())

    return my_joystick


def get_readings(my_joystick, use_minimal):
    g_keys = pygame.event.get()

    commands = []
    if (use_minimal):
        # Returns only joystick throttle/pivots (no buttons)
        # 3 readings between [-1,1]: x, y, theta
        for i in range(0, my_joystick.get_numaxes()-1):
            commands.append(my_joystick.get_axis(i))
    else:
        # Returns 4 axes, 12 buttons
        # Axis readings between [-1,1]
        # Button readings binary off/on {0,1}
        for i in range(0, my_joystick.get_numaxes()):
            commands.append(my_joystick.get_axis(i))

        for i in range(0, my_joystick.get_numbuttons()):
            commands.append(my_joystick.get_button(i))

    return commands
   

# initialize joystick and serial connection
def initialize_joystick():
    global joystick
    joystick = init_joystick()
    time.sleep(1)


def initialize_hamr():
    global device
    device = hamr_serial.initialize(port, baudrate, timeout_=2, write_timeout_=2)
    hamr_serial.connect(device)
    time.sleep(1)


def read_joystick():
    global val_dd_v_prev
    global sig_l_prev
    global val_dd_r_prev

    val_dd_v_prev = ""
    sig_l_prev = ""
    val_dd_r_prev = ""

    while (True):
        time.sleep(.1)
        commands = get_readings(joystick, 0)

        # obtain signals
        x = commands[0]     # left/right
        y = -commands[1]    # forward/backward
        theta = commands[THETA_POSITION] # rotation
        send_signal = commands[4] # index finger button. controls signals

        # round signals
        val_dd_v = str(precision(y, Y_PRECISION))
        sig_l = str(precision(x, X_PRECISION))
        val_dd_r = str(precision(theta, THETA_PRECISION))

        # send data to HAMR
        if ((val_dd_v != val_dd_v_prev) or (val_dd_r != val_dd_r_prev)) and send_signal:
        # if equals_float(val_dd_v, val_dd_v_prev) or equals_float(val_dd_r, val_dd_r_prev):
            sending = ""
            if device.is_open:
                sending = "sending:"

                device.write(SIG_DD_V)
                device.write(val_dd_v)
                time.sleep(.1)

                # uncomment this part to send rotation signal
                # device.write(SIG_DD_R)
                # device.write(val_dd_r)
                # time.sleep(.1)

                val_dd_r_prev = val_dd_r
                val_dd_v_prev = val_dd_v

                # device.write(SIG_L_MOTOR)
                # device.write(sig_l)
                # time.sleep(.1)
                # sig_l_prev = sig_l

            print sending + val_dd_v + ", " + val_dd_r


def main():
    initialize_joystick()
    initialize_hamr()
    read_joystick()


def external_main(dev):
    global device
    device = dev
    initialize_joystick()
    read_joystick()


def equals_float(a, b):
    if abs(a-b) < .001: 
        return True
    else: 
        return False


if __name__ == '__main__': main()