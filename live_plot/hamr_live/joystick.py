import os
import sys
import platform

import pygame
#from pygame.locals import *
import time
import hamr_serial
from hamr_constants import *

MODE_DD = 1
MODE_HOLONOMIC = 2
MODE_MOTORS = 3 # not implemented

# set precision values for output from joystick
Y_PRECISION = .01
X_PRECISION = .01
THETA_PRECISION = .01<


THETA_POSITION = 2
if platform.system() is 'Windows':
    THETA_POSITION = 3

WRITE_DELAY = .03

#update this to the HAMR port
# port = 'COM3'
port = 'COM12'
baudrate = 250000

# set joystick mode
mode = MODE_MOTORS


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
    global vertical_prev
    global horizontal_prev
    global rotational_prev

    vertical_prev = ""
    horizontal_prev = ""
    rotational_prev = ""

    while (True):
        time.sleep(.1)
        commands = get_readings(joystick, 0)

        # obtain signals
        x = commands[0]     # left/right
        y = -commands[1]    # forward/backward
        theta = commands[THETA_POSITION] # rotation
        send_signal = commands[4] # index finger button. controls signals

        # round signals
        horizontal = str(precision(x / 5.0, X_PRECISION))
        vertical = str(precision(y / 5.0, Y_PRECISION))
        rotational = str(precision(theta, THETA_PRECISION))

        vertical_M1_pwm = str(int(50 * y))
        vertical_M2_pwm = str(int(50 * y))
        rotational_MT_pwm = str(int(-50 * y))

        # send data to HAMR
        # if ((vertical != vertical_prev) or (rotational != rotational_prev)) and send_signal:
        if send_signal:
            sending = ""
            if device.is_open:
                sending = "sending:"

                if mode == MODE_DD:
                    device.write(SIG_DD_V)
                    device.write(vertical) 
                    time.sleep(WRITE_DELAY)

                    device.write(SIG_DD_R)
                    device.write(rotational)
                elif mode == MODE_HOLONOMIC:
                    device.write(SIG_HOLO_X)
                    device.write(horizontal)
                    time.sleep(WRITE_DELAY)

                    device.write(SIG_HOLO_Y)
                    device.write(vertical)
                    time.sleep(WRITE_DELAY)

                else:
                    device.write(SIG_R_MOTOR)
                    device.write(vertical_M1_pwm)
                    time.sleep(.01)

                    # device.write(SIG_L_MOTOR)
                    # device.write(vertical_M2_pwm)
                    # time.sleep(.01)

                    # device.write(SIG_T_MOTOR)
                    # device.write(rotational_MT_pwm)
                    # time.sleep(.01)

                    # device.write(SIG_HOLO_R)
                    # device.write(rotational)

            # if mode == MODE_MOTORS:
            #     print sending + vertical_M1_pwm + ", " + vertical_M2_pwm + ", " + rotational_MT_pwm
            # else: 
            #     print sending + horizontal + ", " + vertical + ", " + rotational

        # uncomment below to show signals
        # print 'joystick:' + horizontal + ", " + vertical + ", " + rotational


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