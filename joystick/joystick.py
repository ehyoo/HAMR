import os
import sys
import pygame
#from pygame.locals import *
import time
import hamr_serial
from hamr_constants import *

#update this to the HAMR port
port = 'COM4'
baudrate = 250000

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
        
def main():
    joystick = init_joystick()

    device = hamr_serial.initialize(port, baudrate, timeout_=2, write_timeout_=2)
    hamr_serial.connect(device)
    time.sleep(1)
    # device.write(b'1')
    # device.write(b'1')

    sig_r_prev = 0
    sig_l_prev = 0
    sig_t_prev = 0

    while (True):
        time.sleep(.1)
        commands = get_readings(joystick, 0)

        # obtain signals
        x = commands[0]     # left/right
        y = -commands[1]    # forward/backward
        theta = commands[3] # rotation

        # round signals
        sig_r = str(int(y * 10) / 10.0)
        sig_l = str(int(x * 10) / 10.0)
        sig_t = str(int(theta * 10) / 10.0)

        # send data to HAMR
        if(sig_r != sig_r_prev or sig_t != sig_t_prev):
            sending = ""
            if device.is_open:
                sending = "sending:"

                device.write(SIG_R_MOTOR)
                device.write(sig_r)
                time.sleep(.1)

                # uncomment this part to send these signals
                # device.write(SIG_L_MOTOR)
                # device.write(sig_l)
                # time.sleep(.1)

                device.write(SIG_T_MOTOR)
                device.write(sig_t)
                time.sleep(.1)

                sig_r_prev = sig_t
                sig_l_prev = sig_l
                sig_t_prev = sig_r

            print sending + sig_r + ", " + sig_t

if __name__ == '__main__': main()
