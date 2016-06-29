## Description:
# Functions about the arduino- used for the functions in dashboard_callbacks 

# TODO: Consider putting this file somewhere else.
# Just putting this in the callbacks directory- should probably be 
# somewhere else but it's only being used for callbacks. 

# Not in-house libraries
import sys
import numpy as np

# In-house libraries/modules/constants
sys.path.append('../modules')
sys.path.append('../shared')
import hamr_serial
import time_me
import global_variables as gv
import constants

def connect_arduino(button_instance):
    # Connects to hamr and clears xdata and ydata
    if not hamr_serial.connect(device): 
        raise IOError('Connection Failed')
        return
    gv.xdata = np.zeros(DATA_SIZE)
    for y in gv.ydata: y[:] = None
    print "Connected Succesfully"

def disconnect_arduino():
     # tell thread_read to exit and wait for it to do so
    gv.continue_reading = False
    while reading: pass # waits for reading to turn to false

    hamr_serial.write(gv.device, constants.SIG_STOP_LOG)
    hamr_serial.disconnect(gv.device)

    # button_log_data.label.set_text("Log Data") # TODO: change logdata button
    print "Disonnected Succesfully"


def log_arduino():  
    # attempt to connect. wait 1000 ms before giving up
    device.write(constants.SIG_START_LOG)
    print "Sent SIG_START_LOG\n"
    timer = time_me.TimeMe(1000)
    connected = True
    while gv.device.read() != constants.SIG_START_LOG:
        if timer.times_up(): # give up
            connected = False
            raise IOError("Failed to receive confirmation signal\n")
            break
    if connected:
        continue_reading = True
        gv.thread_read = threading.Thread(target=read_loop)
        gv.thread_read.start()
        # reset graphing data arrays
        gv.xdata = np.zeros(DATA_SIZE)
        for y in gv.ydata: y[:] = None


def end_log_arduino(button_instance):
    # tell thread read to exit and wait for it to do so
    gv.continue_reading = False
    while(gv.reading): pass
    # attempt to disconnect
    gv.device.write(constants.SIG_STOP_LOG) 
    print "Sent SIG_STOP_LOG\n"
    timer = time_me.TimeMe(2000)
    while gv.device.read() != constants.SIG_STOP_LOG:
        if timer.times_up(): #give up
            raise IOError("Failed to receive confirmation signal\n")
            break
    # button_instance.set_text('Log Data') TODO: change this