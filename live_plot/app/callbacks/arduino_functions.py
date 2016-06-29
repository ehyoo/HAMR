# TODO: Consider putting this file somewhere else.
# Just putting this in the callbacks directory- should probably be 
# somewhere else but it's only being used for callbacks. 

# Functions about the arduino- used for the functions in dashboard_callbacks


## needs:
"""
./hamr_serial
./views/dashboard.py
"""
##

def connection_callback(button_instance):
    # a controller that directs the callback to connect or disconnect
    if button_instance.text == 'Connect':
        connect_arduino(button_instance)
        button_instance.text = 'Disconnect'
    elif button_instance.text == 'Disconnect':
        disconnect_arduino(button_instance)
        button_instance.text = 'Connect'

def connect_arduino(button_instance):
    if not hamr_serial.connect(device): 
        print "Connection Failed"
        return
    
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