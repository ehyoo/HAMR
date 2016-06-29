
from kivy.uix.button import Button
# In-house
import arduino_functions
import global_variables as gv
import constants

# right, left, theta 
r_slider_value = 0
l_slider_value = 0
t_slider_value = 0
# PID 
p_slider_value = 0
i_slider_value = 0
d_slider_value = 0

def connect(button_instance):
    if button_instance.text == 'Connect':
        arduino_functions.connect_arduino(button_instance)
        button_instance.text = 'Disconnect'
    elif button_instance.text == 'Disconnect':
        arduino_functions.disconnect_arduino(button_instance)
        button_instance.text = 'Connect'

def log_data(button_instance):
    if not gv.device.is_open:
        raise IOError("You're not connected\n")
        return
    if button_instance.text == 'Log Data':
        try:
            arduino_functions.log_arduino()
            button_instance.text = 'Stop Logging'
        except IOError:
            print 'Something went wrong in callback_log_data'
    else:
        try:
            arduino_functions.end_log_arduino()
            button_instance.text = 'Log Data'
        except IOError:
            print 'Something went wrong in callback_log_data'

def pause_graph(button_instance):
    if not gv.device.is_open:
        print "You're not connected\n"
        return
    if button_instance.text == 'Pause Graph':
        gv.pause_graph = True
        button_instance.text = 'Unpause Graph'
    else:
        gv.pause_graph = False
        button_instance.text = 'Pause Graph'


def update_arduino(event):
    if not device.is_open: return
    # needs all the slider inputs
    write_r_motor(slider_input_1.val)
    time.sleep(constants.WRITE_DELAY)
    # WHY WAS THIS COMMMENTED OUT
    write_l_motor(slider_input_2.val)
    time.sleep(constants.WRITE_DELAY)
    #
    write_t_motor(slider_input_3.val)
    time.sleep(constants.WRITE_DELAY)
    write_P(slider_P.val)
    time.sleep(constants.WRITE_DELAY)
    write_I(slider_I.val)
    time.sleep(constants.WRITE_DELAY)
    write_D(slider_D.val)
    time.sleep(constants.WRITE_DELAY)

def reset_arduino(event):
    write_r_motor(0)
    slider_input_1.set_val(0)
    time.sleep(constants.WRITE_DELAY)
    write_l_motor(0)
    # WHY WAS THIS COMMENTED OUT
    slider_input_2.set_val(0)
    time.sleep(constants.WRITE_DELAY)
    #
    write_t_motor(0)
    slider_input_3.set_val(0)
    time.sleep(constants.WRITE_DELAY)