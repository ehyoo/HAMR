

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