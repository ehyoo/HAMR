#######################################################
# CALLBACK FUNCTIONS
#######################################################

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
    elif 'HOLO' in label:
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
