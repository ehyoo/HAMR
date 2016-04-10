"""
This file contains all non critical constants or constants that are not meant to be changed (the SIG consstants).
"""

# SLIDERS
MIN_R_MOTOR = -1
MAX_R_MOTOR = 1

MIN_L_MOTOR = -1
MAX_L_MOTOR = 1

MIN_T_MOTOR = -1
MAX_T_MOTOR = 1

MIN_P = 0
MAX_P = .2

MIN_I = 0
MAX_I = .2

MIN_D = 0
MAX_D = .05

PID_ROUND = 5

INPUT_PRECISION = .01
PID_PRECISION = .01


# GRAPHING 
NUM_DATA_POINTS = 5
NUM_PLOTS = 8
DATA_SIZE = 500
GRAPH_UPDATE_DELAY = 20

GRAPH_ROWS = 2


# SIGNALS
SIG_START_LOG = b'['
SIG_STOP_LOG = b']'

# inputs
SIG_HOLO_X = b'x'
SIG_HOLO_Y = b'y'
SIG_HOLO_R = b'a'

SIG_DD_V = b'd'
SIG_DD_R = b'D'

SIG_R_MOTOR = b'r'
SIG_L_MOTOR = b'l'
SIG_T_MOTOR = b't'

# pid params
SIG_R_KP = b'1'
SIG_R_KI = b'2'
SIG_R_KD = b'3'

SIG_L_KP = b'4'
SIG_L_KI = b'5'
SIG_L_KD = b'6'

SIG_T_KP = b'7'
SIG_T_KI = b'8'
SIG_T_KD = b'9'

SIG_HOLO_X_KP = b'Q'
SIG_HOLO_X_KI = b'W'
SIG_HOLO_X_KD = b'E'

SIG_HOLO_Y_KP = b'R'
SIG_HOLO_Y_KI = b'T'
SIG_HOLO_Y_KD = b'Y'

SIG_HOLO_R_KP = b'U'
SIG_HOLO_R_KI = b'I'
SIG_HOLO_R_KD = b'O'
