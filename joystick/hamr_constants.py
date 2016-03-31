"""
This file contains all non critical constants or constants that are not meant to be changed (the SIG consstants).
"""

# SLIDERS
MIN_R_MOTOR = -1
MAX_R_MOTOR = 1

MIN_L_MOTOR = -255
MAX_L_MOTOR = 255

MIN_T_MOTOR = -1
MAX_T_MOTOR = 1

MIN_P = 0
MAX_P = 5

MIN_I = 0
MAX_I = 5

MIN_D = 0
MAX_D = .05

PID_ROUND = 5

# GRAPHING 
NUM_DATA_POINTS = 5
NUM_PLOTS = 4
DATA_SIZE = 500
GRAPH_UPDATE_DELAY = 20

GRAPH_ROWS = 2


# SIGNALS
SIG_START_LOG = b'['
SIG_STOP_LOG = b']'

SIG_HOLO_X = b'x'
SIG_HOLO_Y = b'y'
SIG_HOLO_T = b'a'

SIG_DD_V = b'd'
SIG_DD_R = b'D'

SIG_R_MOTOR = b'r'
SIG_L_MOTOR = b'l'
SIG_T_MOTOR = b't'

SIG_R_KP = b'1'
SIG_R_KI = b'2'
SIG_R_KD = b'3'

SIG_L_KP = b'4'
SIG_L_KI = b'5'
SIG_L_KD = b'6'

SIG_T_KP = b'7'
SIG_T_KI = b'8'
SIG_T_KD = b'9'
