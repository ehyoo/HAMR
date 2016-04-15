/*
 * Robot Constants
 */
#define TICKS_PER_REV_DDRIVE  4096.0  // number of encoder ticks in one full rotation on diff drive motor
#define TICKS_PER_REV_TURRET  360.0  // turret motor (250:1 gear ratio * 12)
#define WHEEL_DIAMETER        0.060325 // in meters (2 3/8" diameter)  
#define WHEEL_RADIUS          (WHEEL_DIAMETER / 2.0)  // wheel radius, in meters
#define WHEEL_DIST            0.19812 // distance between diff drive wheels, in meters (7.8")
#define DIST_PER_REV          (PI*WHEEL_DIAMETER)  // circumference of wheel in meters
#define LOOPTIME              20.0 // in ms      

// For live-plotting
/* SIGNALS */
#define SIG_START_STRING '$'
#define SIG_START_LOG '['
#define SIG_STOP_LOG ']'
#define SIG_UNINITIALIZED '!'

#define SIG_HOLO_X 'x'
#define SIG_HOLO_Y 'y'
#define SIG_HOLO_R 'a'

#define SIG_DD_V 'd'
#define SIG_DD_R 'D'

#define SIG_R_MOTOR 'r'
#define SIG_L_MOTOR 'l'
#define SIG_T_MOTOR 't'

#define SIG_R_KP '1'
#define SIG_R_KI '2'
#define SIG_R_KD '3'

#define SIG_L_KP '4'
#define SIG_L_KI '5'
#define SIG_L_KD '6'

#define SIG_T_KP '7'
#define SIG_T_KI '8'
#define SIG_T_KD '9'

#define SIG_HOLO_X_KP 'Q'
#define SIG_HOLO_X_KI 'W'
#define SIG_HOLO_X_KD 'E'

#define SIG_HOLO_Y_KP 'R'
#define SIG_HOLO_Y_KI 'T'
#define SIG_HOLO_Y_KD 'Y'

#define SIG_HOLO_R_KP 'U'
#define SIG_HOLO_R_KI 'I'
#define SIG_HOLO_R_KD 'O'


/**********************************************
 * Pin Definitions
 * 
 * M1 = Left Differential Drive (ddrive) motor 
 * M2 = Right Differential Drive (ddrive) motor
 * MT = Turret motor
 **********************************************/
/* Motor Driver Pinouts */
#define M1_PWM_PIN 2
#define M1_DIR_PIN 43
#define M1_SLP_PIN 45
#define M1_FLT_PIN 47

#define M2_PWM_PIN 3
#define M2_DIR_PIN 49
#define M2_SLP_PIN 51
#define M2_FLT_PIN 53

#define MT_PWM_PIN 4
#define MT_DIR_PIN 37
#define MT_SLP_PIN 39
#define MT_FLT_PIN 41


/* Decoder Pinouts */
#define DECODER_SEL_PIN 48
#define DECODER_OE_PIN 50
#define DECODER_RST_PIN 52

int M1_DECODER_D_PINS[8] = {19,18,17,16,15,14,24,22}; // D0-D7 pinouts
int M2_DECODER_D_PINS[8] = {5,7,8,9,10,11,12,13}; // D0-D7 pinouts
int MT_DECODER_D_PINS[8] = {44,42,40,38,36,34,32,30}; // D0-D7 pinouts
