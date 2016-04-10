/*
 * Robot Constants
 */
#define TICKS_PER_REV_DDRIVE  1632.0  // number of encoder ticks in one full rotation on diff drive motor
#define TICKS_PER_REV_TURRET  3000.0  // turret motor (250:1 gear ratio * 12)
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


/**********************************************
 * Pin Definitions
 * 
 * M1 = Left Differential Drive (ddrive) motor 
 * M2 = Right Differential Drive (ddrive) motor
 * M3 = Turret motor
 **********************************************/
// DD motor encoders
#define PIN_M1_ENCODER_OUTA 38
#define PIN_M1_ENCODER_OUTB 40
#define PIN_M2_ENCODER_OUTA 32
#define PIN_M2_ENCODER_OUTB 34
// Turret motor encoder
#define PIN_M3_ENCODER_OUTA 30
#define PIN_M3_ENCODER_OUTB 28
// DD motor driver
#define PIN_DD_EN 22
#define PIN_M1_DRIVER_INA 24
#define PIN_M1_DRIVER_INB 26
#define PIN_M1_DRIVER_PWM 3
#define PIN_M2_DRIVER_INA 48
#define PIN_M2_DRIVER_INB 50
#define PIN_M2_DRIVER_PWM 4
// Turret motor driver
#define PIN_M3_DRIVER_INA 41
#define PIN_M3_DRIVER_INB 43
#define PIN_M3_DRIVER_PWM 2
 