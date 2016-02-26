#ifndef Motor_h
#define Motor_h

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

#define TICKS_PER_REV_ddrive  1632.0  // number of encoder ticks in one full rotation on diff drive (ddrive) motor
#define TICKS_PER_REV_turret  3000.0  // turret (turret) motor (250:1 gear ratio * 12)
#define WHEEL_DIAMETER    0.060325 // in meters (2 3/8" diameter)   
#define DIST_PER_REV      (PI*WHEEL_DIAMETER)  // circumference of wheel in meters
#define LOOPTIME          50.0 // in ms      

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir);

void set_speed(int pwm_val, int pin_driver_inA, int pin_driver_inB, int pin_pwm);

void measure_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count); // calculate speed

void measure_rot_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count); // calculate speed

#endif
