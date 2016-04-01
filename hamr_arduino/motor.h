#ifndef Motor_h
#define Motor_h

#include <Servo.h>
#include "pid.h"

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

void init_servo(Servo* motor, int pwm_pin);

void set_servo_speed(Servo* motor, 
                     PID_Vars* pid, 
                     float ang_speed_req,
                     float ang_speed_act,
                     float t_elapsed);

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir);

void set_speed(PID_Vars* pid, 
               float speed_req, 
               float speed_act, 
               float* speed_cmd,
               float t_elapsed, 
               int* pwm_val,
               int pin_driver_inA, 
               int pin_driver_inB, 
               int pin_pwm);
               
float get_speed(long encoder_counts,
                float ticks_per_rev, 
                float dist_per_rev, 
                float time_elapsed);
               
float get_ang_speed(long encoder_counts,
                    float ticks_per_rev,
                    float time_elapsed);

                    
#endif
