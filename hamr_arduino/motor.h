#ifndef Motor_h
#define Motor_h

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir);

void set_speed(int pwm_val, int pin_driver_inA, int pin_driver_inB, int pin_pwm);

float get_speed(long encoder_counts,
                float ticks_per_rev, 
                float dist_per_rev, 
                float time_elapsed);
               
float get_ang_speed(long encoder_counts,
                    float ticks_per_rev,
                    float time_elapsed);
#endif
