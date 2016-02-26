#ifndef Motor_h
#define Motor_h

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir) {

void set_speed(int pwm_val, int pin_driver_inA, int pin_driver_inB, int pin_pwm) {

void measure_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count) { // calculate speed

void measure_rot_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count) { // calculate speed

#endif
