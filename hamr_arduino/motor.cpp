
#include "motor.h"
#include "Arduino.h"

#define MAX_ROT_SPEED 270.0

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir) {
  digitalWrite(pin_driver_inA, !dir);
  digitalWrite(pin_driver_inB, dir);
}

// Sets direction and Arduino PWM to desired speed
void set_speed(PID_Vars* pid, 
               float speed_req, 
               float speed_act, 
               float t_elapsed, 
               int* pwm_val,
               int pin_driver_inA, 
               int pin_driver_inB, 
               int pin_pwm) {

  float pid_pwm = pid->update_pid(speed_req, speed_act, t_elapsed);
  *pwm_val = round(constrain(pid_pwm * 255.0, -255, 255));

  if (*pwm_val < 0) {
    // reverse direction
    set_direction(pin_driver_inA, pin_driver_inB, DIR_BACKWARD);
  } else {
    // forward direction
    set_direction(pin_driver_inA, pin_driver_inB, DIR_FORWARD);
  }

  analogWrite(pin_pwm, abs(*pwm_val));
}

void init_servo(Servo* motor, int pwm_pin) {
  motor->attach(pwm_pin);
}

void set_servo_speed(Servo* motor, 
                     PID_Vars* pid, 
                     float ang_speed_req,
                     float ang_speed_act,
                     float t_elapsed) {
  float ang_speed = pid->update_pid(ang_speed_req, ang_speed_act, t_elapsed);
  int pwm_val = round(map(ang_speed, -MAX_ROT_SPEED, MAX_ROT_SPEED, 0, 180));
  motor->write(pwm_val);
}

/*
 * Calculates speed from encoder counts over time
 * 
 * curr_count, prev_count: current and previous encoder counts
 * ticks_per_rev: number of encoder counts per revolution
 * dist_per_rev: circumference of wheel
 * time_elapsed: time in ms
 */
float get_speed(long encoder_counts,
                float ticks_per_rev, 
                float dist_per_rev, 
                float time_elapsed) {
  // Calculating the speed using encoder count
  /*Serial.print(encoder_counts);
  Serial.print(time_elapsed,2);
  Serial.print(ticks_per_rev,2);
  Serial.print(dist_per_rev,2);*/

  return ((((float) encoder_counts) / ticks_per_rev) * dist_per_rev) / (time_elapsed / 1000.0);
}

/*
 * Calculates angular speed from encoder counts over time
 * 
 * encoder_counts: encoder counts over the time period time_elapsed
 * ticks_per_rev: number of encoder counts per revolution
 * time_elapsed: time in ms
 */
float get_ang_speed(long encoder_counts,
                    float ticks_per_rev,
                    float time_elapsed) {
  return 360.0 * (((float) encoder_counts) / ticks_per_rev) / (time_elapsed / 1000.0);
}
