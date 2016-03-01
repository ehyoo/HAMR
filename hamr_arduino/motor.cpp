#include "motor.h"
#include "Arduino.h"

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir) {
  digitalWrite(pin_driver_inA, !dir);
  digitalWrite(pin_driver_inB, dir);
}

// Sets direction and Arduino PWM from controls-derived PWM value
void set_speed(int pwm_val, int pin_driver_inA, int pin_driver_inB, int pin_pwm) {
  if (pwm_val < 0) {
    // reverse direction
    set_direction(pin_driver_inA, pin_driver_inB, DIR_BACKWARD);
  } else {
    // forward direction
    set_direction(pin_driver_inA, pin_driver_inB, DIR_FORWARD);
  }

  analogWrite(pin_pwm, abs(pwm_val));
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
