#include "motor.h"
#include "Arduino.h"





void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir) {
  digitalWrite(pin_driver_inA, !dir);
  digitalWrite(pin_driver_inB, dir);
}

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

void measure_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count) { // calculate speed
  // Calculating the speed using encoder count
 // int count_diff = curr_count - prev_count;
 // float distance = ((float) count_diff / TICKS_PER_REV)
  //*speed_act = (((float) (curr_count - prev_count) / TICKS_PER_REV) * DIST_PER_REV) / ((float) LOOPTIME / 1000.0);
  *speed_act = (((float) (*curr_count - *prev_count) / 1632.0) * DIST_PER_REV) / (LOOPTIME / 1000.0);
  //Serial.println(round(100* *speed_act));

   // Reset counts
  *prev_count = 0;
  *curr_count = 0;
}

void measure_rot_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count) { // calculate speed
  *speed_act = 360.0 * ((*curr_count - *prev_count) / (float) TICKS_PER_REV_turret);
  
  // Reset counts
  *prev_count = 0;
  *curr_count = 0;
}
