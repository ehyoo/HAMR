#include "localize.h"
#include "pid.h"
#include "Arduino.h"

/*
void trajectory(int set_angle, float set_velocity, location* loc) {
  float rho = sqrt((loc->dx)^2 + (loc->dy)^2);
  float alpha = -(loc->theta) + atan2(loc->dy, loc->dx);
}
*/

PID_Vars dd_ctrl(1.0, 0.0, 0.0);

/*
 * dtheta_req: angular velocity setpoint
 * dtheta_act: measured angular velocity from encoders
 * dtheta_cmd: value given to control law to change angle
 */
void angle_control(float dtheta_req, float dtheta_act, float dtheta_cmd, float speed_req,
                   float* M1_speed, float* M2_speed, float wheel_dist, float wheel_rad, float t) {  
  
  dtheta_cmd = dd_ctrl.update_pid(dtheta_req, dtheta_act, t);
  Serial.print("dtheta_req: ");
  Serial.print(dtheta_req);
  Serial.print(", ");
  Serial.print("dtheta_act: ");
  Serial.print(dtheta_act);
  Serial.print(", ");
  Serial.print("dtheta_cmd: ");
  Serial.print(dtheta_cmd);
  Serial.print("\n");
  // Control law
  // Determine speeds for each indiv motor to achieve angle at speed
  *M1_speed = speed_req - (wheel_dist/wheel_rad) * 0.5 * dtheta_cmd;
  *M2_speed = speed_req + (wheel_dist/wheel_rad) * 0.5 * dtheta_cmd;
}

