#include "localize.h"
#include "pid.h"
#include "Arduino.h"

/*
void trajectory(int set_angle, float set_velocity, location* loc) {
  float rho = sqrt((loc->dx)^2 + (loc->dy)^2);
  float alpha = -(loc->theta) + atan2(loc->dy, loc->dx);
}
*/


/*
 * dtheta_req: angular velocity setpoint
 * dtheta_act: measured angular velocity from encoders
 * dtheta_cmd: value given to control law to change angle
 */
void angle_control(PID_Vars* pid, float dtheta_req, float dtheta_act, float* dtheta_cmd, float speed_req,
                   float* M1_speed, float* M2_speed, float wheel_dist, float wheel_rad, float t) {  

//  dtheta_cmd = dd_ctrl.update_pid(dtheta_req * PI/180.0, dtheta_act, t);
  float pid_output = pid->update_pid(dtheta_req * PI, dtheta_act, t); // USE FOR CONTROLLER INPUT: maps [-1,1]->[-PI,PI] rads
  *dtheta_cmd += pid_output;
  
//  Serial.print("dtheta_req: ");
//  Serial.print(dtheta_req * 180);
//  Serial.print(", ");
//  Serial.print("dtheta_act: ");
//  Serial.print(dtheta_act * 180/PI);
//  Serial.print(", ");
//  Serial.print("dtheta_cmd: ");
//  Serial.print(*dtheta_cmd * 180/PI);
//  Serial.print("\n");

  // Control law
  // Determine speeds for each indiv motor to achieve angle at speed
  float ang_speed = (wheel_dist/2.0) * (*dtheta_cmd);
  if (dtheta_req == 0.0) {
    // Remove any turning if not input in
    *M1_speed = speed_req;
    *M2_speed = speed_req;
  } else {
    *M1_speed = speed_req - ang_speed;
    *M2_speed = speed_req + ang_speed;
  }
        
//  Serial.print("speed_req: ");
//  Serial.print(speed_req);
//  Serial.print(" +/- ");
//  Serial.print((wheel_dist/2.0) * 0.5 * (dtheta_act + dtheta_cmd));
//  Serial.print(", ");
//  Serial.print("M1_speed: ");
//  Serial.print(*M1_speed);
//  Serial.print(", ");
//  Serial.print("M2_speed: ");
//  Serial.print(*M2_speed);
//  Serial.print("\n\n");
}

