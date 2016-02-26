#include "pid.h"

float update_pid(PID_Vars* pid_vars, int* command, float targetValue, float currentValue) {

  float Kp = pid_vars->Kp;
  float Ki = pid_vars->Ki;
  float Kd = pid_vars->Kd;

  float error_acc_limit = 1.0;
  
  float pidTerm = 0.0;   
  float error = 0.0;        
 
  error = targetValue - currentValue; 
  pid_vars->error_acc += error * (LOOPTIME/1000.0);

  pidTerm = (Kp * error) + (Kd * (error - pid_vars->error_prev) / (LOOPTIME/1000.0)) + (Ki * (pid_vars->error_acc));      

  Serial.print("P: ");
  Serial.print((Kp * error),3);
  Serial.print(", D: ");
  Serial.print((Kd * (error - pid_vars->error_prev) / (LOOPTIME/1000.0)),3);
  Serial.print(", I: ");
  Serial.print((Ki * (pid_vars->error_acc)),3);
  Serial.print("\n");


  // Anti integrator windup using clamping
  pid_vars->error_acc = constrain((pid_vars->error_acc), -1*error_acc_limit, error_acc_limit);
  
  pid_vars->error_prev = error; // update error
  
  //*command = constrain(round(*command + pidTerm), -255, 255);
  *command = constrain(round(pidTerm) * 2.55, -255, 255);
}
