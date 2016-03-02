#ifndef Pid_h
#define Pid_h

typedef struct PID_Vars {
  float Kp;
  float Ki;
  float Kd;
  float error_acc;
  float error_prev; 
  PID_Vars(float Kp_in, float Ki_in, float Kd_in) {
    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;
    error_acc = 0.0;
    error_prev = 0.0;
  }
};

float update_pid(PID_Vars* pid_vars, int* command, float targetValue, float currentValue, float time_elapsed);

#endif
