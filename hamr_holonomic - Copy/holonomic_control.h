#ifndef HOLO_CON_H
#define HOLO_CON_H

/*
Update holonomic actuator velocties with given current known state. 
Return the computed output in the output pointers
*/
void update_holonomic_state(float _state_xdot, float _state_ydot,  float _state_tdot, 
	                        float* _output_m1, float* _output_m2, float* _output_mt);

/* 
Helper Function for update_holonomic_state()
compute the jacobian with the given inputs*/
void compute_ramsis_jacobian(float xdot, float ydot, float tdot, float t);

void set_setpoints(float xdot, float ydot, float rdot);

void set_max_linear_acceleration(float a);

void set_max_angular_acceleration(float a);


#endif