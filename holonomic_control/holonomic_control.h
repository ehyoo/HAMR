#ifndef HOLO_CON_H
#define HOLO_CON_H

/*
Update holonomic actuator velocties with given current known state. 
Return the computed output in the output pointers
*/
void update_holonomic_state(float _state_x, float _state_y,  float _state_t, 
	                        float* _output_x, float* _output_y, float* _output_t);

/* 
Helper Function for update_holonomic_state()
compute the jacobian with the given inputs*/
void compute_ramsis_jacobian(float xdot, float ydot, float tdot);

void set_setpoints(float x, float y, float z);

void set_max_linear_acceleration(float a);

void set_max_angular_acceleration(float a);


#endif