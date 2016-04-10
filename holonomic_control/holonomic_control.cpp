#include "holonomic_control.h"
#include "math.h"

//remember to define these correctly
#define DIM_A 1
#define DIM_B 1
#define DIM_R 1

// this value will determine how quickly to change the setpoint
float max_linear_acceleration = .1;
float max_angular_acceleration = .1;


// desired user velocities
float setpoint_x = 0;  float setpoint_y = 0;  float setpoint_t = 0; 

/*
Actual velocities input into Jacobian. 
These depend on the set_point and max_linear_acceleration
*/
float input_x = 0;  float input_y = 0;  float input_t = 0;

/*
last set of velocities output from jacobian 
*/
float output_x = 0; float output_y = 0; float output_t = 0;

/* last known state */
float state_x = 0; float state_y = 0; float state_t = 0;

/* 
Update holonomic actuator velocities with given current known state. 
Return the computed output in the output pointers
*/
void update_holonomic_state(float _state_x, float _state_y,  float _state_t, 
	                        float* _output_x, float* _output_y, float* _output_t){
	state_x = _state_x;
	state_y = _state_y;
	state_t = _state_t;

	//increment input
	//TODO: slowing down is easier than speeding up, so max_acceleation should be variable
	input_x = min(setpoint_x - input_x, max_linear_acceleration);
	input_y = min(setpoint_y - input_y, max_linear_acceleration);
	input_t = min(setpoint_t - input_t, max_angular_acceleration);

	compute_ramsis_jacobian(input_x, input_y, input_z);

	_output_x->output_x;
	_output_y->output_y;s
	_output_t->output_t;
}

/* 
Helper Function for update_holonomic_state()
compute the jacobian with the given inputs*/
void compute_ramsis_jacobian(float xdot, float ydot, float tdot){
	float sint, cost;
	float b_s; b_c; a_s; a_c; //intermediate calculations
	float m1, m2, mt;

	sint = sin(t);
	cost = cos(t);

	b_s = DIM_B * sint;
	b_c = DIM_B * cost;
	a_s = DIM_A * sint;
	a_c = DIM_A * cost;

	// jacobian
	output_x = (xdot*(-b_s - a_c) + ydot*(b_c - a_s)) / (DIM_R*DIM_B);
	output_y = (xdot*(-b_s + a_c) + ydot*(b_c + a_s)) / (DIM_R*DIM_B);
	output_t = (-xdot * cost - ydot * sint) / DIM_B - tdot;
}

void set_setpoints(float x, float y, float z){
	setpoint_x = x;
	setpoint_y = y;
	setpoint_t = z;
}

void set_max_linear_acceleration(float a){
	max_linear_acceleration = a;
}

void set_max_angular_acceleration(float a){
	max_angular_acceleration = a;
}
