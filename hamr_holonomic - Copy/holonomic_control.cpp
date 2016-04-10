#include "holonomic_control.h"
#include "math.h"
#include "Arduino.h"

//remember to define these correctly
#define DIM_A 1
#define DIM_B 1
#define DIM_R 1

// this value will determine how quickly to change the setpoint
float max_linear_acceleration = .1;
float max_angular_acceleration = .1;


// desired user velocities
float setpoint_x = 0;  float setpoint_y = 0;  float setpoint_r = 0; 


/* last known state (updated by update_holonomic_state()) */
float state_xdot = 0; float state_ydot = 0; float state_t = 0;

/*
Actual velocities input into Jacobian. 
These depend on the set_point and max_linear_acceleration
*/
float input_x = 0;  float input_y = 0;  float input_r = 0;

/*
last set of velocities output from jacobian 
*/
float output_m1 = 0; float output_m2 = 0; float output_mt = 0;

/* 
Update holonomic actuator velocities with given current known state. 
Return the computed output in the output pointers
*/
void update_holonomic_state(float _state_xdot, float _state_ydot,  float _state_t, 
	                        float* _output_m1, float* _output_m2, float* _output_mt){
	state_xdot = _state_xdot;
	state_ydot = _state_ydot;
	state_t = _state_t;

	input_x = setpoint_x;
	input_y = setpoint_y;
	input_r = setpoint_r;

	// compute jacobian based on input velocities. store outputs in the ouput_* variables
	compute_ramsis_jacobian(input_x, input_y, input_r, state_t);

	* _output_m1 = output_m1;
	* _output_m2 = output_m2;
	* _output_mt = output_mt;
}

/* 
Helper Function for update_holonomic_state()
compute the jacobian with the given inputs*/
void compute_ramsis_jacobian(float xdot, float ydot, float tdot, float t){
	float sint, cost;
	float b_s, b_c, a_s, a_c; //intermediate calculations
	float m1, m2, mt;

	sint = sin(t);
	cost = cos(t);

	b_s = DIM_B * sint;
	b_c = DIM_B * cost;
	a_s = DIM_A * sint;
	a_c = DIM_A * cost;

	// jacobiano
	output_m1 = (xdot*(-b_s - a_c) + ydot*(b_c - a_s)) / (DIM_R*DIM_B);
	output_m2 = (xdot*(-b_s + a_c) + ydot*(b_c + a_s)) / (DIM_R*DIM_B);
	output_mt = (-xdot * cost - ydot * sint) / DIM_B - tdot;
}

void set_setpoints(float xdot, float ydot, float rdot){
	setpoint_x = xdot;
	setpoint_y = ydot;
	setpoint_r = rdot;
}

void set_max_linear_acceleration(float a){
	max_linear_acceleration = a;
}

void set_max_angular_acceleration(float a){
	max_angular_acceleration = a;
}
