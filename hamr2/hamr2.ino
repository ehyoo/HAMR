#include "pid.h"
#include "motor.h"
#include "localize.h"
#include "hamr_imu.h"
#include "dd_control.h"
#include "constants.h"
#include "holonomic_control.h"

/* -------------------------------------------------------*/
/* These following values are modifiable through serial communication or determined by a control output */
int send_data = 0;  // whether the arduino should send data 

/* DESIRED VALUES */
// holonomic velocities 
float desired_h_x = 0; float desired_h_y = 0; float desired_h_r = 0;

// differential drive velocities
float desired_dd_v = 0.0; // Diff Drive (m/s)
float desired_dd_r = 0.0; // desired angular velocity for Diff Drive: set between [-1,1] by controller, mapped to [-90,90] degrees in code
// float speed_req_turret = 0.0; // Turret (rad/s)?

// motor velocities
float desired_m1_v = 0; float desired_m2_v = 0; float desired_mt_v = 0;

// motor PWMs
int PWM_M1 = 0; 
int PWM_M2 = 0; 
int PWM_M3 = 0; 
float m1_v_cmd = 0.0;
float m2_v_cmd = 0.0;

/* CONTROL PARAMETERS */
PID_Vars pid_vars_M1(0.15, 0.0, 0.04);
PID_Vars pid_vars_M2(0.15, 0.0, 0.04);
PID_Vars pid_vars_M3(0.01, 0.0, 0.0);
PID_Vars dd_ctrl(0.1, 0.0, 0.0);
// PID_Vars pid_vars_dd_v(1.0, 0.0, 0.0);
// PID_Vars pid_vars_dd_r(1.0, 0.0, 0.0);
PID_Vars pid_vars_h_x(0.1, 0.0, 0.0);
PID_Vars pid_vars_h_y(0.1, 0.0, 0.0);
PID_Vars pid_vars_h_r(0.1, 0.0, 0.0);
/* -------------------------------------------------------*/
/* -------------------------------------------------------*/

/* -------------------------------------------------------*/
/* SENSORS */
// encoders
// Encoder counting interrupt functions
void rencoderA_M1(); void rencoderB_M1();
void rencoderA_M2(); void rencoderB_M2();
// void rencoderA_M3(); void rencoderB_M3();

/* encoder counters */
volatile long curr_count_M1 = 0; volatile long prev_count_M1 = 0; 
volatile long curr_count_M2 = 0; volatile long prev_count_M2 = 0; 
volatile long curr_count_M3 = 0; volatile long prev_count_M3 = 0; 

/* encoder output state */
int interrupt_M1_A = 0; int interrupt_M1_B = 0;
int interrupt_M2_A = 0; int interrupt_M2_B = 0;
int interrupt_M3_A = 0; int interrupt_M3_B = 0;

/* measured velocities */
float sensed_m1_v = 0.0; 
float sensed_m2_v = 0.0; 
float sensed_m3_v = 0.0;   

/* velocity commands */
float dtheta_cmd = 0.0;

/* IMU settings */
const float SENSOR_LOOPTIME = .01;
unsigned long next_sensor_time = millis();
unsigned long prev_micros = 0;
float hamr_angle = 0;

/* -------------------------------------------------------*/
/* -------------------------------------------------------*/

Servo M3;

// timing: main loop
unsigned long startMilli;
unsigned long lastMilli = 0;   
float t_elapsed;                

// dd localization
location hamr_loc;

void setup() {
  init_serial();    // initialize serial communication
  init_actuators(); // initialiaze motors and servos
  delay(100);

  initialize_imu();
  startMilli = millis();
  delay(1000); 
}

/*************
 * MAIN LOOP *
 *************/
  void loop() {
  // uncomment this when not testing
  test_motors();

  // while(1){
  //   // last timing was between 900 and 1200 microseconds. the range seems high...
  //   //uncomment the first and last line in while loop to test timing
  //   // unsigned long start_time = micros();

  //   if((millis()-lastMilli) >= LOOPTIME) {
  //     //serial communication
  //     read_serial();
  //     send_serial();
      
  //     t_elapsed = (float) (millis() - lastMilli);
  //     lastMilli = millis();
      
  //     sense_motors(); // read encoders

  //     // DIFFERENTIAL DRIVE CONTROL
  //     int use_dd_control = 0;
  //     if (use_dd_control == 0) {
  //       // PID velocity control, same input to both motors
  //       desired_m1_v = desired_dd_v;
  //       desired_m2_v = desired_dd_v;
  //     } else if (use_dd_control == 1) {
  //       // Differential drive control
  //       angle_control(&dd_ctrl, desired_dd_r, hamr_loc.w, &dtheta_cmd, desired_dd_v, &desired_m1_v, &desired_m2_v, WHEEL_DIST, WHEEL_RADIUS, t_elapsed);
  //     } else {
  //       // use indiv setpoints
  //       desired_m1_v = (desired_dd_v - (WHEEL_DIST/2.0) * PI/2.0);
  //       desired_m2_v = (desired_dd_v + (WHEEL_DIST/2.0) * PI/2.0);
  //     }

  //     //M1 is the RIGHT motor and is forward facing caster wheels
  //     //M2 is LEFT

  //     // HOLONOMIC CONTROL
  //     int use_holonomic_control = 1;
  //     if (use_holonomic_control){
  //       set_setpoints(desired_h_x, desired_h_y, desired_h_r);
  //       update_holonomic_state(hamr_angle, &desired_m1_v, &desired_m2_v, &desired_mt_v);
  //     }

  //     // Serial.print("the" );
  //     // Serial.print(hamr_loc.theta);
  //     // Serial.print(" dm1 ");
  //     // Serial.print(desired_m1_v);
  //     // Serial.print(" dm2 ");
  //     // Serial.println(desired_m2_v);

  //     // set speeds on motors      
  //     set_speed(&pid_vars_M1,
  //               desired_m1_v,
  //               sensed_m1_v, 
  //               &m1_v_cmd,
  //               t_elapsed, 
  //               &PWM_M1,
  //               PIN_M1_DRIVER_INA, 
  //               PIN_M1_DRIVER_PWM);
  //     set_speed(&pid_vars_M2,
  //               desired_m2_v,
  //               sensed_m2_v, 
  //               &m2_v_cmd,
  //               t_elapsed, 
  //               &PWM_M2,
  //               PIN_M2_DRIVER_INA,
  //               PIN_M2_DRIVER_INB,
  //               PIN_M2_DRIVER_PWM);
  //     set_servo_speed(&M3, &pid_vars_M3, hamr_loc.w * 180 * 0.191, 0, 0);
  //   }

  //   if(next_sensor_time < millis() && is_imu_working()){
  //     unsigned long current_micros = micros();

  //     compute_imu((current_micros - prev_micros) / 1000000.0); //update imu with time change

  //     hamr_angle = get_current_angle() * PI / 180;

  //     next_sensor_time = millis() + SENSOR_LOOPTIME;
  //     prev_micros = current_micros;

  //     // potentially combine hamr_loc.theta with imu angle?
  //   } else if(!is_imu_working()) {
  //     hamr_angle = hamr_loc.theta;
  //   }

  //   // unsigned long finish_time = micros();
  //   // Serial.print("total_time: "); Serial.println(finish_time - start_time);
  // }
}

void init_serial(){
  Serial.begin(250000);
  Serial.println("Arduino Ready\n"); // needs to be sent to detect that arduino has initialized
  Serial.setTimeout(0);              // required to speed up serial reading 
}

/* read a byte from Serial. Perform appropriate action based on byte*/
void read_serial(){
  String str;
  float temp;
  float* sig_var;
  char buffer[1];

  buffer[0] = SIG_UNINITIALIZED; 
  if(Serial.available()){
    Serial.readBytes(buffer, 1);
    Serial.print(buffer[0]);

    switch(buffer[0]){
      case SIG_START_LOG:
        send_data = 1;
        break;

      case SIG_STOP_LOG:
        send_data = 0;
        break;

      // holonomic inputs
      case SIG_HOLO_X:
        sig_var = &desired_h_x;
        break;

      case SIG_HOLO_Y:
        sig_var = &desired_h_y;
        break;

      case SIG_HOLO_R:
        sig_var = &desired_h_r;
        break;

      // differential drive inputs
      case SIG_DD_V:
        sig_var = &desired_dd_v;
        break;

      case SIG_DD_R:
        sig_var = &desired_dd_r;
        break;

      // motor velocities
      case SIG_R_MOTOR:
        sig_var = &desired_m1_v;
        break;

      case SIG_L_MOTOR:
        sig_var = &desired_m2_v;
        break;

      case SIG_T_MOTOR:
        sig_var = &desired_mt_v;
        break;

      // right motor PID
      case SIG_R_KP:
        sig_var = &(pid_vars_M1.Kp);
        break;

      case SIG_R_KI:
        sig_var = &(pid_vars_M1.Ki);
        break;

      case SIG_R_KD:
        sig_var = &(pid_vars_M1.Kd);
        break;

      // left motor PID
      case SIG_L_KP:
        sig_var = &(pid_vars_M2.Kp);
        break;

      case SIG_L_KI:
        sig_var = &(pid_vars_M2.Ki);
        break;

      case SIG_L_KD:
        sig_var = &(pid_vars_M2.Kd);
        break;

      // turret motor PID
      case SIG_T_KP:
        sig_var = &(dd_ctrl.Kp);
        break;

      case SIG_T_KI:
        sig_var = &(dd_ctrl.Ki);
        break;

      case SIG_T_KD:
        sig_var = &(dd_ctrl.Kd);
        break;

      // holonomic X PID
      case SIG_HOLO_X_KP:
        sig_var = &(pid_vars_h_x.Kp);
        break;

      case SIG_HOLO_X_KI:
        sig_var = &(pid_vars_h_x.Ki);
        break;

      case SIG_HOLO_X_KD:
        sig_var = &(pid_vars_h_x.Kd);
        break;

      // holonomic Y PID

      case SIG_HOLO_Y_KP:
        sig_var = &(pid_vars_h_y.Kp);
        break;

      case SIG_HOLO_Y_KI:
        sig_var = &(pid_vars_h_y.Ki);
        break;

      case SIG_HOLO_Y_KD:
        sig_var = &(pid_vars_h_y.Kd);
        break;

      // holonomic R PID

      case SIG_HOLO_R_KP:
        sig_var = &(pid_vars_h_r.Kp);
        break;

      case SIG_HOLO_R_KI:
        sig_var = &(pid_vars_h_r.Ki);
        break;

      case SIG_HOLO_R_KD:
        sig_var = &(pid_vars_h_r.Kd);
        break;
    }

    if(Serial.available()){
      *sig_var = Serial.readString().toFloat();
      Serial.print(" "); Serial.print(*sig_var);

      // uncomment below to test if correct signals are received
      // Serial.print("signal received:"); Serial.println(buffer[0]);
      // Serial.print("value received:"); Serial.println(*sig_var);
    }

  }
}

/* send relevant data through serial 
Everything in the if statement takes between 1200 and 1300 microseconds
uncomment first and last line to test timing */
void send_serial(){
  // unsigned long starttime = micros();
   if(send_data and Serial){
       Serial.println(SIG_START_STRING);
       delayMicroseconds(500);
       Serial.println(millis() - startMilli); // total time
       Serial.println(sensed_m1_v, 3);
       Serial.println(sensed_m2_v, 3);
       Serial.println(hamr_angle * 180.0 / PI, 3);
       // Serial.println(hamr_loc.theta * 180.0 / PI, 2);

       Serial.println(desired_h_x, 3);
       Serial.println(desired_h_y, 3);
       Serial.println(hamr_angle * 180.0 / PI, 3);

       Serial.println(desired_h_x, 3);
       Serial.println(desired_h_y, 3);
       Serial.println(desired_h_r, 3);

       // Serial.println(desired_dd_v);
       // Serial.println(desired_dd_r);
   }
   // unsigned long finishtime = micros();
  // Serial.print("total_time: "); Serial.println(finishtime - starttime);
}

void sense_motors(){
  // Count encoder increments since last loop
  long encoder_counts_M1 = curr_count_M1 - prev_count_M1;
  long encoder_counts_M2 = curr_count_M2 - prev_count_M2;
  long encoder_counts_M3 = curr_count_M3 - prev_count_M3;
  prev_count_M1 = curr_count_M1;
  prev_count_M2 = curr_count_M2;
  prev_count_M3 = curr_count_M3;

  // compute speed
  sensed_m1_v = get_speed(encoder_counts_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  sensed_m2_v = get_speed(encoder_counts_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  sensed_m3_v = get_ang_speed(encoder_counts_M3, TICKS_PER_REV_TURRET, t_elapsed);         
  
  hamr_loc.update(sensed_m1_v, sensed_m2_v, WHEEL_DIST, t_elapsed);
}



void init_actuators(){

  // Set DD motor driver pins as outputs
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_SLP_PIN, OUTPUT);
  pinMode(M1_FLT_PIN, INPUT_PULLUP);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_SLP_PIN, OUTPUT);
  pinMode(M2_FLT_PIN, INPUT_PULLUP);
  pinMode(MT_PWM_PIN, OUTPUT);
  pinMode(MT_DIR_PIN, OUTPUT);
  pinMode(MT_SLP_PIN, OUTPUT);
  pinMode(MT_FLT_PIN, INPUT_PULLUP);

  // Set Motors as forward
  digitalWrite(M1_DIR_PIN, LOW);
  digitalWrite(M2_DIR_PIN, LOW);
  digitalWrite(MT_DIR_PIN, LOW);

  // Initialize PWMs to 0
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  analogWrite(MT_PWM_PIN, 0);
}

void print1(){
  Serial.println(sensed_m3_v, 4);

   Serial.print(sensed_m1_v, 4);
   Serial.print(" (");
   Serial.print(PWM_M1);
   Serial.print("), ");
   Serial.print(sensed_m2_v, 4);
   Serial.print(" (");
   Serial.print(PWM_M2);
   Serial.print(")\n");

   Serial.print(curr_count_M1);
   Serial.print(" ");
   Serial.print(curr_count_M2);
   Serial.print("\n");
}


// Testing functions
int increment = 1;
int pwm = 0;
void test_motors(){

  pwm += increment;
  if (pwm > 255){
    increment = -1;
  } else if (pwm < 0){
    increment = 1;
  }

  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  analogWrite(MT_PWM_PIN, 0);
  delay(50);
}
