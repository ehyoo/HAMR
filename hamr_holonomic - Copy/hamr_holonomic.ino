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
long next_sensor_time = millis();
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
  // initialize_imu();
  //initialize_imu();
  startMilli = millis();
  delay(1000); 
}

/*************
 * MAIN LOOP *
 *************/
void loop() {

  if((millis()-lastMilli) >= LOOPTIME) {
    //serial communication
    read_serial();
    send_serial();
    
    t_elapsed = (float) (millis() - lastMilli);
    lastMilli = millis();
    
    sense_motors(); // read encoders


    // DIFFERENTIAL DRIVE CONTROL
    int use_dd_control = 0;
    if (use_dd_control == 0) {
      // PID velocity control, same input to both motors
      desired_m1_v = desired_dd_v;
      desired_m2_v = desired_dd_v;
    } else if (use_dd_control == 1) {
      // Differential drive control
      angle_control(&dd_ctrl, desired_dd_r, hamr_loc.w, &dtheta_cmd, desired_dd_v, &desired_m1_v, &desired_m2_v, WHEEL_DIST, WHEEL_RADIUS, t_elapsed);
    } else {
      // use indiv setpoints
      desired_m1_v = (desired_dd_v - (WHEEL_DIST/2.0) * PI/2.0);
      desired_m2_v = (desired_dd_v + (WHEEL_DIST/2.0) * PI/2.0);
    }


    // HOLONOMIC CONTROL
    int use_holonomic_control = 1;
    // note sensed_m1_v and sensed_m2_v should have no effect atm
    if (use_holonomic_control){
      set_setpoints(desired_h_x, desired_h_y, desired_h_r);
      update_holonomic_state(sensed_m1_v, sensed_m2_v, -hamr_loc.theta, &desired_m1_v, &desired_m2_v, &desired_mt_v);
      // desired_m1_v = .3;
      // desired_m2_v = .6;
    }

    //M1 is the RIGHT motor and is forward facing caster wheels
    //M2 is left and so on

    // SerialUSB.print("the" );
    // SerialUSB.print(hamr_loc.theta);
    // SerialUSB.print(" dm1 ");
    // SerialUSB.print(desired_m1_v);
    // SerialUSB.print(" dm2 ");
    // SerialUSB.println(desired_m2_v);

    // set speeds on motors      
    set_speed(&pid_vars_M1,
              desired_m1_v,
              sensed_m1_v, 
              &m1_v_cmd,
              t_elapsed, 
              &PWM_M1,
              PIN_M1_DRIVER_INA, 
              PIN_M1_DRIVER_INB, 
              PIN_M1_DRIVER_PWM);
    set_speed(&pid_vars_M2,
              desired_m2_v,
              sensed_m2_v, 
              &m2_v_cmd,
              t_elapsed, 
              &PWM_M2,
              PIN_M2_DRIVER_INA, 
              PIN_M2_DRIVER_INB, 
              PIN_M2_DRIVER_PWM);
    //set_servo_speed(&M3,pid_vars_M3, desired_h_r, speed_act_turret, t_elapsed);

//    if (hamr_loc.theta > 109.0*PI/180.0) {
//        analogWrite(PIN_M1_DRIVER_PWM,0);
//        analogWrite(PIN_M2_DRIVER_PWM,0);
//    }
    

    // uncomment this to read IMU
    // if(next_sensor_time < millis()){
    //   compute_imu(SENSOR_LOOPTIME);
    //   // print_raw_imu();
    //   // SerialUSB.println("");
    //   hamr_angle = get_current_angle();
    //   SerialUSB.println(hamr_angle);
    // }
    // next_sensor_time = millis() + SENSOR_LOOPTIME * 1000;

  }

}

void init_serial(){
  SerialUSB.begin(250000);
  SerialUSB.println("Arduino Ready\n"); // needs to be sent to detect that arduino has initialized
  SerialUSB.setTimeout(0);              // required to speed up serial reading 
}

/* read a byte from SerialUSB. Perform appropriate action based on byte*/
void read_serial(){
  String str;
  float temp;
  float* sig_var;
  char buffer[1];

  buffer[0] = SIG_UNINITIALIZED; 
  if(SerialUSB.available()){
    SerialUSB.readBytes(buffer, 1);
    SerialUSB.print(buffer[0]);

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

    if(SerialUSB.available()){
      *sig_var = SerialUSB.readString().toFloat();
      SerialUSB.print(" "); SerialUSB.print(*sig_var);

      // uncomment below to test if correct signals are received
      // SerialUSB.print("signal received:"); SerialUSB.println(buffer[0]);
      // SerialUSB.print("value received:"); SerialUSB.println(*sig_var);
    }

  }
}

/* send relevant data through serial */
void send_serial(){
     if(send_data and Serial){
       SerialUSB.println(SIG_START_STRING);
       delayMicroseconds(500);
       SerialUSB.println(millis() - startMilli); // total time
       SerialUSB.println(sensed_m1_v, 4);
       SerialUSB.println(sensed_m2_v, 4);
       SerialUSB.println(desired_dd_v);
       SerialUSB.println(hamr_loc.w * 180.0 / PI, 4);
   }
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
 *  ENCODER COUNTING INTERRUPT ROUTINES
 *  Motor index: 1 (left DD), 2 (right DD), 3 (turret)
 */

void rencoderA_M1()  {
  int encoderA_pin = PIN_M1_ENCODER_OUTA;
  int encoderB_pin = PIN_M1_ENCODER_OUTB;
  // Record rising or falling edge from encoder
  interrupt_M1_A = digitalRead(encoderA_pin);

  if (interrupt_M1_A != interrupt_M1_B) curr_count_M1--; // encoderA changed before encoderB -> forward
  else                                  curr_count_M1++; // encoderB changed before encoderA -> reverse
}

void rencoderB_M1()  {
  int encoderA_pin = PIN_M1_ENCODER_OUTA;
  int encoderB_pin = PIN_M1_ENCODER_OUTB;
  // Record rising or falling edge from encoder
  interrupt_M1_B = digitalRead(encoderB_pin);

  if (interrupt_M1_A != interrupt_M1_B) curr_count_M1++; // encoderB changed before encoderA -> reverse
  else                                  curr_count_M1--; // encoderA changed before encoderB -> forward
}

void rencoderA_M2()  {
  int encoderA_pin = PIN_M2_ENCODER_OUTA;
  int encoderB_pin = PIN_M2_ENCODER_OUTB;
  // Record rising or falling edge from encoder
  interrupt_M2_A = digitalRead(encoderA_pin);

  //curr_count_M2++;
  if (interrupt_M2_A != interrupt_M2_B) curr_count_M2--; // encoderA changed before encoderB -> forward
  else                                  curr_count_M2++; // encoderB changed before encoderA -> reverse
}

void rencoderB_M2()  {
  int encoderA_pin = PIN_M2_ENCODER_OUTA;
  int encoderB_pin = PIN_M2_ENCODER_OUTB;
  // Record rising or falling edge from encoder
  interrupt_M2_B = digitalRead(encoderB_pin);

  //curr_count_M2++;
  if (interrupt_M2_A != interrupt_M2_B) curr_count_M2++; // encoderB changed before encoderA -> reverse
  else                                  curr_count_M2--; // encoderA changed before encoderB -> forward
}


void init_actuators(){
    // Set up turret motor
  init_servo(&M3, PIN_M3_DRIVER_PWM);

  // Set DD motor driver pins as outputs
  pinMode(PIN_DD_EN, OUTPUT);
  pinMode(PIN_M1_DRIVER_INA, OUTPUT);
  pinMode(PIN_M1_DRIVER_INB, OUTPUT);  
  pinMode(PIN_M2_DRIVER_INA, OUTPUT);
  pinMode(PIN_M2_DRIVER_INB, OUTPUT);
  // Set turret driver pins as outputs
  pinMode(PIN_M3_DRIVER_INA, OUTPUT);
  pinMode(PIN_M3_DRIVER_INB, OUTPUT);
  // Set encoder pins as inputs
  pinMode(PIN_M1_ENCODER_OUTA, INPUT);
  pinMode(PIN_M1_ENCODER_OUTB, INPUT);
  pinMode(PIN_M2_ENCODER_OUTA, INPUT);
  pinMode(PIN_M2_ENCODER_OUTB, INPUT);
  pinMode(PIN_M3_ENCODER_OUTA, INPUT);
  pinMode(PIN_M3_ENCODER_OUTB, INPUT);

  //set pin no 2 as interrupt pin (INTERRUPT 1) 
  //digitalWrite(PIN_M1_ENCODER_OUTA, HIGH);                      
  //digitalWrite(PIN_M1_ENCODER_OUTB, HIGH);
  attachInterrupt(PIN_M1_ENCODER_OUTA, rencoderA_M1, CHANGE);  
  attachInterrupt(PIN_M1_ENCODER_OUTB, rencoderB_M1, CHANGE);
  attachInterrupt(PIN_M2_ENCODER_OUTA, rencoderA_M2, CHANGE);  
  attachInterrupt(PIN_M2_ENCODER_OUTB, rencoderB_M2, CHANGE);  
  // attachInterrupt(PIN_M3_ENCODER_OUTA, rencoderA_M3, CHANGE);  
  // attachInterrupt(PIN_M3_ENCODER_OUTB, rencoderB_M3, CHANGE);  

  digitalWrite(PIN_DD_EN, HIGH);
  // Set Motors as forward
  digitalWrite(PIN_M1_DRIVER_INA, LOW);
  digitalWrite(PIN_M1_DRIVER_INB, HIGH);
  digitalWrite(PIN_M2_DRIVER_INA, LOW);
  digitalWrite(PIN_M2_DRIVER_INB, HIGH);
  digitalWrite(PIN_M3_DRIVER_INA, HIGH);
  digitalWrite(PIN_M3_DRIVER_INB, LOW);

  // Initialize PWMs to 0
  analogWrite(PIN_M1_DRIVER_PWM, 0);
  analogWrite(PIN_M2_DRIVER_PWM, 0);
  set_servo_speed(&M3, 0, 0.0, 0.0, 1.0);
}

void print1(){
  SerialUSB.println(sensed_m3_v, 4);

   SerialUSB.print(sensed_m1_v, 4);
   SerialUSB.print(" (");
   SerialUSB.print(PWM_M1);
   SerialUSB.print("), ");
   SerialUSB.print(sensed_m2_v, 4);
   SerialUSB.print(" (");
   SerialUSB.print(PWM_M2);
   SerialUSB.print(")\n");

   SerialUSB.print(curr_count_M1);
   SerialUSB.print(" ");
   SerialUSB.print(curr_count_M2);
   SerialUSB.print("\n");
}