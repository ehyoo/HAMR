#include "pid.h"
#include "motor.h"
#include "localize.h"
#include "hamr_imu.h"
#include "dd_control.h"
#include "constants.h"


/* IMU settings */
// const float SENSOR_LOOPTIME = .005;
// long next_sensor_time = millis();
// float hamr_angle = 0;


/* These following values are modifiable through serial communication*/
int send_data = 0;  // whether the arduino should send data 

// CONTROL PARAMETER

// PID motors
float r_Kp = 0; float r_Ki = 0; float r_Kd = 0;
float l_Kp = 0; float l_Ki = 0; float l_Kd = 0;
float t_Kp = 0; float t_Ki = 0; float t_Kd = 0;

// Differential Drive
float dd_v_Kp = 0; float dd_v_Ki = 0; float dd_v_Kd = 0; // DD velocity
float dd_r_Kp = 0; float dd_r_Ki = 0; float dd_r_Kd = 0; // DD rotation

// Holonomic
float x_Kp = 0; float x_Ki = 0; float x_Kd = 0;
float y_Kp = 0; float y_Ki = 0; float y_Kd = 0;
// float t_Kp = 0; float t_Ki = 0; float t_Kd = 0;

// DESIRED VALUES        
float speed_req_ddrive = 0.0; // Diff Drive (m/s)
float speed_req_ang = 0.0;    // desired angular velocity for Diff Drive: set between [-1,1] by controller, mapped to [-90,90] degrees in code
float speed_req_turret = 0.0; // Turret (deg/s)

// motor velocities
float right_motor = 0; float left_motor = 0; float turret_motor = 0; 

/* -------------------------------------------------------*/
/* -------------------------------------------------------*/

float startMilli;

Servo M3;

unsigned long lastMilli = 0;                   
unsigned long lastMilliPrint = 0; 

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
float speed_act_M1 = 0.0; 
float speed_act_M2 = 0.0; 
float speed_act_M3 = 0.0; 

/* desired motor velocities */
float speed_req_M1 = 0.0;
float speed_req_M2 = 0.0;

/* PWM input to motors */
int PWM_M1 = 0; 
int PWM_M2 = 0; 
int PWM_M3 = 0;         

float dtheta_cmd = 0.0;

/* PID LOOP VARIABLES */
//PID_Vars pid_vars_M1(2.09, 0.79, 0.014);
//PID_Vars pid_vars_M2(2.09, 0.79, 0.014);
PID_Vars pid_vars_M1(1.0, 0.0, 0.0);
PID_Vars pid_vars_M2(1.0, 0.0, 0.0);
PID_Vars pid_vars_M3(1.0, 0.0, 0.0);

location hamr_loc;

void setup() {
  init_serial();    // initialize serial communication
  init_actuators(); // initialiaze motors and servos
  //initialize_imu();
  startMilli = millis();
}

int m1_v = 0;
int m2_v = 0;
/*************
 * MAIN LOOP *
 *************/
void loop() {
  //set_speed(255, PIN_M1_DRIVER_INA, PIN_M1_DRIVER_INB, PIN_M1_DRIVER_PWM);
  //set_speed(255, PIN_M2_DRIVER_INA, PIN_M2_DRIVER_INB, PIN_M2_DRIVER_PWM);
  //while(1){};

  if((millis()-lastMilli) >= LOOPTIME) {

    //serial communication
    read_serial();
    send_serial();
    
    if (millis() - startMilli >= 0) { // conditional used to delay start of control loop for testing
      float t_elapsed = (float) (millis() - lastMilli);
      lastMilli = millis();

      // Count encoder increments since last loop
      long encoder_counts_M1 = curr_count_M1 - prev_count_M1;
      long encoder_counts_M2 = curr_count_M2 - prev_count_M2;
      long encoder_counts_M3 = curr_count_M3 - prev_count_M3;
      prev_count_M1 = curr_count_M1;
      prev_count_M2 = curr_count_M2;
      prev_count_M3 = curr_count_M3;

      // compute speed
      speed_act_M1 = get_speed(encoder_counts_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
      speed_act_M2 = get_speed(encoder_counts_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
      speed_act_M3 = get_ang_speed(encoder_counts_M3, TICKS_PER_REV_TURRET, t_elapsed);         
      
      hamr_loc.update(speed_act_M1, speed_act_M2, WHEEL_DIST, t_elapsed);

     //Serial.println((millis() - startMilli)/1000.0);

//      if (abs(hamr_loc.theta) > PI) {
//        speed_req_M1 = 0.0;
//        speed_req_M2 = 0.0;
//      }


      // get desired speed for diff drive
      int use_dd_control = 1;
      if (use_dd_control == 0) {
        speed_req_M1 = speed_req_ddrive;
        speed_req_M2 = speed_req_ddrive;
      } else if (use_dd_control == 1) {
        angle_control(speed_req_ang, hamr_loc.w, dtheta_cmd, speed_req_ddrive, &speed_req_M1, &speed_req_M2, WHEEL_DIST, WHEEL_RADIUS, t_elapsed);
      } else { 
        // use indiv setpoints
        speed_req_M1 = (speed_req_ddrive - (WHEEL_DIST/2.0) * PI/2.0);
        speed_req_M2 = (speed_req_ddrive + (WHEEL_DIST/2.0) * PI/2.0);
      }

      // set speeds on motors      
      set_speed(&pid_vars_M1,
                speed_req_M1,
                speed_act_M1, 
                t_elapsed, 
                &PWM_M1,
                PIN_M1_DRIVER_INA, 
                PIN_M1_DRIVER_INB, 
                PIN_M1_DRIVER_PWM);
      set_speed(&pid_vars_M2,
                speed_req_M2,
                speed_act_M2, 
                t_elapsed, 
                &PWM_M2,
                PIN_M2_DRIVER_INA, 
                PIN_M2_DRIVER_INB, 
                PIN_M2_DRIVER_PWM);
      //set_servo_speed(&M3,pid_vars_M3, speed_req_turret, speed_act_turret, t_elapsed);
      
      // print_1();
    }

    // if(next_sensor_time < millis()){
    //   compute_imu(SENSOR_LOOPTIME);
    //   hamr_angle = get_current_angle();
    // }
    // next_sensor_time = millis() + SENSOR_LOOPTIME * 1000;

    //read_serial(); //see end of file for this function
  }

}

void init_serial(){
  Serial.begin(250000);
  Serial.println("Arduino Ready\n"); //needs to be sent to detect that arduino has initialized
  Serial.setTimeout(0);
}

/* read a byte from serial. Perform appropriate action based on byte*/
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

      // case SIG_HOLO_X:
      //   sig_var = &speed_req_ddrive;
      //   break;

      // case SIG_HOLO_Y:
      //   sig_var = &speed_req_ddrive;
      //   break;

      // case SIG_HOLO_T:
      //   sig_var = &speed_req_ang;
      //   break;

      case SIG_DD_V:
        sig_var = &speed_req_ddrive;
        break;

      case SIG_DD_R:
        sig_var = &speed_req_ang;
        break;

      // motor velocities
      case SIG_R_MOTOR:
        sig_var = &right_motor;
        break;

      case SIG_L_MOTOR:
        sig_var = &left_motor;
        break;

      case SIG_T_MOTOR:
        sig_var = &turret_motor;
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

      // turrent motor PID
      case SIG_T_KP:
        sig_var = &(pid_vars_M3.Kp);
        break;

      case SIG_T_KI:
        sig_var = &(pid_vars_M3.Ki);
        break;

      case SIG_T_KD:
        sig_var = &(pid_vars_M3.Kd);
        break;
    }

    if(Serial.available()){
      *sig_var = Serial.readString().toFloat();
      Serial.print(" "); Serial.print(*sig_var);
    }
  }
}

/* send relevant data through serial */
void send_serial(){
     if(send_data and Serial){
       Serial.println(SIG_START_STRING);
       delayMicroseconds(500);
       Serial.println(millis() - startMilli); // total time
       Serial.println(speed_act_M1, 4);
       Serial.println(speed_act_M2, 4);
       Serial.println(speed_req_ddrive);
       Serial.println(0);
   }
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

// void rencoderA_M3()  {
//   int encoderA_pin = PIN_M3_ENCODER_OUTA;
//   int encoderB_pin = PIN_M3_ENCODER_OUTB;
//   // Record rising or falling edge from encoder
//   interrupt_M3_A = digitalRead(encoderA_pin);

//   //curr_count_M3++;
  
//   if (interrupt_M3_A != interrupt_M3_B) curr_count_M3--; // encoderA changed before encoderB -> forward
//   else                                  curr_count_M3++; // encoderB changed before encoderA -> reverse
// }

// void rencoderB_M3()  {
//   int encoderA_pin = PIN_M3_ENCODER_OUTA;
//   int encoderB_pin = PIN_M3_ENCODER_OUTB;
//   // Record rising or falling edge from encoder
//   interrupt_M3_B = digitalRead(encoderB_pin);

//   //curr_count_M3++;
//   if (interrupt_M3_A != interrupt_M3_B) curr_count_M3++; // encoderB changed before encoderA -> reverse
//   else                                  curr_count_M3--; // encoderA changed before encoderB -> forward
// }



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
  //Serial.println(speed_act_M3, 4);

//      Serial.print(speed_act_M1, 4);
//      Serial.print(" (");
//      Serial.print(PWM_M1);
//      Serial.print("), ");
//      Serial.print(speed_act_M2, 4);
//      Serial.print(" (");
//      Serial.print(PWM_M2);
//      Serial.print(")\n");

//      Serial.print(curr_count_M1);
//      Serial.print(" ");
//      Serial.print(curr_count_M2);
//      Serial.print("\n");
}