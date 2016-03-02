#include "pid.h"
#include "motor.h"
#include "localize.h"

/**********************************************
 * Pin Definitions
 * 
 * M1 = Left Differential Drive (ddrive) motor 
 * M2 = Right Differential Drive (ddrive) motor
 * M3 = Turret motor
 **********************************************/
// DD motor encoders
#define PIN_M1_ENCODER_OUTA 38
#define PIN_M1_ENCODER_OUTB 40
#define PIN_M2_ENCODER_OUTA 32
#define PIN_M2_ENCODER_OUTB 34
// Turret motor encoder
#define PIN_M3_ENCODER_OUTA 30
#define PIN_M3_ENCODER_OUTB 28
// DD motor driver
#define PIN_DD_EN 22
#define PIN_M1_DRIVER_INA 24
#define PIN_M1_DRIVER_INB 26
#define PIN_M1_DRIVER_PWM 3
#define PIN_M2_DRIVER_INA 48
#define PIN_M2_DRIVER_INB 50
#define PIN_M2_DRIVER_PWM 4
// Turret motor driver
#define PIN_M3_DRIVER_INA 41
#define PIN_M3_DRIVER_INB 43
#define PIN_M3_DRIVER_PWM 2

/*
 * Robot Constants
 */
#define TICKS_PER_REV_DDRIVE  1632.0  // number of encoder ticks in one full rotation on diff drive motor
#define TICKS_PER_REV_TURRET  3000.0  // turret motor (250:1 gear ratio * 12)
#define WHEEL_DIAMETER        0.060325 // in meters (2 3/8" diameter)  
#define WHEEL_RADIUS          (WHEEL_DIAMETER / 2.0)  // wheel radius, in meters
#define WHEEL_DIST            0.0 // distance between diff drive wheels
#define DIST_PER_REV          (PI*WHEEL_DIAMETER)  // circumference of wheel in meters
#define LOOPTIME              20.0 // in ms      


// For live-plotting
/* SIGNALS */
const char SIG_START_STRING = '$';
const char SIG_START_LOG = '[';
const char SIG_STOP_LOG = ']';
const char SIG_UNINITIALIZED = '!';

const char SIG_R_MOTOR = 'r';
const char SIG_L_MOTOR = 'l';
const char SIG_T_MOTOR = 't';

const char SIG_R_KP = '1';
const char SIG_R_KI = '2';
const char SIG_R_KD = '3';

const char SIG_L_KP = '4';
const char SIG_L_KI = '5';
const char SIG_L_KD = '6';

const char SIG_T_KP = '7';
const char SIG_T_KI = '8';
const char SIG_T_KD = '9';


/* ignore this - for testing*/
int sin_table[200] = {500,516,531,547,563,578,594,609,624,639,655,669,684,699,713,727,741,755,768,781,794,806,819,831,842,854,864,875,885,895,905,914,922,930,938,946,952,959,965,970,976,980,984,988,991,994,996,998,999,1000,1000,1000,999,998,996,994,991,988,984,980,976,970,965,959,952,946,938,930,922,914,905,895,885,875,864,854,842,831,819,806,794,781,768,755,741,727,713,699,684,669,655,639,624,609,594,578,563,547,531,516,500,484,469,453,437,422,406,391,376,361,345,331,316,301,287,273,259,245,232,219,206,194,181,169,158,146,136,125,115,105,95,86,78,70,62,54,48,41,35,30,24,20,16,12,9,6,4,2,1,0,0,0,1,2,4,6,9,12,16,20,24,30,35,41,48,54,62,70,78,86,95,105,115,125,136,146,158,169,181,194,206,219,232,245,259,273,287,301,316,331,345,361,376,391,406,422,437,453,469,484};
int i;

/* This holds the buffer we write to at the beginning of every incoming communication*/
char buffer[1];

/* This is used to indicate that Arduino is in send_data mode*/
int send_data = 0;

/* These should be whatever variables you want to change*/
float right_motor = 0;
float left_motor = 0;
float turret_motor = 0;
float r_Kp = 0;
float r_Ki = 0;
float r_Kd = 0;

float l_Kp = 0;
float l_Ki = 0;
float l_Kd = 0;

float t_Kp = 0;
float t_Ki = 0;
float t_Kd = 0;

float startMilli;

// Encoder counting interrupt functions
void rencoderA_M1();
void rencoderB_M1();
void rencoderA_M2();
void rencoderB_M2();
void rencoderA_M3();
void rencoderB_M3();

Servo M3;


void setup() {
  init_serial();

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
  attachInterrupt(PIN_M3_ENCODER_OUTA, rencoderA_M3, CHANGE);  
  attachInterrupt(PIN_M3_ENCODER_OUTB, rencoderB_M3, CHANGE);  

  
  digitalWrite(PIN_DD_EN, HIGH);
  // Set Motors as forward
  digitalWrite(PIN_M1_DRIVER_INA, LOW);
  digitalWrite(PIN_M1_DRIVER_INB, HIGH);
  digitalWrite(PIN_M2_DRIVER_INA, LOW);
  digitalWrite(PIN_M2_DRIVER_INB, HIGH);
  digitalWrite(PIN_M3_DRIVER_INA, HIGH);
  digitalWrite(PIN_M3_DRIVER_INB, LOW);

  analogWrite(PIN_M1_DRIVER_PWM, 0);
  analogWrite(PIN_M2_DRIVER_PWM, 0);
  analogWrite(PIN_M3_DRIVER_PWM, 0);

  startMilli = millis();
}


unsigned long lastMilli = 0;                   
unsigned long lastMilliPrint = 0;              
float speed_req_ddrive = 0.0; // desired velocity for Diff Drive (m/s)
float speed_req_turret = 0.0; // desired rotational velocity for Turret (deg/s)
float speed_act_M1 = 0.0;  //actual value (m/s)
float speed_act_M2 = 0.0;  //actual value (m/s)
float speed_act_M3 = 0.0;  //actual value (deg/s)
int PWM_M1 = 0; // PWM value for M1
int PWM_M2 = 0; // PWM value for M2
int PWM_M3 = 0; // PWM value for M3        

volatile long curr_count_M1 = 0; // M1 encoder revolution counter
volatile long prev_count_M1 = 0; // M2 encoder revolution counter
volatile long curr_count_M2 = 0; // M3 encoder revolution counter                                                    
volatile long prev_count_M2 = 0; // M1 encoder revolution counter
volatile long curr_count_M3 = 0; // M2 encoder revolution counter
volatile long prev_count_M3 = 0; // M3 encoder revolution counter


/* PID LOOP VARIABLES */
PID_Vars pid_vars_M1(0.0, 0.0, 0.0);
PID_Vars pid_vars_M2(0.0, 0.0, 0.0);
PID_Vars pid_vars_M3(0.0, 0.0, 0.0);


location hamr_loc;


/*************
 * MAIN LOOP *
 *************/
void loop() {
  //set_speed(255, PIN_M1_DRIVER_INA, PIN_M1_DRIVER_INB, PIN_M1_DRIVER_PWM);
  //set_speed(255, PIN_M2_DRIVER_INA, PIN_M2_DRIVER_INB, PIN_M2_DRIVER_PWM);
  //while(1){};

  if((millis()-lastMilli) >= LOOPTIME) {
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
      
      hamr_loc.update(encoder_counts_M1, encoder_counts_M2, TICKS_PER_REV_DDRIVE, WHEEL_RADIUS, WHEEL_DIST);

      // compute speed
      speed_act_M1 = get_speed(encoder_counts_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
      speed_act_M2 = get_speed(encoder_counts_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
      speed_act_M3 = get_ang_speed(encoder_counts_M3, TICKS_PER_REV_TURRET, t_elapsed);         
      
      // compute PWM value
      update_pid(&pid_vars_M1, &PWM_M1, speed_req_ddrive, speed_act_M1, t_elapsed);
      update_pid(&pid_vars_M2, &PWM_M2, speed_req_ddrive, speed_act_M2, t_elapsed);
      update_pid(&pid_vars_M3, &PWM_M3, speed_req_turret, speed_act_M3, t_elapsed);

      set_speed(PWM_M1, PIN_M1_DRIVER_INA, PIN_M1_DRIVER_INB, PIN_M1_DRIVER_PWM);
      set_speed(PWM_M2, PIN_M2_DRIVER_INA, PIN_M2_DRIVER_INB, PIN_M2_DRIVER_PWM);
      set_speed(PWM_M3, PIN_M3_DRIVER_INA, PIN_M3_DRIVER_INB, PIN_M3_DRIVER_PWM);

      /*
      PWM_M1 = (int) right_motor;
      PWM_M2 = (int) left_motor;
      PWM_M3 = (int) turret_motor;
      */
/*
      Serial.print(speed_act_M1, 4);
      Serial.print(" (");
      Serial.print(PWM_M1);
      Serial.print("), ");
      Serial.print(speed_act_M2, 4);
      Serial.print(" (");
      Serial.print(PWM_M2);
      Serial.print(")\n");
      Serial.print(curr_count_M1);
      Serial.print(" ");
      Serial.print(curr_count_M2);
      Serial.print("\n");
*/
    }

    //read_serial();
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
 *  ENCODER COUNTING INTERRUPT ROUTINES
 *  Motor index: 1 (left DD), 2 (right DD), 3 (turret)
 */
int interrupt_M1_A = 0;
int interrupt_M1_B = 0;
int interrupt_M2_A = 0;
int interrupt_M2_B = 0;
int interrupt_M3_A = 0;
int interrupt_M3_B = 0;

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

void rencoderA_M3()  {
  int encoderA_pin = PIN_M3_ENCODER_OUTA;
  int encoderB_pin = PIN_M3_ENCODER_OUTB;
  // Record rising or falling edge from encoder
  interrupt_M3_A = digitalRead(encoderA_pin);

  //curr_count_M3++;
  
  if (interrupt_M3_A != interrupt_M3_B) curr_count_M3--; // encoderA changed before encoderB -> forward
  else                                  curr_count_M3++; // encoderB changed before encoderA -> reverse
}

void rencoderB_M3()  {
  int encoderA_pin = PIN_M3_ENCODER_OUTA;
  int encoderB_pin = PIN_M3_ENCODER_OUTB;
  // Record rising or falling edge from encoder
  interrupt_M3_B = digitalRead(encoderB_pin);

  //curr_count_M3++;
  if (interrupt_M3_A != interrupt_M3_B) curr_count_M3++; // encoderB changed before encoderA -> reverse
  else                                  curr_count_M3--; // encoderA changed before encoderB -> forward
}


void init_serial(){
  Serial.begin(250000);
  Serial.println("Arduino Ready\n"); //needs to be sent to detect that arduino has initialized
  i = 0;
  Serial.setTimeout(0);
}

/* read a byte from serial. Perform appropriate action based on byte*/
void read_serial(){
  String str;
  float temp;
  float* sig_var;

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

      // motor velocities
      case SIG_R_MOTOR:
        sig_var = &speed_req_ddrive;
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
       i = i == 199 ? 0: i + 1;
   }
}
