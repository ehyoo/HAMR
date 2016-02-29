#include "pid.h"
#include "motor.h"
#include "localization.h"

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
#define PIN_M2_ENCODER_OUTA 34
#define PIN_M2_ENCODER_OUTB 32
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
#define LOOPTIME              50.0 // in ms      


// For live-plotting
unsigned long startMilli = 0;
int send_data = 0;

void rencoderA_M1();
void rencoderB_M1();
void rencoderA_M2();
void rencoderB_M2();
void rencoderA_M3();
void rencoderB_M3();


void setup() {
  Serial.begin(250000);       //establishimg serial communication at 9600 baud

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

  Serial.println("Arduino Ready"); //needs to be sent to detect that arduino has initialized
  startMilli = millis();
}


unsigned long lastMilli = 0;                   
unsigned long lastMilliPrint = 0;              
float speed_req_ddrive = 0.5; // desired velocity for Diff Drive (m/s)
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
PID_Vars pid_vars_M1(100.0, 0.0, 0.0);
PID_Vars pid_vars_M2(100.0, 0.0, 0.0);
PID_Vars pid_vars_M3(0.0, 0.0, 0.0);


location hamr_loc;


/*************
 * MAIN LOOP *
 *************/
void loop() {
  set_speed(-255, PIN_M1_DRIVER_INA, PIN_M1_DRIVER_INB, PIN_M1_DRIVER_PWM);
  set_speed(-255, PIN_M2_DRIVER_INA, PIN_M2_DRIVER_INB, PIN_M2_DRIVER_PWM);

  while(1){};

  
  if(Serial.available()){
    String str = Serial.readString();
    Serial.print(str);
    
    if(str == "send\n"){
      send_data = 1;
      delay(200);
    } 
    else if (str == "stop\n"){
      send_data = 0;
    }
  }

  if((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();

    if(send_data){
      Serial.println("st");
      Serial.println(millis() - startMilli);
      Serial.println(speed_act_M1, 4);
      Serial.println(speed_act_M2, 4);
      Serial.println(speed_act_M3, 4);
    }
    
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

    if (millis() - startMilli >= 0) { // conditional used to delay start of control loop for testing
      float t_elapsed = (float) (millis() - startMilli);

      // Count encoder increments since last loop
      long encoder_counts_M1 = curr_count_M1 - prev_count_M1;
      long encoder_counts_M2 = curr_count_M2 - prev_count_M2;
      long encoder_counts_M3 = curr_count_M3 - prev_count_M3;

      hamr_loc.update(encoder_counts_M1, encoder_counts_M1, TICKS_PER_REV_DDRIVE, WHEEL_RADIUS, WHEEL_DIST);
      
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


// Read serial from USB and adjust variables
/*
float* current_serial_var;
void read_serial(){
  if(Serial.available()){
    String str = Serial.readString();
    
    if(str == "p"){
        current_serial_var = &Kp;
    } 
    else if(str == "i"){
        current_serial_var = &Ki;
    }
    else if(str == "d"){
        current_serial_var = &Kd;
    } 
    else if(str == "s"){
        current_serial_var = &speed_req_ddrive;
    }
    else {
        *current_serial_var = str.toFloat();
        //Serial.print("Variable was changed to:");
        //Serial.println(*current_serial_var);
    }
  }
}
*/
