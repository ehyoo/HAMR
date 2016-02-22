
/**********************************************
 * Pin Definitions
 * 
 * M1 = Left Differential Drive (ddrive) motor 
 * M2 = Right Differential Drive (ddrive) motor
 * M3 = Turret motor
 **********************************************/
// DD motor encoders
#define PIN_M2_ENCODER_OUTA 36
#define PIN_M2_ENCODER_OUTB 34
#define PIN_M1_ENCODER_OUTA 38
#define PIN_M1_ENCODER_OUTB 40
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

// PID control defines
#define TICKS_PER_REV_ddrive  1632.0  // number of encoder ticks in one full rotation on diff drive (ddrive) motor
#define TICKS_PER_REV_turret  3000.0  // turret (turret) motor (250:1 gear ratio * 12)
#define WHEEL_DIAMETER    0.060325 // in meters (2 3/8" diameter)   
#define DIST_PER_REV      (PI*WHEEL_DIAMETER)  // circumference of wheel in meters
#define LOOPTIME          50.0 // in ms      

// For live-plotting
unsigned long startMilli = 0;
int send_data = 0;

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
  analogWrite(PIN_M3_DRIVER_PWM, 255);

  Serial.println("Arduino Ready"); //needs to be sent to detect that arduino has initialized
  startMilli = millis();
}


unsigned long lastMilli = 0;                   
unsigned long lastMilliPrint = 0;              
float speed_req_ddrive = 0.0; // desired velocity for Diff Drive (m/s)
float speed_req_turret = 45.0; // desired rotational velocity for Turret (deg/s)
float speed_act_M1 = 0.0;  //actual value (m/s)
float speed_act_M2 = 0.0;  //actual value (m/s)
float speed_act_M3 = 0.0;  //actual value (deg/s)
int PWM_M1 = 0; // PWM value for M1
int PWM_M2 = 0; // PWM value for M2
int PWM_M3 = 0; // PWM value for M3        
int interrupt_M1_A = 0;
int interrupt_M1_B = 0;
int interrupt_M2_A = 0;
int interrupt_M2_B = 0;
int interrupt_M3_A = 0;
int interrupt_M3_B = 0;
volatile long curr_count_M1 = 0; // M1 encoder revolution counter
volatile long prev_count_M1 = 0; // M2 encoder revolution counter
volatile long curr_count_M2 = 0; // M3 encoder revolution counter                                                    
volatile long prev_count_M2 = 0; // M1 encoder revolution counter
volatile long curr_count_M3 = 0; // M2 encoder revolution counter
volatile long prev_count_M3 = 0; // M3 encoder revolution counter

#define DIR_FORWARD 1
#define DIR_BACKWARD 0
void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir) {
  digitalWrite(pin_driver_inA, !dir);
  digitalWrite(pin_driver_inB, dir);
}

void set_speed(int pwm_val, int pin_driver_inA, int pin_driver_inB, int pin_pwm) {
  if (pwm_val < 0) {
    // reverse direction
    set_direction(pin_driver_inA, pin_driver_inB, DIR_BACKWARD);
  } else {
    // forward direction
    set_direction(pin_driver_inA, pin_driver_inB, DIR_FORWARD);
  }

  analogWrite(pin_pwm, abs(pwm_val));
}


void measure_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count) { // calculate speed
  // Calculating the speed using encoder count
 // int count_diff = curr_count - prev_count;
 // float distance = ((float) count_diff / TICKS_PER_REV)
  //*speed_act = (((float) (curr_count - prev_count) / TICKS_PER_REV) * DIST_PER_REV) / ((float) LOOPTIME / 1000.0);
  *speed_act = (((float) (*curr_count - *prev_count) / 1632.0) * DIST_PER_REV) / (LOOPTIME / 1000.0);
  //Serial.println(round(100* *speed_act));
  *prev_count = *curr_count; //setting count value to last count
}

void measure_rot_speed(float* speed_act, volatile long* curr_count, volatile long* prev_count) { // calculate speed
  *speed_act = 360.0 * ((*curr_count - *prev_count) / (float) TICKS_PER_REV_turret);
  *prev_count = *curr_count; //setting count value to last count
  Serial.println(*speed_act);
}

/* PID LOOP VARIABLES */
float ddrive_Kp = 120.0;
float ddrive_Ki = 150.0; 
float ddrive_Kd = 1.5;
float turret_Kp = 10.0;
float turret_Ki = 0.0; 
float turret_Kd = 0.0;
float error_acc_M1 = 0.0;
float error_acc_M2 = 0.0;
float error_acc_M3 = 0.0;
float error_prev_M1 = 0.0;
float error_prev_M2 = 0.0;
float error_prev_M3 = 0.0;

float update_pid(String mode, int* command, float targetValue, float currentValue, float* error_acc, float* prev_error) {
  float Kp = 0.0;
  float Ki = 0.0;
  float Kd = 0.0;
  
  if (mode == "d") {
    Kp = ddrive_Kp;
    Ki = ddrive_Ki;
    Kd = ddrive_Kd;
  } else if (mode == "t") {
    Kp = turret_Kp;
    Ki = turret_Ki;
    Kd = turret_Kd;
  }
  
  float pidTerm = 0.0;   
  float error = 0.0;        
  
  float error_acc_limit = 1.0;
                          
  error = targetValue - currentValue; 
  *error_acc += error * (LOOPTIME/1000.0);

  pidTerm = (Kp * error) + (Kd * (error - *prev_error) / (LOOPTIME/1000.0)) + (Ki * (*error_acc));      
  /*
  Serial.print("P: ");
  Serial.print((Kp * error),3);
  Serial.print(", D: ");
  Serial.print((Kd * (error - *prev_error) / (LOOPTIME/1000.0)),3);
  Serial.print(", I: ");
  Serial.print((Ki * (*error_acc)),3);
  Serial.print("\n");
*/
  // Anti integrator windup using clamping
  *error_acc = constrain(*error_acc, -1*error_acc_limit, error_acc_limit);
  
  *prev_error = error; // update error
  
  //*command = constrain(round(*command + pidTerm), -255, 255);
  *command = constrain(round(pidTerm) * 2.55, -255, 255);
}

/*************
 * MAIN LOOP *
 *************/
void loop() {
  //getParam();
  // check keyboard
  //doDemo();
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
    /*
    Serial.print(speed_act_M1, 4);
    Serial.print(" (");
    Serial.print(PWM_M1);
    Serial.print("), ");
    Serial.print(speed_act_M2, 4);
    Serial.print(" (");
    Serial.print(PWM_M2);
    Serial.print(")\n");*/

    if (millis() - startMilli >= 0) {
    // calculate speed
    measure_speed(&speed_act_M1, &curr_count_M1, &prev_count_M1);
    measure_speed(&speed_act_M2, &curr_count_M2, &prev_count_M2);
    measure_rot_speed(&speed_act_M3, &curr_count_M3, &prev_count_M3);         
    Serial.println(curr_count_M3);
    // compute PWM value
    update_pid("d", &PWM_M1, speed_req_ddrive, speed_act_M1, &error_acc_M1, &error_prev_M1);
    update_pid("d", &PWM_M2, speed_req_ddrive, speed_act_M2, &error_acc_M2, &error_prev_M2);
    //update_pid("t", &PWM_M3, speed_req_turret, speed_act_M3, &error_acc_M3, &error_prev_M3);

    set_speed(PWM_M1, PIN_M1_DRIVER_INA, PIN_M1_DRIVER_INB, PIN_M1_DRIVER_PWM);
    set_speed(PWM_M2, PIN_M2_DRIVER_INA, PIN_M2_DRIVER_INB, PIN_M2_DRIVER_PWM);
    //set_speed(PWM_M3, PIN_M3_DRIVER_INA, PIN_M3_DRIVER_INB, PIN_M3_DRIVER_PWM);

    }
    //read_serial();
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

  if (interrupt_M2_A != interrupt_M2_B) curr_count_M2--; // encoderA changed before encoderB -> forward
  else                                  curr_count_M2++; // encoderB changed before encoderA -> reverse
}

void rencoderB_M2()  {
  int encoderA_pin = PIN_M2_ENCODER_OUTA;
  int encoderB_pin = PIN_M2_ENCODER_OUTB;
  // Record rising or falling edge from encoder
  interrupt_M2_B = digitalRead(encoderB_pin);
  
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
