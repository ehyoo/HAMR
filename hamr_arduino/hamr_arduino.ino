

/**********************************************
 * Pin Definitions
 * 
 * M1 = Left Differential Drive (DD) motor 
 * M2 = Right Differential Drive (DD) motor
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

// PD control defines
#define TICKS_PER_REV   1632  // number of encoder ticks in one full rotation on motor
#define WHEEL_DIAMETER  0.060325 // in meters (2 3/8" diameter)   
#define DIST_PER_REV    (PI*WHEEL_DIAMETER)  // circumference of wheel in meters
#define NUMREADINGS     10
#define LOOPTIME        100 // in ms      
int readings[NUMREADINGS];


void setup() {
  Serial.begin(9600);       //establishimg serial communication at 115600 baud

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

  attachInterrupt(PIN_M2_ENCODER_OUTA, rencoder_M2, CHANGE);  
  attachInterrupt(PIN_M3_ENCODER_OUTA, rencoder_M3, CHANGE);  

  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  
  
  digitalWrite(PIN_DD_EN, HIGH);
  // Set Motors as forward
  digitalWrite(PIN_M1_DRIVER_INA, LOW);
  digitalWrite(PIN_M1_DRIVER_INB, HIGH);
  digitalWrite(PIN_M2_DRIVER_INA, LOW);
  digitalWrite(PIN_M2_DRIVER_INB, HIGH);
  
  digitalWrite(PIN_M3_DRIVER_INA, HIGH);
  digitalWrite(PIN_M3_DRIVER_INB, LOW);


  analogWrite(PIN_M1_DRIVER_PWM, 0);
  //analogWrite(PIN_M2_DRIVER_PWM, 50);
  //analogWrite(PIN_M3_DRIVER_PWM, 255);
}



unsigned long lastMilli = 0;                   
unsigned long lastMilliPrint = 0;              
float speed_req = 1.0;              // Set Point
float speed_act_M1 = 0.0;           //actual value
float speed_act_M2 = 0.0;           //actual value
float speed_act_M3 = 0.0;           //actual value
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

float Kp = 15.0;          //setting Kp  
float Kd = 0.0;            //setting Kd

void doDemo() {
    if (curr_count_M1/48 < 100) {
      digitalWrite(PIN_M1_DRIVER_INA, LOW);
      digitalWrite(PIN_M1_DRIVER_INB, HIGH);
      digitalWrite(PIN_M2_DRIVER_INA, LOW);
      digitalWrite(PIN_M2_DRIVER_INB, HIGH);
      digitalWrite(PIN_M3_DRIVER_INA, HIGH);
      digitalWrite(PIN_M3_DRIVER_INB, LOW);
    } else if (curr_count_M1/48 < 150) {
      digitalWrite(PIN_M1_DRIVER_INA, HIGH);
      digitalWrite(PIN_M1_DRIVER_INB, LOW);
      digitalWrite(PIN_M2_DRIVER_INA, LOW);
      digitalWrite(PIN_M2_DRIVER_INB, HIGH);
    } else if (curr_count_M1/48 < 250) {
      digitalWrite(PIN_M1_DRIVER_INA, LOW);
      digitalWrite(PIN_M1_DRIVER_INB, HIGH);
      digitalWrite(PIN_M2_DRIVER_INA, LOW);
      digitalWrite(PIN_M2_DRIVER_INB, HIGH);
      digitalWrite(PIN_M3_DRIVER_INA, LOW);
      digitalWrite(PIN_M3_DRIVER_INB, HIGH);
    } else if (curr_count_M1/48 < 300) {
      digitalWrite(PIN_M1_DRIVER_INA, HIGH);
      digitalWrite(PIN_M1_DRIVER_INB, LOW);
      digitalWrite(PIN_M2_DRIVER_INA, LOW);
      digitalWrite(PIN_M2_DRIVER_INB, HIGH);
      curr_count_M1 = 0;
    } else {
      analogWrite(PIN_M1_DRIVER_PWM, 0);
      analogWrite(PIN_M2_DRIVER_PWM, 0);
      analogWrite(PIN_M3_DRIVER_PWM, 0);
    }
  
}      

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

void loop() {
  //getParam();
  // check keyboard
  //doDemo();


  if((millis()-lastMilli) >= LOOPTIME) {                                                     // enter timed loop
    lastMilli = millis();

    // calculate speed
    getMotorData(&speed_act_M1, &curr_count_M1, &prev_count_M1);
    //getMotorData(speed_act_M2, curr_count_M2, prev_count_M2);
    //getMotorData(speed_act_M3, curr_count_M3, prev_count_M3);         

    // compute PWM value
    updatePid(&PWM_M1, speed_req, speed_act_M1);
    //PWM_M2 = updatePid(PWM_M2, speed_req, speed_act_M2);
    //PWM_M3 = updatePid(PWM_M3, speed_req, speed_act_M3);
    
    // send PWM to motor
 //   Serial.print("PWM M1: ");
 //   Serial.println(PWM_M1);

    set_speed(PWM_M1, PIN_M1_DRIVER_INA, PIN_M1_DRIVER_INB, PIN_M1_DRIVER_PWM);
    //analogWrite(PIN_M2_DRIVER_PWM, PWM_M2);
    //analogWrite(PIN_M3_DRIVER_PWM, PWM_M3);

  }
 
  
  //printMotorInfo();    
}

void getMotorData(float* speed_act, volatile long* curr_count, volatile long* prev_count) { // calculate speed
  // Calculating the speed using encoder count

  
 // int count_diff = curr_count - prev_count;
 // float distance = ((float) count_diff / TICKS_PER_REV)
  //speed_act = (((float) (curr_count - prev_count) / TICKS_PER_REV) * DIST_PER_REV) / ((float) LOOPTIME / 1000.0);
  *speed_act = (((float) (*curr_count - *prev_count) / 1632.0) * DIST_PER_REV) / (100.0 / 1000.0);
  Serial.println(*speed_act);
  *prev_count = *curr_count;                                           //setting count value to last count
}

void updatePid(int* command, float targetValue, float currentValue) {      // compute PWM value
  float pidTerm = 0.0;                                                   // PID correction
  float error = 0.0;                                  
  static float last_error = 0.0;                             
  error = abs(targetValue) - abs(currentValue); 
  pidTerm = (Kp * error) + (Kd * (error - last_error));                            
  last_error = error;
  *command = constrain(round(*command + pidTerm), -255, 255);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////

/* 
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

void rencoder_M2()  {
  int encoder_pin = PIN_M2_ENCODER_OUTB;

  if (digitalRead(encoder_pin)==HIGH)    
  curr_count_M2--;                // if encoder pin 2 = HIGH then count --
  else                     
  curr_count_M2++;                // if encoder pin 2 = LOW then count ++
}

void rencoder_M3()  {
  int encoder_pin = PIN_M3_ENCODER_OUTB;

  if (digitalRead(encoder_pin)==HIGH)    
  curr_count_M3--;                // if encoder pin 2 = HIGH then count --
  else                     
  curr_count_M3++;                // if encoder pin 2 = LOW then count ++
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
int getParam()  
{
  char param, cmd;
  if(!Serial.available())    return 0;
  delay(10);                  
  param = Serial.read();                              // get parameter byte
  if(!Serial.available())    return 0;
  cmd = Serial.read();                                // get command byte
  Serial.flush();                                     // clean serial buffer
  
  switch (param) 
  {  
    case 's':                                         // adjust speed more
    if(cmd=='+')  
      {
        speed_req += 100;
        if(speed_req>400)   speed_req=400;
      }
      if(cmd=='-')    
      {
        speed_req -= 100;
        if(speed_req<0)   speed_req=0;
      }
      break;


/////////////////////////////////////////////  
    case 'r':                                         // adjust speed slowly 
    if(cmd=='+')  
      {
        speed_req += 20;
        if(speed_req>400)   speed_req=400;
      }
    if(cmd=='-')    
      {
        speed_req -= 20;
        if(speed_req<0)   speed_req=0;
      }
      break;
  ////////////////////////////////////////////    
    case 'a':                                        // adjust direction
    if(cmd=='+')
      {
        digitalWrite(InA1, LOW);
        digitalWrite(InB1, HIGH);
      }
    if(cmd=='-')   
      {
        digitalWrite(InA1, HIGH);
        digitalWrite(InB1, LOW);
      }
    break;
  ////////////////////////////////////////////   
    case 'o':                                        // type "oo" to stop motor
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    speed_req = 0;
    break;
  ////////////////////////////////////////////
    case 'p':                                       //adjust proportional gain
    if(cmd=='+')
      {
       Kp += 0.1;
      }
    if(cmd=='-')   {
       Kp -= 0.1;
      }
    break;
  ///////////////////////////////////////////
    case 'd':                                     //adjust derivative gain
    if(cmd=='+')
      {
       Kd += 0.1;
      }
    if(cmd=='-')   
      {
       Kd -= 0.1;
      }
    break;
  ///////////////////////////////////////////
    default: 
      Serial.println("ERROR");
  }
}

void printMotorInfo()  {                                                     // display data
  if((millis()-lastMilliPrint) >= 150)   
  {                     
    lastMilliPrint = millis();

    int pwm_value = PWM_val;
    pwm_value = map(pwm_value, 0, 255, 0, 100);

    Serial.print("SP:");                Serial.print(speed_req);  
    Serial.print("  RPM:");           Serial.print(speed_act);
    Serial.print("  PWM:");          Serial.print(PWM_val);   
    Serial.print("  enc_count:");   Serial.print(count);
    Serial.print("  kp:");              Serial.print(Kp);
    Serial.print("  kd:");              Serial.println(Kd);            
}}
*/
