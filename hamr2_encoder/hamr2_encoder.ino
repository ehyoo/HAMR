#include "pid.h"
#include "motor.h"
#include "localize.h"
#include "hamr_imu.h"
#include "dd_control.h"
#include "constants.h"
#include "holonomic_control.h"
#include "decoder.h"

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
int pwm_M1 = 0; 
int pwm_M2 = 0; 
int pwm_M3 = 0; 
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


// timing: main loop
unsigned long startMilli;
unsigned long lastMilli = 0;   
float t_elapsed;                

// dd localization
location hamr_loc;

void setup() {
  init_serial();    // initialize serial communication
  init_actuators(); // initialiaze motors and servos        REMEMBER TO REENABLE THIS
  delay(100);

  // init_decoders();

  // initialize_imu();
  startMilli = millis();
  delay(1000); 

   Serial.println("running");

   Serial.println(PIO_PB17, BIN);
   Serial.println(PIO_PB18, BIN);
}

long next_time = micros();

/*************
 * MAIN LOOP *
 *************/
void loop() {
    //comment these tests when not testing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  while(1){
    while(micros() < next_time);
      Serial.println((curr_count_M1 - prev_count_M1));
      prev_count_M1 = curr_count_M1;
      next_time += 1000000;
    }
  //  test_motors();

  // test decoders 2
  // digitalWrite(DECODER_SEL_PIN, HIGH);
  // while(1);

  Serial.println("Begin test_decoders2");
  while(1){
    test_decoders3();
  }

  // test decoders 1
  // while(1){
  //   void test_decoders();
  // }


  while(1){
    // last timing was between 900 and 1200 microseconds. the range seems high...
    //uncomment the first and last line in while loop to test timing
    // unsigned long start_time = micros();

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

      //M1 is the RIGHT motor and is forward facing caster wheels
      //M2 is LEFT

      // HOLONOMIC CONTROL
      int use_holonomic_control = 1;
      if (use_holonomic_control){
        set_setpoints(desired_h_x, desired_h_y, desired_h_r);
        update_holonomic_state(hamr_angle, &desired_m1_v, &desired_m2_v, &desired_mt_v);
      }

      // Serial.print("the" );
      // Serial.print(hamr_loc.theta);
      // Serial.print(" dm1 ");
      // Serial.print(desired_m1_v);
      // Serial.print(" dm2 ");
      // Serial.println(desired_m2_v);

      // set speeds on motors      
      set_speed(&pid_vars_M1,
                desired_m1_v,
                sensed_m1_v, 
                &m1_v_cmd,
                t_elapsed, 
                &pwm_M1,
                M1_DIR_PIN, 
                M1_PWM_PIN);
      set_speed(&pid_vars_M2,
                desired_m2_v,
                sensed_m2_v, 
                &m2_v_cmd,
                t_elapsed, 
                &pwm_M2,
                M2_PWM_PIN,
                M2_DIR_PIN);
    }

    if(next_sensor_time < millis() && is_imu_working()){
      unsigned long current_micros = micros();

      compute_imu((current_micros - prev_micros) / 1000000.0); //update imu with time change

      hamr_angle = get_current_angle() * PI / 180;

      next_sensor_time = millis() + SENSOR_LOOPTIME;
      prev_micros = current_micros;

      // potentially combine hamr_loc.theta with imu angle?
    } else if(!is_imu_working()) {
      hamr_angle = hamr_loc.theta;
    }

    // unsigned long finish_time = micros();
    // Serial.print("total_time: "); Serial.println(finish_time - start_time);
  }
}

int read_pin(uint32_t ulPin){
  if ( PIO_Get( g_APinDescription[ulPin].pPort, PIO_INPUT, g_APinDescription[ulPin].ulPin ) == 1 )
    {
        return HIGH;
    }
  return LOW;
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


  // Set DD motor driver pins as outputs
  // pinMode(M1_PWM_PIN, OUTPUT);
//   pinMode(M1_DIR_PIN, OUTPUT);
//   // pinMode(M1_SLP_PIN, OUTPUT); // this should be high
//   pinMode(M1_FLT_PIN, INPUT_PULLUP);
//   // pinMode(M2_PWM_PIN, OUTPUT);
//   pinMode(M2_DIR_PIN, OUTPUT);
//   // pinMode(M2_SLP_PIN, OUTPUT);
//   pinMode(M2_FLT_PIN, INPUT_PULLUP);
//   // pinMode(MT_PWM_PIN, OUTPUT);
//   pinMode(MT_DIR_PIN, OUTPUT);
//   // pinMode(MT_SLP_PIN, OUTPUT);
//   pinMode(MT_FLT_PIN, INPUT_PULLUP);

//   // Set Motors as forward
//   digitalWrite(M1_DIR_PIN, HIGH);
//   digitalWrite(M2_DIR_PIN, HIGH);
//   digitalWrite(MT_DIR_PIN, HIGH);

//   // Initialize PWMs to 0
//   analogWrite(M1_PWM_PIN, 0);
//   analogWrite(M2_PWM_PIN, 0);
//   analogWrite(MT_PWM_PIN, 0);
// }

void print1(){
  Serial.println(sensed_m3_v, 4);

   Serial.print(sensed_m1_v, 4);
   Serial.print(" (");
   Serial.print(pwm_M1);
   Serial.print("), ");
   Serial.print(sensed_m2_v, 4);
   Serial.print(" (");
   Serial.print(pwm_M2);
   Serial.print(")\n");

   Serial.print(curr_count_M1);
   Serial.print(" ");
   Serial.print(curr_count_M2);
   Serial.print("\n");
}

/* 
Testing functions
*/
int increment = 1;
int pwm = 0;
void test_motors(){

  pwm += increment;
  if (pwm > 255){
    increment = -1;
  } else if (pwm < 0){
    increment = 1;
  }

  analogWrite(M1_PWM_PIN, pwm);
  analogWrite(M2_PWM_PIN, pwm);
  analogWrite(MT_PWM_PIN, pwm);
  
  Serial.print("M1_PWM_PIN: "); Serial.println(M1_PWM_PIN);
  Serial.print("M2_PWM_PIN: "); Serial.println(M2_PWM_PIN);
  Serial.print("MT_PWM_PIN: "); Serial.println(MT_PWM_PIN);
  Serial.print("pwm: "); Serial.println(pwm);
  delay(50);
}



void init_decoders(){

  // enablePWM(6, maxCount, dutyCycleCount);
  pinMode(DECODER_SEL_PIN, OUTPUT);  
  pinMode(DECODER_OE_PIN, OUTPUT);
  pinMode(DECODER_RST_PIN, OUTPUT);

  
  for (int i = 0; i < 8; i++) {
    pinMode(M1_DECODER_D_PINS[i], INPUT);
    pinMode(M2_DECODER_D_PINS[i], INPUT);
    pinMode(MT_DECODER_D_PINS[i], INPUT);

  }

  // Reset decoder count to 0 at init
  digitalWrite(DECODER_RST_PIN, LOW);
  delay(100);
  digitalWrite(DECODER_RST_PIN, HIGH);

  // reset inhibit logic on decoders
  digitalWrite(DECODER_OE_PIN, LOW);
}


void test_decoders(){
  Serial.println("Starting...");

  // Produce quadrature output
  pinMode(A8,OUTPUT);
  pinMode(A9,OUTPUT);

  for (int i = 0; i < 500; i++) {
    digitalWrite(A8,HIGH);
    delayMicroseconds(100);
    digitalWrite(A9,HIGH);
    delayMicroseconds(100);
    digitalWrite(A8,LOW);
    delayMicroseconds(100);
    digitalWrite(A9,LOW);
    delayMicroseconds(100);
  }
  Serial.println("Done");

  int count = read_decoder(2);

  // float encoder_diff = count 
  // float time_elapsed = 
  // float velocity = get_speed(encoder_diff, 4096, time_elapsed)
  Serial.println("count: ");
  Serial.print(count);
  while(1);

}


int test_previous_decoder_count;
long test_prev_millis;

void test_decoders2(){
  int test_current_decoder_count = read_decoder(3); // read decoder

  // get encoder difference
  int decoder_diff = test_current_decoder_count - test_previous_decoder_count;
  Serial.print("pre-overflow diff "); Serial.print(decoder_diff);
  // test for overflow
  if(abs(decoder_diff) > 32767){
    if(test_current_decoder_count < 0){
      decoder_diff = (test_current_decoder_count - test_previous_decoder_count) + 65535 + 1;
    }
    else {
      decoder_diff = (test_current_decoder_count - test_previous_decoder_count) - 65535 - 1;
    }
    // decoder_diff = test_current_decoder_count + test_previous_decoder_count; 
  }

  // measure time difference
  long test_current_millis = millis();
  float time_elapsed = test_current_millis - test_prev_millis;

  //compute velocity 
  // float wheel_diameter = ??????
  // float dist_per_rev = wheel_diamter * PI;
  float dist_per_rev = 1;
  float ticks_per_rev = 4096;
  float measured_velocity = get_speed((float) decoder_diff, ticks_per_rev, dist_per_rev, time_elapsed);

  Serial.print("velocity: "); Serial.print(measured_velocity);

  Serial.print(" count: "); Serial.print(test_current_decoder_count);
  Serial.print(" difference: "); Serial.println(decoder_diff);
  Serial.print(" time elapsed: "); Serial.println(time_elapsed);

  //set previous values
  test_previous_decoder_count = test_current_decoder_count;
  test_prev_millis = test_current_millis;
}


int test_previous_decoder_count1 = 0;
int test_previous_decoder_count2 = 0;
int test_previous_decoder_count3 = 0;
void test_decoders3(){
  int test_current_decoder_count1 = read_decoder(1);
  int test_current_decoder_count2 = read_decoder(2);
  int test_current_decoder_count3 = read_decoder(3);

  int decoder_diff1 = test_current_decoder_count1 - test_previous_decoder_count1;
  int decoder_diff2 = test_current_decoder_count2 - test_previous_decoder_count2;
  int decoder_diff3 = test_current_decoder_count3 - test_previous_decoder_count3;

  Serial.print(test_current_decoder_count1); Serial.print(" ");
  Serial.print(test_current_decoder_count2); Serial.print(" ");
  Serial.print(test_current_decoder_count3); Serial.print(" ");

  Serial.print(decoder_diff1); Serial.print(" ");
  Serial.print(decoder_diff2); Serial.print(" ");
  Serial.print(decoder_diff3); Serial.println(" ");

  test_previous_decoder_count1 = test_current_decoder_count1;
  test_previous_decoder_count2 = test_current_decoder_count2;
  test_previous_decoder_count3 = test_current_decoder_count3;
}


/* initialize encoder */
void init_actuators(){
  pinMode(PIN_M1_ENCODER_OUTA, INPUT);
  pinMode(PIN_M1_ENCODER_OUTB, INPUT);

  attachInterrupt(PIN_M1_ENCODER_OUTA, rencoderA_M1, CHANGE);  
  attachInterrupt(PIN_M1_ENCODER_OUTB, rencoderB_M1, CHANGE);
}

// encoder interrupts
void rencoderA_M1()  {
  interrupt_M1_A = (PIOB->PIO_PDSR >> 17) & 1;
  if (interrupt_M1_A != interrupt_M1_B) curr_count_M1--; // encoderA changed before encoderB -> forward
  else                                  curr_count_M1++; // encoderB changed before encoderA -> reverse
}

void rencoderB_M1()  {
  interrupt_M1_B = (PIOB->PIO_PDSR >> 18) & 1;
  if (interrupt_M1_A != interrupt_M1_B) curr_count_M1++; // encoderB changed before encoderA -> reverse
  else                                  curr_count_M1--; // encoderA changed before encoderB -> forward
}