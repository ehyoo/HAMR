#include <Wire.h>

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

/* this value controls whether the robot should move or not */
int move_true = 1;

/*this value is updated by the control software. if it is true, the arduino will send data through the send_serial function */
int send_data = 0;

/* DESIRED VALUES */
// holonomic velocities
float desired_h_xdot = 0; float desired_h_ydot = 0; float desired_h_rdot = 0;

// differential drive velocities
float desired_dd_v = 0; // Diff Drive (m/s)
float desired_dd_r = 0; // desired angular velocity for Diff Drive: set between [-1,1] by controller, mapped to [-90,90] degrees in code
// float speed_req_turret = 0.0; // Turret (rad/s)?

// motor velocities
float desired_M1_v = 0; float desired_M2_v = 0; float desired_MT_v = 0;

// motor PWMs
int pwm_M1 = 0;
int pwm_M2 = 0;
int pwm_MT = 0;
float M1_v_cmd = 0;
float M2_v_cmd = 0;
float MT_v_cmd = 0;

/* CONTROL PARAMETERS */
PID_Vars pid_vars_M1(0.18, 0.0, 0.002);
PID_Vars pid_vars_M2(0.18, 0.0, 0.002);
PID_Vars pid_vars_MT(0.1, 0.0, 0.00002);
PID_Vars dd_ctrl(0.1, 0.0, 0.0);

// PID_Vars pid_vars_dd_v(1.0, 0.0, 0.0);
// PID_Vars pid_vars_dd_r(1.0, 0.0, 0.0);
PID_Vars pid_vars_h_xdot(0.05, 0.0, 0.0);
PID_Vars pid_vars_h_ydot(0.05, 0.0, 0.0);
PID_Vars pid_vars_h_rdot(0.01, 0.0, 0.0);

// Velocity control command
float h_xdot_cmd = 0; float h_ydot_cmd = 0; float h_rdot_cmd = 0; //holonomic
float dtheta_cmd = 0; //differential drive
/* -------------------------------------------------------*/
/* -------------------------------------------------------*/

/* SENSORS -----------------------------------------------*/
// decoder counts
int decoder_count_M1 = 0;
int decoder_count_M2 = 0;
int decoder_count_MT = 0;

const int AVG_FILT_SZ = 5;
float decoder_count_arr_M1[AVG_FILT_SZ];
float decoder_count_arr_M2[AVG_FILT_SZ];
float decoder_count_arr_MT[AVG_FILT_SZ];

int decoder_count_M1_prev = 0;
int decoder_count_M2_prev = 0;
int decoder_count_MT_prev = 0;

long decoder_turret_total = 0;

// encoders
// Encoder counting interrupt functions
//void rencoderA_M1(); void rencoderB_M1();
//void rencoderA_M2(); void rencoderB_M2();
//void rencoderA_MT(); void rencoderB_MT();

/* encoder counters */
//volatile long curr_count_M1 = 0; volatile long prev_count_M1 = 0;
//volatile long curr_count_M2 = 0; volatile long prev_count_M2 = 0;
volatile long curr_count_MT = 0; volatile long prev_count_MT = 0;
//volatile long curr_count_M1 = 0; volatile long prev_count_M1 = 0;
//volatile long curr_count_M2 = 0; volatile long prev_count_M2 = 0;
//volatile long curr_count_MT = 0; volatile long prev_count_MT = 0;

/* encoder output state */
int interrupt_M1_A = 0; int interrupt_M1_B = 0;
int interrupt_M2_A = 0; int interrupt_M2_B = 0;
int interrupt_MT_A = 0; int interrupt_MT_B = 0;

/* measured velocities */
float sensed_M1_v = 0.0;
float sensed_M2_v = 0.0;
float sensed_MT_v = 0.0;

float sensed_M1_v_prev = 0.0;
float sensed_M2_v_prev = 0.0;
float sensed_MT_v_prev = 0.0;

float sensed_M1_v_filt = 0.0;
float sensed_M2_v_filt = 0.0;
float sensed_MT_v_filt = 0.0;

/* computed holonomic velocities */
float computed_xdot = 0.0;
float computed_ydot = 0.0;
float computed_tdot = 0.0;

/* IMU settings */
const float SENSOR_LOOPTIME = 10; //10000 //remeber to update this to microseconds later, or an exact timing method if possible
unsigned long next_sensor_time = millis(); // micros()
unsigned long prev_micros = 0;
float sensed_drive_angle = 0;

/* -------------------------------------------------------*/
/* -------------------------------------------------------*/

// timing: main loop
unsigned long startMilli;
unsigned long lastMilli = 0;
float t_elapsed;

// dd localization
location hamr_loc;

void setup() {
  init_serial();              // initialize serial communication
  init_actuators();           // initialiaze all motors
  // init_encoder_interrupts();  // initialize the encoder interrupts
  init_I2C();                 // initialize I2C bus as master
  init_decoders();
  enable_decoder_clk();

  for (int i = 0; i < AVG_FILT_SZ; i++) {
    decoder_count_arr_M1[i] = 0.0;
    decoder_count_arr_M2[i] = 0.0;
    decoder_count_arr_MT[i] = 0.0;
  }

  // initialize_imu();          // initialize this after integrating IMU

  delay(2000);
  startMilli = millis(); //startMicro = micros()
}

/*HOLONOMIC PID TESTING*/
void holonomic_pid_test() {
    if(millis() < startMilli + 4000){
      desired_h_xdot = 0;
      desired_h_ydot = 0.25;
    } else {
      desired_h_xdot = 0;
      desired_h_ydot = 0;
    }
}

/*SQUARE VIDEO TEST*/
void square_vid_test() {
    if(millis() < startMilli + 2000){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else if(millis() < startMilli + 4000){
      desired_h_xdot = 0;
      desired_h_ydot = -.2;
    } else if(millis() < startMilli + 6000){
      desired_h_xdot = .2;
      desired_h_ydot = 0;
    } else if(millis() < startMilli + 8000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else {
      desired_h_xdot = 0;
      desired_h_ydot = 0;
    }
}

/*RIGHT ANGLE VIDEO TEST*/
void right_angle_vid_test() {
    if(millis() < startMilli + 11000){
      desired_h_xdot = 0;
      desired_h_ydot = .4;
    } else if(millis() < startMilli + 11500){
      desired_h_xdot = 0;
      desired_h_ydot = 0.3;
    } else if(millis() < startMilli + 12000){
      desired_h_xdot = 0;
      desired_h_ydot = 0.2;
    } else if(millis() < startMilli + 12500){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else if(millis() < startMilli + 24000){
      desired_h_xdot = -.35;
      desired_h_ydot = 0;
    } else {
      desired_h_xdot = 0;
      desired_h_ydot = 0;
    }

}

/*SINE WAVE VIDEO TEST
  constant -X_dot, sin wave Y_doty, zero Phi_dot */
void sine_wave_vid_test() {
    if(millis() < startMilli + 24000){
      desired_h_xdot = -0.2;
      desired_h_ydot = 0.2*sin((float) (millis()-startMilli)/12000*2*PI);
    } else {
      desired_h_xdot = 0;
      desired_h_ydot = 0;
    }

}

/***************************/
/* MAIN LOOP
/***************************/
void loop() {
  //  test();       /////////// TESTING
  int i = 0;

  while (move_true) {
    // last timing was between 900 and 1200 microseconds. the range seems high...
    // uncomment the first and last line in while loop to test timing
    // unsigned long start_time = micros();

    // ---------------------------------------------------------------------------
//    holonomic_pid_test();
//    square_vid_test();  
    //right_angle_vid_test();
    //sine_wave_vid_test();
    // ---------------------------------------------------------------------------

    if ((millis() - lastMilli) >= LOOPTIME) { //micros() - lastMicro()
      //serial communication
      read_serial();
      send_serial();

      t_elapsed = (float) (millis() - lastMilli); // (micros() - lastMicros) / 1000.0
      lastMilli = millis();

      //      Serial.println(decoder_count_M1);
      //      unsigned int diff = decoder_count_M1;

      compute_sensed_motor_velocities(); // read encoders
      //      Serial.println(decoder_count_M1);
      //      diff = diff - decoder_count_M1;
      //      Serial.println(diff);

      // DIFFERENTIAL DRIVE CONTROL
      // int use_dd_control = 0;
      // if (use_dd_control == 0) {
      //   // PID velocity control, same input to both motors
      //   desired_M1_v = desired_dd_v;
      //   desired_M2_v = desired_dd_v;
      // } else if (use_dd_control == 1) {
      //   // Differential drive control
      //   angle_control(&dd_ctrl, desired_dd_r, hamr_loc.w, &dtheta_cmd, desired_dd_v, &desired_M1_v, &desired_M2_v, WHEEL_DIST, WHEEL_RADIUS, t_elapsed);
      // } else {
      //   // use indiv setpoints
      //   desired_M1_v = (desired_dd_v - (WHEEL_DIST/2.0) * PI/2.0);
      //   desired_M2_v = (desired_dd_v + (WHEEL_DIST/2.0) * PI/2.0);
      // }

      //M1 is the RIGHT motor and is forward facing caster wheels
      //M2 is LEFT

      /* *********************** */
      /* BEGIN HOLONOMIC CONTROL */
      // compute xdot, ydot, and theta dot using the sensed motor velocties and drive angle
      compute_global_state(sensed_M1_v, sensed_M2_v, sensed_MT_v, sensed_drive_angle,
                           &computed_xdot, &computed_ydot, &computed_tdot);
//      //
      h_xdot_cmd = desired_h_xdot;
      h_ydot_cmd = desired_h_ydot;
      h_rdot_cmd = desired_h_rdot;
//      Serial.print(desired_h_xdot); Serial.print("  "); Serial.println(desired_h_ydot);

      // UNCOMMENT THE FOLLOWING LINE TO ENABLE HOLONOMIC PID
      // holonomic PID
//      h_xdot_cmd += pid_vars_h_xdot.update_pid(desired_h_xdot, computed_xdot, t_elapsed);
//      h_ydot_cmd += pid_vars_h_ydot.update_pid(desired_h_ydot, computed_ydot, t_elapsed);
//      h_rdot_cmd += pid_vars_h_rdot.update_pid(desired_h_rdot, computed_tdot, t_elapsed);

      // using output of holonomic PID, compute jacobian values for motor inputs
     set_holonomic_desired_velocities(h_xdot_cmd, h_ydot_cmd, h_rdot_cmd); // set these setpoints to the output of the holonomic PID controllers
     get_holonomic_motor_velocities(sensed_drive_angle, &desired_M1_v, &desired_M2_v, &desired_MT_v);
     get_holonomic_motor_velocities(hamr_loc.theta, &desired_M1_v, &desired_M2_v, &desired_MT_v);
//    


//     Serial.println("-------------------------------------: ");
//     Serial.print("desired_h_rdot: "); Serial.println(desired_h_rdot);
//     Serial.print("computed_tdot: "); Serial.println(computed_tdot);
//     
//     Serial.print("desired_h_rdot: "); Serial.println(sensed_MT_v);
//     Serial.print("desired_MT_v: "); Serial.println(desired_MT_v);
//     
//     Serial.print("h_rdot_cmd: "); Serial.println(h_rdot_cmd);
//     
//     Serial.print("hamr_loc.theta: "); Serial.println(hamr_loc.theta);
//     Serial.print("sensed_drive_angle: "); Serial.println(sensed_drive_angle);
//      
     
      /* END HOLONOMIC CONDTROL */
      /* ********************** */

       Serial.println(desired_MT_v);
       Serial.println(sensed_MT_v);
      set_speed(&pid_vars_M1,
                desired_M1_v,
                sensed_M1_v,
                &M1_v_cmd,
                t_elapsed,
                &pwm_M1,
                M1_DIR_PIN,
                M1_PWM_PIN);

      set_speed(&pid_vars_M2,
                desired_M2_v,
                sensed_M2_v,
                &M2_v_cmd,
                t_elapsed,
                &pwm_M2,
                M2_DIR_PIN,
                M2_PWM_PIN);

      set_speed(&pid_vars_MT,
                desired_MT_v,
                sensed_MT_v,
                &MT_v_cmd,
                t_elapsed,
                &pwm_MT,
                MT_DIR_PIN,
                MT_PWM_PIN);
    }

    // Serial.print(desired_MT_v); Serial.print(" sensed ");
    // Serial.println(sensed_MT_v); Serial.println(decoder_count_MT);


    // update_prevs();

//    if (next_sensor_time < micros() && is_imu_working()) {
//      unsigned long current_micros = micros();
//
//      compute_imu((current_micros - prev_micros) / 1000000.0); //update imu with time change
//
//      sensed_drive_angle = get_current_angle() * PI / 180;
//
//      next_sensor_time = micros() + SENSOR_LOOPTIME;
//      prev_micros = current_micros;
//
//      // potentially combine hamr_loc.theta with imu angle?
//    } else if (!is_imu_working()) {
//      sensed_drive_angle = hamr_loc.theta;
//    }

    // set the drive angle
    sensed_drive_angle = 2 * PI * (decoder_turret_total % (long) TICKS_PER_REV_TURRET) / (float) TICKS_PER_REV_TURRET;

//     FOR TIME TESTING
//     unsigned long finish_time = micros();
//     Serial.print("total_time: "); Serial.println(finish_time - start_time);
  }

  // disable all PWMs 
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  analogWrite(MT_PWM_PIN, 0);
}

/***************************/
/* END MAIN LOOP
/***************************/

/******************************************/
/* BEGIN SERIAL COMMUNCIATION CODE
/******************************************/

void init_serial() {
  Serial.begin(250000);
  Serial.println("Arduino Ready\n"); // needs to be sent to detect that arduino has initialized
  Serial.setTimeout(0);              // required to speed up serial reading
}

// void update_prevs() {
//   sensed_M1_v_prev = sensed_M1_v;
//   sensed_M2_v_prev = sensed_M2_v;
//   sensed_MT_v_prev = sensed_MT_v;
// }

/* read a byte from Serial. Perform appropriate action based on byte*/
void read_serial() {
  String str;
  float temp;
  float* sig_var;
  char buffer[1];

  buffer[0] = SIG_UNINITIALIZED;
  if (Serial.available()) {
    Serial.readBytes(buffer, 1);
    Serial.print(buffer[0]);

    switch (buffer[0]) {
      case SIG_START_LOG:
        send_data = 1;
        break;

      case SIG_STOP_LOG:
        send_data = 0;
        break;

      // holonomic inputs
      case SIG_HOLO_X:
        sig_var = &desired_h_xdot;
        break;

      case SIG_HOLO_Y:
        sig_var = &desired_h_ydot;
        break;

      case SIG_HOLO_R:
        sig_var = &desired_h_rdot;
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
        sig_var = &desired_M1_v;
        break;

      case SIG_L_MOTOR:
        sig_var = &desired_M2_v;
        break;

      case SIG_T_MOTOR:
        sig_var = &desired_MT_v;
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
        sig_var = &(pid_vars_MT.Kd);
        // sig_var = &(dd_ctrl.Kp);
        break;

      case SIG_T_KI:
        sig_var = &(pid_vars_MT.Kd);
        // sig_var = &(dd_ctrl.Ki);
        break;

      case SIG_T_KD:
        sig_var = &(pid_vars_MT.Kd);
        // sig_var = &(dd_ctrl.Kd);
        break;

      // holonomic X PID
      case SIG_HOLO_X_KP:
        sig_var = &(pid_vars_h_xdot.Kp);
        break;

      case SIG_HOLO_X_KI:
        sig_var = &(pid_vars_h_xdot.Ki);
        break;

      case SIG_HOLO_X_KD:
        sig_var = &(pid_vars_h_xdot.Kd);
        break;

      // holonomic Y PID

      case SIG_HOLO_Y_KP:
        sig_var = &(pid_vars_h_ydot.Kp);
        break;

      case SIG_HOLO_Y_KI:
        sig_var = &(pid_vars_h_ydot.Ki);
        break;

      case SIG_HOLO_Y_KD:
        sig_var = &(pid_vars_h_ydot.Kd);
        break;

      // holonomic R PID

      case SIG_HOLO_R_KP:
        sig_var = &(pid_vars_h_rdot.Kp);
        break;

      case SIG_HOLO_R_KI:
        sig_var = &(pid_vars_h_rdot.Ki);
        break;

      case SIG_HOLO_R_KD:
        sig_var = &(pid_vars_h_rdot.Kd);
        break;

      case SIG_MOVE_FALSE:
        move_true = 0;
        break;
    }

    if (Serial.available()) {
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
void send_serial() {
  // unsigned long starttime = micros(); //uncomment for  time testing
  if (send_data and Serial) {
    Serial.println(SIG_START_STRING);
    delayMicroseconds(500);
    Serial.println(millis() - startMilli); // total time


    Serial.println(desired_h_xdot, 3);
    Serial.println(desired_h_ydot, 3);
    Serial.println(desired_h_rdot, 3);

    Serial.println(computed_xdot, 3);
    Serial.println(computed_ydot, 3);
    Serial.println(computed_tdot, 3);
  
//    Serial.println(desired_M1_v, 3);
//    Serial.println(desired_M2_v, 3);
//    Serial.println(desired_MT_v, 3);
//
//    Serial.println(sensed_M1_v, 3);
//    Serial.println(sensed_M2_v, 3);
//    Serial.println(sensed_MT_v, 3);
//
//    Serial.println(hamr_loc.theta);
//    Serial.println(hamr_loc.w);
//    Serial.println(sensed_drive_angle);

    // Serial.println(desired_dd_v);
    // Serial.println(desired_dd_r);
  }
  // unsigned long finishtime = micros(); //uncomment for  time testing
  // Serial.print("total_time: "); Serial.println(finishtime - starttime); //uncomment for  time testing
}

/******************************************/
/* END SERIAL COMMUNCIATION CODE
/******************************************/


/******************************************/
/* BEGIN MOTOR CODE
/******************************************/
void init_actuators() {
  // Set DD motor driver pins as outputs
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(MT_DIR_PIN, OUTPUT);

  // Set Motors as forward
  digitalWrite(M1_DIR_PIN, M1_FORWARD); // LOW is forwards
  digitalWrite(M2_DIR_PIN, M2_FORWARD); // HIGH is forwards
  digitalWrite(MT_DIR_PIN, MT_COUNTER); // HIGH is CLOCKWISE (use low for default)

  // Initialize PWMs to 0
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  analogWrite(MT_PWM_PIN, 0);
}

float compute_avg(float* arr, int sz) {
  float sum = 0;
  for (int i = 0; i < sz; i++) {
    sum += arr[i];
  }
  return sum / (float) sz;
}

void compute_sensed_motor_velocities() {
  // Count encoder increments since last loop
  // long encoder_count_change_M1 = curr_count_M1 - prev_count_M1;
  // long encoder_count_change_M2 = curr_count_M2 - prev_count_M2;
  // long encoder_count_change_MT = curr_count_MT - prev_count_MT;

  // prev_count_M1 = curr_count_M1;
  // prev_count_M2 = curr_count_M2;
  // prev_count_MT = curr_count_MT;

  // compute speed
  // sensed_M1_v = get_speed(encoder_count_change_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  // sensed_M2_v = get_speed(encoder_count_change_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  // sensed_MT_v = get_ang_speed(encoder_count_change_MT, TICKS_PER_REV_TURRET, t_elapsed);

  // read decoders to get encoder counts
  decoder_count_M1 = read_decoder(0);
  decoder_count_M2 = read_decoder(1);
  decoder_count_MT = read_decoder(2);

  // Serial.println(decoder_count_MT);
  // get change in decoder counts
  float decoder_count_change_M1 = decoder_count_M1 - decoder_count_M1_prev;
  float decoder_count_change_M2 = decoder_count_M2 - decoder_count_M2_prev;
  float decoder_count_change_MT = decoder_count_MT - decoder_count_MT_prev;

  decoder_turret_total += decoder_count_MT - decoder_count_MT_prev;

  //  Serial.println(decoder_count_change_M1);
  //  Serial.println(decoder_count_change_M2);
  //  Serial.println(decoder_count_change_MT);
  //  Serial.println();
  //  if(abs(decoder_count_change_M1) > 32767){
  //    if(decoder_count_change_M1 < 0) {
  //      decoder_count_change_M1 = (decoder_count_M1 - decoder_count_M1_prev) + 65535 + 1;
  //    } else {
  //      decoder_count_change_M1 = (decoder_count_M1 - decoder_count_M1_prev) - 65535 - 1;
  //    }
  //  }

  decoder_count_M1_prev = decoder_count_M1;
  decoder_count_M2_prev = decoder_count_M2;
  decoder_count_MT_prev = decoder_count_MT;

  // Moving average filter on decoder count differences
  for (int i = 1; i < AVG_FILT_SZ; i++) {
    decoder_count_arr_M1[i] = decoder_count_arr_M1[i - 1];
    decoder_count_arr_M2[i] = decoder_count_arr_M2[i - 1];
    decoder_count_arr_MT[i] = decoder_count_arr_MT[i - 1];
  }
  decoder_count_arr_M1[0] = decoder_count_change_M1;
  decoder_count_arr_M2[0] = decoder_count_change_M2;
  decoder_count_arr_MT[0] = decoder_count_change_MT;
  float decoder_count_change_filt_M1 = compute_avg(decoder_count_arr_M1, AVG_FILT_SZ);
  float decoder_count_change_filt_M2 = compute_avg(decoder_count_arr_M2, AVG_FILT_SZ);
  float decoder_count_change_filt_MT = compute_avg(decoder_count_arr_MT, AVG_FILT_SZ);

  // compute robot velocities
  sensed_M1_v = get_speed(decoder_count_change_filt_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  sensed_M2_v = get_speed(decoder_count_change_filt_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  sensed_MT_v = get_ang_speed(decoder_count_change_filt_MT, TICKS_PER_REV_TURRET, t_elapsed);


    // turret encoder testing"
//  Serial.print("sensed: "); Serial.println(sensed_MT_v);
//  Serial.print("turret count: "); Serial.println(decoder_count_MT);
//  // Serial.print(" desired: "); Serial.println(desired_MT_v);

  hamr_loc.update(sensed_M1_v, sensed_M2_v, WHEEL_DIST, t_elapsed);
}

/* initialize encoder interrupts for turret motor */
// void init_encoder_interrupts(){
//   pinMode(PIN_MT_ENCODER_A, INPUT);
//   // pinMode(PIN_MT_ENCODER_B, INPUT);

//   attachInterrupt(PIN_MT_ENCODER_A, rencoderA_MT, CHANGE);
//   attachInterrupt(PIN_MT_ENCODER_B, rencoderB_MT, CHANGE);
// }

// encoder interrupts handlers
// void rencoderA_M1()  {
//   interrupt_M1_A = (PIOB->PIO_PDSR >> 17) & 1;
//   if (interrupt_M1_A != interrupt_M1_B) curr_count_M1--; // encoderA changed before encoderB -> forward
//   else                                  curr_count_M1++; // encoderB changed before encoderA -> reverse
// }

// void rencoderB_M1()  {
//   interrupt_M1_B = (PIOB->PIO_PDSR >> 18) & 1;
//   if (interrupt_M1_A != interrupt_M1_B) curr_count_M1++; // encoderB changed before encoderA -> reverse
//   else                                  curr_count_M1--; // encoderA changed before encoderB -> forward
// }

// void rencoderA_M2()  {
//   interrupt_M2_A = (PIOB->PIO_PDSR >> 19) & 1;
//   if (interrupt_M2_A != interrupt_M2_B) curr_count_M2--; // encoderA changed before encoderB -> forward
//   else                                  curr_count_M2++; // encoderB changed before encoderA -> reverse
// }

// void rencoderB_M2()  {
//   interrupt_M2_B = (PIOB->PIO_PDSR >> 20) & 1;
//   if (interrupt_M2_A != interrupt_M2_B) curr_count_M2++; // encoderB changed before encoderA -> reverse
//   else                                  curr_count_M2--; // encoderA changed before encoderB -> forward
// }

// void rencoderA_MT()  {
//   interrupt_MT_A = (PIOB->PIO_PDSR >> 21) & 1;
//   if (interrupt_MT_A != interrupt_MT_B) curr_count_MT--; // encoderA changed before encoderB -> forward
//   else                                  curr_count_MT++; // encoderB changed before encoderA -> reverse
// }

// void rencoderB_MT()  {
//   interrupt_MT_B = (PIOB->PIO_PDSR >> 22) & 1;
//   if (interrupt_MT_A != interrupt_MT_B) curr_count_MT++; // encoderB changed before encoderA -> reverse
//   else                                  curr_count_MT--; // encoderA changed before encoderB -> forward
// }


/*
  Testing functions
*/
int increment = 1;
int pwm = 0;
void test_motors() {

  pwm += increment;
  if (pwm > 30) {
    increment = -1;
  } else if (pwm < 1) {
    increment = 1;
    digitalWrite(M1_DIR_PIN, LOW);
    digitalWrite(M2_DIR_PIN, HIGH);
    //    digitalWrite(MT_DIR_PIN, LOW);
  }

  analogWrite(M1_PWM_PIN, pwm);
  analogWrite(M2_PWM_PIN, pwm);
  //  analogWrite(MT_PWM_PIN, pwm);

  Serial.print("M1_PWM_PIN: "); Serial.println(M1_PWM_PIN);
  Serial.print("M2_PWM_PIN: "); Serial.println(M2_PWM_PIN);
  //  Serial.print("MT_PWM_PIN: "); Serial.println(MT_PWM_PIN);
  Serial.print("pwm: "); Serial.println(pwm);
  delay(50);
}

/******************************************/
/* END MOTOR CODE
/******************************************/


/******************************************/
/* BEGIN I2C CODE
/******************************************/
void init_I2C() {
  Wire.begin();
}

void request_decoder_count(char slave_addr) {
  Wire.beginTransmission(slave_addr);
  int bytes_available = Wire.requestFrom(slave_addr, (uint8_t) 4);

  if (bytes_available == 4)
  {
    decoder_count_M1 = Wire.read() << 8 | Wire.read();
    decoder_count_M2 = Wire.read() << 8 | Wire.read();
    // decoder_count_MT = Wire.read() << 8 | Wire.read();

    Serial.print("count0: "); Serial.print(decoder_count_M1);
    Serial.print(" count1: "); Serial.println(decoder_count_M2);
    // Serial.print(" count2: "); Serial.println(decoder_count_MT);

  }
  else
  {
    Serial.println("I2C error. Bytes Received: "); Serial.println(bytes_available);
    // light up an LED here
  }
  Wire.endTransmission();
}

void test_I2C_decoder_count() {
  request_decoder_count(99);
  delayMicroseconds(10000);
}

/******************************************/
/* END I2C CODE
/******************************************/


/* TEST FUNCTION. INSERT ALL TESTING IN HERE. make sure to comment these out while not testing, otherwise infinite loop */
void test() {
  delayMicroseconds(1000000);

  while (0) {
    test_ADA();
    read_serial();
  }
  while (1) {
    test_motors();
  }

  //  while(1){
  //    test_I2C_decoder_count();
  //  }
}


void test_ADA() {
  pwm_M1 = adjust_speed(pwm_M1, desired_M1_v);
  pwm_M2 = adjust_speed(pwm_M2, desired_M2_v);
  pwm_MT = adjust_speed(pwm_MT, desired_MT_v);

  analogWrite(M1_PWM_PIN, pwm_M1);
  analogWrite(M2_PWM_PIN, pwm_M2);
  analogWrite(MT_PWM_PIN, pwm_MT);

  digitalWrite(M1_DIR_PIN, (desired_M1_v >= 0) ? M1_FORWARD : !M1_FORWARD);
  digitalWrite(M2_DIR_PIN, (desired_M2_v >= 0) ? M2_FORWARD : !M2_FORWARD);
  digitalWrite(MT_DIR_PIN, (desired_MT_v >= 0) ? MT_COUNTER : !MT_COUNTER);


  Serial.print("pwm_M1: "); Serial.println(pwm_M1);
  Serial.print("pwm_M2: "); Serial.println(pwm_M2);
  Serial.print("pwm_MT: "); Serial.println(pwm_MT);
  Serial.print("pwm: "); Serial.println(pwm);
  delay(10);
}

int adjust_speed(int pwm, int desired) {
  if (pwm > abs(desired)) {
    pwm = abs(desired);
  } else if (pwm < abs(desired)) {
    pwm++;
  }
  return pwm;
}

