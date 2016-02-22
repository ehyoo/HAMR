#include <PID_v1.h>

//#include <MahonyAHRS.h>
//#include "MahonyAHRS.h"

#define PIN_M2FB 12
#define PIN_M2SF 11
#define PIN_M2PWM 10
#define PIN_M2IN1 9
#define PIN_M2IN2 8
#define PIN_INV 1
#define PIN_EN 0
#define PIN_M1FB 7
#define PIN_M1SF 6
#define PIN_M1PWM 5
#define PIN_M1IN1 4
#define PIN_M1IN2 3

#define forward true
#define backward false

// update this value
#define MIN_DUTY_CYCLE 48
#define MAX_DUTY_CYCLE 255

#define G 16384.0



// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 //(default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO



//constants
float R2D = 180.0 / M_PI;
float D2R = 1 / R2D;

// time tracking
static unsigned long microsLast = 0; //currently unused
float deltaT = .005;

// Control Variables
float Kp = 2.1;
float Ki = 0;
float Kd = .1;
  
float error;
static float error_prev = 0;

float P;
float I;
float D;

float integral;
static float integral_prev = 0;

// RAW IMU variables
float a_scale = 9.81 / G;
long accelx;
long accely;
long accelz;
long accelx_prev;
long accely_prev;
long accelz_prev;

long square_x = accelx*accelx;
long square_y = accely*accely;
long square_z = accelz*accelz;

float g_scale = 1.17 * (1.0/131.0); // * (M_PI / 180.0);
long gyrox;
long gyrox_prev;
long gyroy;
long gyroz;

int imu_offset[6] = {0, 0, 0, 0, 0, 0}; //this is modified by the calibrate_IMU() function


// Calclated IMU Variables
float accely_angle = 0; 
float accely_angle_prev = 0; 

float accelz_angle = 0; // not being used currently
float accelz_angle_prev = 0; // ^

//float accel_angle_filtered = 0;
//float accel_angle_filtered_prev = 0;

float gyro_angle = 0;
float gyro_angle_prev = 0;
float gyro_angle_total = 0;

//
//float gyro_angle_filtered;
//float gyro_angle_filtered_prev = 0;

float final_angle;
float final_angle_prev = 0;

//Filter Variables
// Measure cutoff by taking a video of robot self balancing and measuring frequency
// need to make this as high as possible while sufficiently removing linear acceleration
float accel_lp_cutfreq = 2; 
float accel_lp_alpha = 0; //1.0 / (deltaT * accel_lp_cutfreq + 1);

// 
float gyro_hp_alpha = .5;
float complement = .95;

unsigned long current_time = 0;
unsigned long prev_time = micros();

// this holds the variable that is currently being manipulated by the serial interface
float* current_serial_var;




// MOTOR CONTROL VARIABLES
int pwm = 0;

void setup() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_INV, OUTPUT);
    pinMode(PIN_M1IN1, OUTPUT);
    pinMode(PIN_M1IN2, OUTPUT);
    pinMode(PIN_M2IN1, OUTPUT);
    pinMode(PIN_M2IN2, OUTPUT);

    enable_driver(true);
    invert_motors(false);
    set_motor_direction(forward);

    calibrate_IMU(); // hold robot still as vertical as possible

    //set filter values
    accelgyro.setDLPFMode(6);
}

float motor1_offset = 20;
float motor2_offset = 0;


int print_time = 0;
int i = 0; //change this to something else...

float print_rate = 10; //this is a float for convenience
int do_print = 0;
int print_imu = 0;
void loop() {
    // -----------------------TIMING-------------------------------
    // ------------------------------------------------------------
    current_time = micros();
    if(current_time - prev_time > deltaT * 1000000 - 8){
      print_time++;
      // TODO: include something that will let me know if it is too slow
        // uncomment this to test timing. comment out ALL other print statements
//      i++;
//      if(i == 500){
//        Serial.print("\n");
//        Serial.print(current_time - prev_time);
//        i = 0;
//      }
//      Serial.print(current_time - prev_time);
      prev_time = current_time; 

//      check serial for updates
      read_serial();
      
      // --------------------------IMU-----------------------------
      //-----------------------------------------------------------
      
      // read raw accel/gyro measurements from device
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      // offset raw values based on calibration
      accelx = (ax - imu_offset[0]); 
      accely = (ay - imu_offset[1]); 
      accelz = (az - imu_offset[2] - 16384); 
      gyrox = (gx - imu_offset[3]); 
      gyroy = (gy - imu_offset[4]);
      gyroz = (gz - imu_offset[5]);

      // is it better to low pass here or low pass the final result?
//      accelx = low_pass(accelx, accelx_prev, accel_lp_alpha);
//      accely = low_pass(accely, accely_prev, accel_lp_alpha);
//      accelz = low_pass(accelz, accelz_prev, accel_lp_alpha);

      // calculate accelx_angle and accely_angle
      calc_accel_angle(accelx, accely, accelz);

      // calculate gyro_angle (only x axis)
      integrate_gyro_angle(gyrox, gyrox_prev); //set prev
      
      //filter angles
//      accel_angle_filtered = low_pass(accelz_angle, accel_angle_filtered_prev, accel_lp_alpha); //set prev
      
      //complementary filter
//      complement = .9 + (2 / (exp(-sqrt(square_x + square_y + square_z)/G) + 1) - 1) / 10.0; 
      final_angle = complement * (final_angle_prev + gyro_angle * g_scale) + (1 - complement) * accely_angle * R2D;

      
      //print IMU variables
      if(print_imu && print_time >= print_rate){
        Serial.println("");
        s_print("accel_angleY", float(accely_angle * R2D));
//        s_print("accel_angleZ", float(accelz_angle * R2D));
        s_print("gyro_angle", float(gyro_angle * g_scale));
//        s_print("raw y", ay);
        s_print("gyro angle total", gyro_angle_total * g_scale);
        s_print("final angle", final_angle);
//        s_print("raw gyro", gyrox * g_scale);
        print_time = 0;
      }




      
      // --------------------------PID-----------------------------
      //-----------------------------------------------------------
      
      error = -final_angle * 10;
      P = Kp*error;
  
//      integral = ((error + error_prev) / 2) * deltaT + integral_prev;
//      integral_prev = integral;
//      I = Ki*integral;

      D = Kd *(final_angle - final_angle_prev) / deltaT;

      float spd = P + D; //+ I
      spd = constrain(spd, -100, 100);
      set_motor_speed(spd);

      if(do_print && print_time >= print_rate){
        s_print("P", P);
        s_print("D", D);
        s_print("speed", spd);
        print_time = 0;
      }
      
      // set previous values
      accelx_prev = accelx;
      accely_prev = accely;
      accelz_prev = accelz;
      gyrox_prev = gyrox;
      
//      gyro_angle_filtered_prev = gyro_angle_filtered;
//      accel_angle_filtered_prev = accel_angle_filtered;
      final_angle_prev = final_angle;
      error_prev = error;

      
      if(0){
          // display tab-separated accel/gyro x/y/z values
          Serial.print("RAW a/g:  ");
          Serial.print(accelx); Serial.print("\t");
          Serial.print(accely); Serial.print("\t");
          Serial.print(accelz); Serial.print("\t");
          Serial.print(gyrox); Serial.print("\t");
          Serial.print(gyroy); Serial.print("\t");
          Serial.println(gyroz);
      }
    }

//    while (micros() < microsLast + deltaT*1000000);  // wait for deltaT since last time through
//    microsLast = micros(); // save value for next time through
}






/// Utility Functions---------------------------------------------------------
void s_print(String s, float f){
  Serial.print("  " + s + ": ");
  Serial.print(f, 5);
}

void s_print(String s, int i){
  Serial.print("  " + s + ": ");
  Serial.print(i);
}

void s_print(String s, long l){
  Serial.print("  " + s + ": ");
  Serial.print(l);
}

void s_print(String s, char c){
  Serial.print("  " + s + ": ");
  Serial.print(c);
}

void s_print(String s, unsigned int i){
  Serial.print("  " + s + ": ");
  Serial.print(i);
}

void s_print(String s, unsigned long l){
  Serial.print("  " + s + ": ");
  Serial.print(l);
}

void s_print(String s, unsigned char c){
  Serial.print("  " + s + ": ");
  Serial.print(c);
}

// Read Serial from USB and adjust variables
void read_serial(){
  if(Serial.available()){
    String str = Serial.readString();
    
    if(str == "ac1"){
        current_serial_var = &accel_lp_cutfreq;
    } 
    else if(str == "ac2"){
        current_serial_var = &accel_lp_alpha;
    }
    else if(str == "gy"){
        current_serial_var = &gyro_hp_alpha;
    } 
    else if(str == "c"){
        current_serial_var = &complement;
    }
    else if(str == "dlpf+"){
        accelgyro.setDLPFMode(accelgyro.getDLPFMode() + 1);
        Serial.println("current DLPF value:");
        Serial.println(accelgyro.getDLPFMode());
    }
    else if(str == "dlpf-"){
        accelgyro.setDLPFMode(accelgyro.getDLPFMode() - 1);
        Serial.println("current DLPF value:");
        Serial.println(accelgyro.getDLPFMode());
    }
    else if(str == "p"){
        current_serial_var = &Kp;
    }
    else if(str == "i"){
        current_serial_var = &Ki;
    }
    else if(str == "d"){
        current_serial_var = &Kd;
    }
    else if(str == "pr"){
        current_serial_var = &print_rate;
    }
    else if(str == "m1o"){
        current_serial_var = &motor1_offset;
    }
    else if(str == "m2o"){
        current_serial_var = &motor2_offset;
    }
    else {
        *current_serial_var = str.toFloat();
        Serial.print("Variable was changed to:");
        Serial.println(*current_serial_var);
    }
  }
}




///___________________________________________________________________________
/// IMU FUNCTIONS__________________________________________________________

void calibrate_IMU(){
   delay(100); // allow IMU to settle
   
   int total = 300;
   long imu_total[6] = {0, 0, 0, 0, 0, 0};
   int count = 0;
   int i;
   while(count <= total){
      count++;
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      imu_total[0] += ax;
      imu_total[1] += ay;
      imu_total[2] += az;
      imu_total[3] += gx;
      imu_total[4] += gy;
      imu_total[5] += gz;
   }
   for(i = 0; i < 6; i++){
    imu_offset[i] = imu_total[i] / total;
    Serial.println(imu_offset[i]);
   }
}



void calc_accel_angle(long accelx, long accely, long accelz){
   square_x = accelx*accelx;
   square_y = accely*accely;
   square_z = accelz*accelz;

   //Y Axis
   float denY = sqrt(square_x + square_z);
   accely_angle = -atan(accely/denY);

   //Z axis
   float denZ = sqrt(square_x + square_y);
   accelz_angle = -atan(accelz/denZ); 
   
}

void integrate_gyro_angle(long current, long prev){
  gyro_angle = ((float) (current + prev) / 2.0) * deltaT;
  gyro_angle_total += gyro_angle;
}

float low_pass(float current, float filtered_prev, float alpha){
  return alpha * filtered_prev + (1 - alpha) * current;
}

float high_pass(float filtered_prev, float current, float prev, float alpha){
  return alpha * (filtered_prev + current - prev);
}



///___________________________________________________________________________
/// MOTOR FUNCTIONS________________________________________________________
void enable_driver(boolean state){
  digitalWrite(PIN_EN, state);
}

void invert_motors(boolean state){
  digitalWrite(PIN_INV, state);
}

void set_motor_direction(boolean dir){
  digitalWrite(PIN_M1IN1, dir);
  digitalWrite(PIN_M1IN2, !dir);
  digitalWrite(PIN_M2IN1, dir);
  digitalWrite(PIN_M2IN2, !dir);
}

void brake_motor(){
  digitalWrite(PIN_M1IN1, 1);
  digitalWrite(PIN_M1IN2, 1);
  digitalWrite(PIN_M2IN1, 1);
  digitalWrite(PIN_M2IN2, 1);
}

void set_motor1_duty_cycle(unsigned int value){
  float duty_cycle = constrain(value, 0, 255);
  analogWrite(PIN_M1PWM, duty_cycle);
}

void set_motor2_duty_cycle(unsigned int value){
  float duty_cycle = constrain(value, 0, 255);
  analogWrite(PIN_M2PWM, duty_cycle);
}

void set_motor_speed(int value){
  if(value < 0){
    set_motor_direction(forward);
  } else {
    set_motor_direction(backward);
  }
  
  float abs_spd = constrain(abs(value), 0, 100);

  // MOTOR 1 seems to start at 39
  // MOTOR 2 seems to start at 26 
  
  // remap motor speeds so that they are equal
  float duty_cycle1 = map(abs_spd, 0, 100, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  float duty_cycle2 = map(abs_spd, 0, 100, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  
  set_motor1_duty_cycle(duty_cycle1);
  set_motor2_duty_cycle(duty_cycle2);
}



