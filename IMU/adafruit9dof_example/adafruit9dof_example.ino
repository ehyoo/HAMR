#include "hamr_imu.h"
//#include "MadgwickAHRS.h"s
#include "math.h"

long start_time = millis();
long next_time = millis();

const float LOOPTIME = .05;
const float GYRO_SCALE = 250.0 / 32768.0;

/* raw IMU values */
int16_t ax_, ay_, az_;
int16_t gx_, gy_, gz_;
int16_t mx_, my_, mz_;

void setup() {
  // initialize serial communication
  Serial.begin(250000);
  initialize_imu();
}

void loop() {
  compute_imu(LOOPTIME);
//  get_imu_raw(&ax_, &ay_, &az_, &gx_, &gy_, &gz_, &mx_, &my_, &mz_);
//  gx_ *= GYRO_SCALE;
//  gy_ *= GYRO_SCALE;
//  gz_ *= GYRO_SCALE;
//
//  MadgwickAHRSupdate(ax_, ay_, az_, gx_, gy_, gz_, mx_, my_, mz_);

//  float w = q0;
//  float x = q1;
//  float y = q2;
//  float z = q3;
//  float roll  = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
//  float pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
//  float yaw   = asin(2*x*y - 2*z*w);

//  Serial.print(roll * 180 / PI);  Serial.print(" ");
//  Serial.print(pitch * 180 / PI); Serial.print(" ");
//  Serial.print(yaw * 180 / PI);   Serial.print(" ");
  // print_raw_imu();
  // print_scaled_imu();
  // print_calculated_linear();	
  print_calculated_angular();	
  
//  Serial.print(get_current_angle());
  Serial.println("");
  
  while (millis() < next_time);
  next_time = millis() + LOOPTIME * 1000;
}


