#include <SPI.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "math.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu(0x69); 

int interrupt_pin=2; // Check where your INT pin is connected. Check below.

#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float heading;          // Simple magnetic heading. (NOT COMPENSATED FOR PITCH AND ROLL)

/* CONSTANTS */
const float ACCEL_SCALE = 2 * 9.8 / 32768;
const float GYRO_SCALE = 250.0 / 32768;
// cont float PI = 3.14159265359;

/* computed values */
float current_x = 0;
float current_y = 0;
float current_x_velocity = 0;
float current_y_velocity = 0;
float current_x_acceleration = 0;
float current_y_acceleration = 0;

float prev_x = 0;
float prev_y = 0;
float prev_x_velocity = 0;
float prev_y_velocity = 0;
float prev_x_acceleration = 0;
float prev_y_acceleration = 0;

float current_angle = 0;
float current_angular_velocity = 0;
float current_angular_acceleration = 0;

float prev_angle = 0;
float prev_angular_velocity = 0;

float prev_gz;

/* calculation parameters */
float looptime = .02;
float angle_comp = .9; // complementary filter value. increasing it increases weight of gyro


/* for timing*/
long start_time = millis();
long next_time = millis();


void setup() {

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  // **************************************************************
  // It is best to configure I2C to 400 kHz. 
  // If you are using an Arduino DUE, modify the variable TWI_CLOCK to 400000, defined in the file:
  // c:/Program Files/Arduino/hardware/arduino/sam/libraries/Wire/Wire.h
  // If you are using any other Arduino instead of the DUE, uncomment the following line:
  //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)  //This line should be commented if you are using Arduino DUE
  // **************************************************************
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(250000);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // TODO: Compute these parameters
  mpu.setXAccelOffset(-1600);
  mpu.setYAccelOffset(-180);
  mpu.setZAccelOffset(650);

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  // Magnetometer configuration

  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x02); 
  Wire.write(0x00);  // Set continuous mode
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x00);
  Wire.write(B00011000);  // 75Hz
  Wire.endTransmission();
  delay(5);

  mpu.setI2CBypassEnabled(0);

  // X axis word
  mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
  mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
  mpu.setSlaveEnabled(0, true);
  mpu.setSlaveWordByteSwap(0, false);
  mpu.setSlaveWriteMode(0, false);
  mpu.setSlaveWordGroupOffset(0, false);
  mpu.setSlaveDataLength(0, 2);

  // Y axis word
  mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
  mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
  mpu.setSlaveEnabled(1, true);
  mpu.setSlaveWordByteSwap(1, false);
  mpu.setSlaveWriteMode(1, false);
  mpu.setSlaveWordGroupOffset(1, false);
  mpu.setSlaveDataLength(1, 2);

  // Z axis word
  mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
  mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
  mpu.setSlaveEnabled(2, true);
  mpu.setSlaveWordByteSwap(2, false);
  mpu.setSlaveWriteMode(2, false);
  mpu.setSlaveWordGroupOffset(2, false);
  mpu.setSlaveDataLength(2, 2);

  mpu.setI2CMasterModeEnabled(1);

  mpu.setDLPFMode(6);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  // get sensor readings
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mx=mpu.getExternalSensorWord(0);
  my=mpu.getExternalSensorWord(2);
  mz=mpu.getExternalSensorWord(4);

  heading = atan2(my, mx);
  if(heading < 0) heading += 2 * M_PI;

  // print_raw_imu();
  // print_scaled_imu(); 
  // print_calculated_imu(); 
  Serial.println("");

  /* computations */
  compute_state();

  // current_x += 
  while (millis() < next_time);
  next_time = millis() + 20;
}


/*********************************
COMPUTATIONS
**********************************/

void compute_state(){
  compute_position_acceleration();
  compute_position_velocity();
  compute_position();

  compute_angle(); //use gyro + mag 
  compute_angular_velocity(); // differentiate above
  compute_angular_acceleration(); // differentiate above
}

void compute_position_acceleration(){
  current_x_acceleration = ax * ACCEL_SCALE;
  current_y_acceleration = ay * ACCEL_SCALE;

  prev_x_acceleration = current_x_acceleration;
  prev_y_acceleration = current_y_acceleration;
}

void compute_position_velocity(){
  current_x_velocity = prev_x_velocity + (current_x_acceleration + prev_x_acceleration) * looptime / 2.0;
  current_y_velocity = prev_y_velocity + (current_y_acceleration + prev_y_acceleration) * looptime / 2.0;

  prev_x_velocity = current_x_velocity;
  prev_y_velocity = current_y_velocity;
}

void compute_position(){
  current_x = prev_x + (current_x_velocity + prev_x_velocity) * looptime / 2.0;
  current_y = prev_y + (current_y_velocity + prev_y_velocity) * looptime / 2.0;

  prev_x = current_x;
  prev_y = current_y;
}

void compute_angle(){
  float mag_angle; //calculate this
  float gyro_angle;

  mag_angle = (my > 0) ? 90.0 - atan((float) mx / my)*180.0/PI : 270.0 - atan((float) mx / my)*180.0/PI;
  // mag_angle = mag_angle - 11;
  if(my == 0){
    mag_angle = (mx > 0) ? 0.0 : 180.0;
  }

  gyro_angle = (gz + prev_gz) * looptime * GYRO_SCALE / 2.0;
  current_angle = angle_comp * (prev_angle + gyro_angle) + (1.0 - angle_comp) * (mag_angle);

  prev_angle = current_angle;

  Serial.print("mag_angle:"); Serial.print(mag_angle); Serial.print("  ");
  Serial.print("current_angle:"); Serial.print(current_angle); Serial.print("  ");
}

void compute_angular_velocity(){
  prev_angular_velocity = current_angular_velocity;
  current_angular_velocity = (current_angle - prev_angle) / looptime; 
}

void compute_angular_acceleration(){
  current_angular_acceleration = (prev_angular_velocity - current_angular_velocity) / looptime;
}

/*********************************
FILTERING
**********************************/
float low_pass(float current, float filtered_prev, float alpha){
  return alpha * filtered_prev + (1 - alpha) * current;
}

float high_pass(float filtered_prev, float current, float prev, float alpha){
  return alpha * (filtered_prev + current - prev);
}


/*********************************
GETTERS AND SETTERS
**********************************/
void set_x_y(float x, float y){
    current_x = x;
    current_y = y;
}

void set_angle(float degrees){
    current_angle = degrees;
}

float get_current_x(){
    return current_x;
}

float get_current_y(){
    return current_y;
}

float get_current_angle(){
    return current_angle;
}

float get_current_x_velocity(){
    return current_x_velocity;
}

float get_current_y_velocity(){
    return current_y_velocity;
}   

float get_current_angular_velocity(){
    return current_angular_velocity;
}


/*********************************
PRINTING
**********************************/
void print_raw_imu(){
  Serial.print("ax:"); Serial.print(ax); Serial.print("  ");
  Serial.print("ay:"); Serial.print(ay); Serial.print("  ");
  Serial.print("az:"); Serial.print(az); Serial.print("  ");
  Serial.print("gx:"); Serial.print(gx); Serial.print("  ");
  Serial.print("gy:"); Serial.print(gy); Serial.print("  ");
  Serial.print("gz:"); Serial.print(gz); Serial.print("  ");
  Serial.print("mx:"); Serial.print(mx); Serial.print("  ");
  Serial.print("my:"); Serial.print(my); Serial.print("  ");
  Serial.print("mz:"); Serial.print(mz); Serial.print("  ");
}

void print_scaled_imu(){
  Serial.print("ax:"); Serial.print(ax * ACCEL_SCALE); Serial.print("  ");
  Serial.print("ay:"); Serial.print(ay * ACCEL_SCALE); Serial.print("  ");
  Serial.print("az:"); Serial.print(az * ACCEL_SCALE); Serial.print("  ");
  Serial.print("gx:"); Serial.print(gx * GYRO_SCALE); Serial.print("  ");
  Serial.print("gy:"); Serial.print(gy * GYRO_SCALE); Serial.print("  ");
  Serial.print("gz:"); Serial.print(gz * GYRO_SCALE); Serial.print("  ");
  Serial.print("heading:"); Serial.print(heading * 180/M_PI);
}

void print_calculated_imu(){
  Serial.print("x_accel:"); Serial.print(current_x_acceleration); Serial.print("  ");
  Serial.print("y_accel:"); Serial.print(current_y_acceleration); Serial.print("  ");
  Serial.print("x vel:"); Serial.print(current_x_velocity); Serial.print("  ");
  Serial.print("y vel:"); Serial.print(current_y_velocity); Serial.print("  ");
  Serial.print("x:"); Serial.print(current_y); Serial.print("  ");
  Serial.print("y:"); Serial.print(current_y); Serial.print("  ");
}

