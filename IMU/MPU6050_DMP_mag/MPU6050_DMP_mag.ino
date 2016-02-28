#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

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

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  // devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // If you don't know yours, you can find an automated sketch for this task from: http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/


  // TODO: Compute these parameters
  mpu.setXAccelOffset(-3591);
  mpu.setYAccelOffset(-842);
  mpu.setZAccelOffset(578);

  mpu.setXGyroOffset(-51);
  mpu.setYGyroOffset(-57);
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

  // make sure it worked (returns 0 if so)
  // if (devStatus == 0) {
  //   // turn on the DMP, now that it's ready
  //   Serial.println(F("Enabling DMP..."));
  //   mpu.setDMPEnabled(true);

  //   // enable Arduino interrupt detection
  //   Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
  //   attachInterrupt(interrupt_pin, dmpDataReady, RISING);
  //   mpuIntStatus = mpu.getIntStatus();

  //   // set our DMP Ready flag so the main loop() function knows it's okay to use it
  //   Serial.println(F("DMP ready! Waiting for first interrupt..."));
  //   dmpReady = true;

  //   // get expected DMP packet size for later comparison
  //   packetSize = mpu.dmpGetFIFOPacketSize();
  // } 
  // else {
  //   // ERROR!

  //   // 1 = initial memory load failed
  //   // 2 = DMP configuration updates failed
  //   // (if it's going to break, usually the code will be 1)
  //   Serial.print(F("DMP Initialization failed (code "));
  //   Serial.print(devStatus);
  //   Serial.println(F(")"));
  // }

  // configure LED for output
  mpu.setDLPFMode(6);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //Read magnetometer measures
  mx=mpu.getExternalSensorWord(0);
  my=mpu.getExternalSensorWord(2);
  mz=mpu.getExternalSensorWord(4);

  heading = atan2(my, mx);
  if(heading < 0) heading += 2 * M_PI;

  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  Serial.print("\t h:\t");
  Serial.print(heading * 180/M_PI);
  Serial.print("\t F:\t");
  Serial.println("");
}

// note must calculate actual angles using turret motor offset

void set_position(float x, float y){

}

void get_current_position_x(){

}

void get_current_position_y(){

}

void set_rotation(float degrees){

}

void get_current_angle(){

}

void get_translation_x(){

}

void get_translation_y(){

}

void get_rate_rotation(){
  // gyro + mag differentiated
}

void get_rate_translation_x(){

}

void get_rate_translation_y(){

}