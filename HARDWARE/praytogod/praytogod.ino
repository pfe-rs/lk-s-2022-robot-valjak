#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccZ;
float accAngleX, accAngleZ;
float roll = 0; 
float pitch = 0;
float yaw = 0;
float AccErrorX,AccErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

unsigned long dt = 5000;
unsigned long previous_time = 0;

void setup() {
  Serial.begin(115200);
  MPU6050_initialization();
  delay(20);
}


void loop() {
  float accAngleZ = read_accel_data();
  // Complementary filter - combine acceleromter and gyro angle values
   float alpha = 0.05;
  pitch = (1 - alpha)*pitch + alpha * accAngleZ*1.57;
  Serial.println(pitch);
  // Time control
  while(micros() < previous_time + dt) {}
  previous_time = micros();
}

void MPU6050_initialization() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
}

float read_accel_data() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  
  
  // Calculating Roll and Pitch from the accelerometer data
  float accAngleZ = atan(AccZ / AccX) + degree_to_radian(1.11);
  

  return accAngleZ; 
}

float degree_to_radian(float degree) {return (degree * PI)/180.0;}
float radian_to_degree(float radian) {return (radian * 180.0)/PI;}
