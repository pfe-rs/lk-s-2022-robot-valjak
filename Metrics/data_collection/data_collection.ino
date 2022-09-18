#include <Wire.h>

#define interrupt_pin 3
#define encoder_pin   4
#define direction_pin 10
#define motor_pin     9

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll = 0; 
float pitch = 0;
float yaw = 0;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

unsigned long dt = 5000;
unsigned long previous_time = 0;

volatile int encoder_counter = 0;

void setup() {
  Serial.begin(9600);

  pinMode(interrupt_pin,INPUT_PULLUP);
  pinMode(encoder_pin,INPUT_PULLUP);
  pinMode(direction_pin,OUTPUT);
  pinMode(motor_pin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), encoder_interrupt, RISING);
  
  MPU6050_initialization();
  
  // Call this function if you need to get the IMU error values for your module
//  calculate_IMU_error();
  delay(20);
}
 
void loop() {
  float accAngleY = read_accel_data();
  float GyroY = read_gyro_data();
  float encoder = read_encoder_data();
  
  Serial.print(accAngleY);
  Serial.print(",");
  Serial.print(GyroY);
  Serial.print(",");
  Serial.println(encoder);
  
  // Time control
  while(micros() < previous_time + dt) {}
  previous_time = micros();
}

void encoder_interrupt() {
  if(digitalRead(encoder_pin)) encoder_counter++;
  else encoder_counter--;
}

void drive_motor(int voltage) {
  int limit = 5;

  voltage = map(constrain(voltage, -limit, limit), -12, 12, -255, 255);

  if(voltage>0) {
    digitalWrite(direction_pin, LOW);
    analogWrite(motor_pin, voltage);
  } else {
    digitalWrite(direction_pin, HIGH);
    analogWrite(motor_pin, 255-abs(voltage));
  }
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
  float accAngleX = atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) + degree_to_radian(0.3); // AccErrorX ~(-1.11) See the calculate_IMU_error()custom function for more details
  float accAngleY = atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) + degree_to_radian(86.0); // AccErrorY ~(-3.37)

  //return accAngleX;
  return accAngleY; 
}

float read_gyro_data() {
  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  
  const float gyro_to_radian = PI/(180.0*131.0); // 131.0 is from datasheet
  float GyroX = (Wire.read() << 8 | Wire.read()) * gyro_to_radian; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  float GyroY = (Wire.read() << 8 | Wire.read()) * gyro_to_radian;
  float GyroZ = (Wire.read() << 8 | Wire.read()) * gyro_to_radian;
  
  // Correct the outputs with the calculated error values
  GyroX = GyroX + degree_to_radian(2.50); // GyroErrorX ~(-2.68)
  GyroY = GyroY + degree_to_radian(1.92); // GyroErrorY ~(-1.92)
  GyroZ = GyroZ + degree_to_radian(1.15); // GyroErrorZ ~ (-1.18)

  //return GyroX;
  return GyroY;
  //return GyroZ;
}

float read_encoder_data() {
  float steps_per_revolution = 600;
  return (encoder_counter*2.0*PI)/steps_per_revolution;
}

float degree_to_radian(float degree) {return (degree * PI)/180.0;}
float radian_to_degree(float radian) {return (radian * 180.0)/PI;}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
