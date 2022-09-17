#include <Wire.h>

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll = 0; 
float pitch = 0;
float yaw = 0;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

//String errCodes[] = ["BVL","IATH"]

int c = 0;
char n;
String out, temp;
int motorOut;

unsigned long dt = 5000;
unsigned long previous_time = 0;

const int MPU = 0x68; // MPU6050 I2C address
const int directionPin = 10;
const int motorPin = 9;
const int batSens = A0;
const int robotDia = 22; 
int motorPWM;
volatile int counter; //vrednost enkodera od pocetka koda

float degree_to_radian(float degree) {return (degree * PI)/180.0;}
float radian_to_degree(float radian) {return (radian * 180.0)/PI;}

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
  float accAngleX = atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) + degree_to_radian(1.11); // AccErrorX ~(-1.11) See the calculate_IMU_error()custom function for more details
  float accAngleY = atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) + degree_to_radian(3.37); // AccErrorY ~(-3.37)

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
  GyroX = GyroX + degree_to_radian(2.68); // GyroErrorX ~(-2.68)
  GyroY = GyroY + degree_to_radian(1.92); // GyroErrorY ~(-1.92)
  GyroZ = GyroZ + degree_to_radian(1.18); // GyroErrorZ ~ (-1.18)

  //return GyroX;
  return GyroY;
  //return GyroZ;
}

//String errCodes[] = ["Battery Voltage Low","IMU Angle too High"]

float getBatteryVoltage(){
  int voltage;
  float voltageOut;
  voltage = analogRead(A0);
  voltageOut = voltage * 4.23 / 205;
  return voltageOut;
  }

void failsafes(){
  int errCode = 0;
  if(getBatteryVoltage() < 15){errCode = 1;};
  if(pitch > 1){errCode = 2;};

  if(errCode != 0){
    Serial.print("System fail.\nError code: ");
    if(errCode = 1){ Serial.print("Battery Voltage Low");};
    if(errCode = 2){ Serial.print("IMU Angle too High");};
    while(1){Serial.println("SYS HALT");delay(100);};
    }
  }

float readSpeed(){//Derives the speed based on the encoder readings in m/s
  int time1, time2;
  int counter2;
  int speedT = 0;
  float speedA = 0;

  time2 = millis();
  if(time2 - time1 > 100){
    speedT = counter2 - counter;
    counter2 = counter;
    speedA = speedT * 10;
    speedA = speedA / 360 * PI * robotDia;
    return speedA;
    }
  }

float readAngleSpeed(){//Derives the speed based on the encoder readings in Rad/s
  int time1, time2;
  int counter2;
  int speedT = 0;
  float speedA = 0;

  time2 = millis();
  if(time2 - time1 > 100){
    speedT = counter2 - counter;
    counter2 = counter;
    speedA = speedT * 10;
    speedA = speedA / 360;
    return speedA;
    }
  }

void printData(){
  float printSpeed = readSpeed();
  float VBat = getBatteryVoltage();
  float upTime = millis()/1000;
  pitch = imuPitch();
  
    String out = "";
    out += pitch;out +=", ";
    out += counter;out +=", ";
    out += motorPWM;out +=", ";
    out += VBat;out +=", ";
    out += upTime;
    
  Serial.println(out);
  }

float readEncoderData(){
  return counter/(2*PI);
  }

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

void encoder(){
  if(digitalRead(4)){
    counter++;
    }else{counter--;};
}

void driveMotor(int voltage){
  int limit = 5;
  Serial.println(voltage);

  voltage = min(voltage,12);
  voltage = max(voltage,-12);
  voltage = map(voltage,-12,12,-255,255);
  
  motorPWM = voltage;

//  Serial.print(speed);
//  Serial.print(',');
  if(voltage>0){digitalWrite(directionPin,LOW);analogWrite(motorPin,abs(voltage));/*Serial.println(1);*/};
  if(voltage<=0){digitalWrite(directionPin,HIGH);analogWrite(motorPin,255-abs(voltage));/*Serial.println(-1);*/};
  }

float imuPitch(){
  float accAngleY = read_accel_data();
    
  float GyroY = read_gyro_data();
  
  // Complementary filter - combine acceleromter and gyro angle values
  float tau = 0.2;
  float alpha = tau/(tau + 0.005);
  pitch = (1 - alpha)*(pitch + (GyroY * dt)/1000000.0) + alpha * accAngleY;//pitch range +/- 1.56
  }

void setup() {
  Serial.begin(9600);

  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(directionPin,OUTPUT);
  pinMode(motorPin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3), encoder, RISING);
  Serial.println("conection.......");
  
  MPU6050_initialization();
}

void loop() {
  
//   while (Serial.available() > 0) {
//      char n = Serial.read();
//      if (isDigit(n)||n=='-') {
//        temp += n;
//    }
//    if (n == '\n' || n == '\r') {
//      motorOut = temp.toInt();
//      driveMotor(motorOut);
//      Serial.print(temp);
//      Serial.print(',');
//      temp = "";
//    }
//  }

  

  // Time control
  while(micros() < previous_time + dt) {}
  previous_time = micros();

  

  printData();
  delay(1);
}
