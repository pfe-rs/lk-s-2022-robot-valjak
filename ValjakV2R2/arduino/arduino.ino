#include <Wire.h>

#define interrupt_pin 3
#define encoder_pin   4
#define direction_pin 10
#define motor_pin     9

const float KpAngle = 6;
const float KiAngle = 0.000001;
const float KdAngle = 0;

const float KpSpeed = 6;
const float KiSpeed = 0.000001;
const float KdSpeed = 0;

const float KpPosition = 6;
const float KiPosition = 0.000001;
const float KdPosition = 0;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

float pitch = 0;

volatile int encoderCounter = 0;
long counter2;

float global_voltage = 0;
int sysState = 0;

unsigned long dt = 0.005;
unsigned long previous_time = 0;

void setup() {
  Serial.begin(9600);

  pinMode(interrupt_pin,INPUT_PULLUP);
  pinMode(encoder_pin,INPUT_PULLUP);
  pinMode(direction_pin,OUTPUT);
  pinMode(motor_pin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), encoder_interrupt, RISING);
  
  MPU6050_initialization();
  
  //delay(5000);
//  for(int i = 10;i > 0;i--){
//    delay(1000);
//    Serial.println(i);
//    };
}
 
void loop() {
  compFilter();

  
  float positionSetpoint = 20;
  float PIDP = PIDPosition(positionSetpoint,positionEstimate());
  float PIDS = PIDSpeed(PIDP,speedEstimate());
  float PIDA = PIDAngle(PIDS,pitch);
  //driveMotor(PIDA);

  
  //printData();
  
  while(micros() < previous_time + dt) {}
  previous_time = micros();
}

unsigned long currentTimePIDAngle, elapsedTimePIDAngle, previousTimePIDAngle;
float errorAngle, cumErrorAngle, rateErrorAngle, lastErrorAngle;

float PIDAngle(float setpoint, float input){
  currentTimePIDAngle = millis();
  elapsedTimePIDAngle = currentTimePIDAngle - previousTimePIDAngle;

  errorAngle = setpoint - input; //offset
  cumErrorAngle += errorAngle * elapsedTimePIDAngle; //used for I component
  const float intErrLimit = 0.001;
  cumErrorAngle = constrain(cumErrorAngle, -intErrLimit,intErrLimit);
  rateErrorAngle = (errorAngle - lastErrorAngle)/elapsedTimePIDAngle; //used for D component

  float output = KpAngle * errorAngle + KiAngle * cumErrorAngle + KdAngle * rateErrorAngle; //calculating output

  lastErrorAngle = errorAngle;
  previousTimePIDAngle = currentTimePIDAngle; //updating parameters

  return output;
  }

unsigned long currentTimePIDSpeed, elapsedTimePIDSpeed, previousTimePIDSpeed;
float errorSpeed, cumErrorSpeed, rateErrorSpeed, lastErrorSpeed;

float PIDSpeed(float setpoint, float input){
  currentTimePIDSpeed = millis();
  elapsedTimePIDSpeed = currentTimePIDSpeed - previousTimePIDSpeed;

  errorSpeed = setpoint - input; //offset
  cumErrorSpeed += errorSpeed * elapsedTimePIDSpeed; //used for I component
  const float intErrLimit = 0.001;
  cumErrorSpeed = constrain(cumErrorSpeed, -intErrLimit,intErrLimit);
  rateErrorSpeed = (errorSpeed - lastErrorSpeed)/elapsedTimePIDSpeed; //used for D component

  float output = KpSpeed * errorSpeed + KiSpeed * cumErrorSpeed + KdSpeed * rateErrorSpeed; //calculating output

  lastErrorSpeed = errorSpeed;
  previousTimePIDSpeed = currentTimePIDSpeed; //updating parameters

  return output;
  }

unsigned long currentTimePIDPosition, elapsedTimePIDPosition, previousTimePIDPosition;
float errorPosition, cumErrorPosition, rateErrorPosition, lastErrorPosition;

float PIDPosition(float setpoint, float input){
  currentTimePIDPosition = millis();
  elapsedTimePIDPosition = currentTimePIDPosition - previousTimePIDPosition;

  errorPosition = setpoint - input; //offset
  cumErrorPosition += errorPosition * elapsedTimePIDPosition; //used for I component
  const float intLimit = 0.001;
  cumErrorPosition = constrain(cumErrorPosition, -intLimit,intLimit);
  rateErrorPosition = (errorPosition - lastErrorPosition)/elapsedTimePIDPosition; //used for D component

  float output = KpPosition * errorPosition + KiPosition * cumErrorPosition + KdPosition * rateErrorPosition; //calculating output

  lastErrorPosition = errorPosition;
  previousTimePIDPosition = currentTimePIDPosition; //updating parameters

  return output;
  }

void compFilter(){
  float accAngleY = read_accel_data();
  float GyroY = read_gyro_data();
  float encoder = readEncoderData();

  // Complementary filter - combine acceleromter and gyro angle values
  float tau = 0.2;
  float alpha = tau/(tau + 0.005);
  pitch = (1 - alpha)*(pitch + (GyroY * dt)/1000000.0) + alpha * accAngleY;
  }


int i = 0;
int state = 1;
unsigned long time1 = 0;
unsigned long time2 = 0;

void stepFunct(){
  int timeout = 5000;
  int timein = 2500;
  int nIterations = 5;
  time2 = millis();
  if(time2 - time1 > timein && state == 0){driveMotor(3);time1 = time2;state = 1;};
  if(time2 - time1 > timeout && state == 1){driveMotor(0);time1 = time2;state = 2;};
  if(time2 - time1 > timein && state == 2){driveMotor(-3);time1 = time2;state = 3;};
  if(time2 - time1 > timeout && state == 3){driveMotor(0);time1 = time2;state = 0;i++;};
  if(i == nIterations){driveMotor(0);state = -1;};
}

void failsafes(){
  int errCode = 0;
  if(getBatteryVoltage() < 14){errCode = 1;};
  if(pitch > 1){errCode = 2;};

  if(errCode != 0){
    Serial.print("System fail.\nError code: ");
    if(errCode = 1){ Serial.print("Battery Voltage Low");};
    if(errCode = 2){ Serial.print("IMU Angle too High");};
    while(1){Serial.println("SYS HALT");delay(100);};
    }
  }

void encoder_interrupt() {
  if(digitalRead(encoder_pin)){encoderCounter++;
  }else{
    encoderCounter--;
  };
}

void driveMotor(float voltage) {
  const int limit = 12;
  
  
  voltage = constrain(-voltage*21.25, -limit*21.25, limit*21.25);
  global_voltage = voltage/21.25;

  
  if(voltage>0) {
    digitalWrite(direction_pin, LOW);
    analogWrite(motor_pin, voltage);
  } else {
    digitalWrite(direction_pin, HIGH);
    analogWrite(motor_pin, 255-abs(voltage));
  }
}

float getBatteryVoltage(){
  int voltage;
  float voltageOut;
  voltage = analogRead(A0);
  voltageOut = voltage / 205.0 * 4.23;
  return voltageOut;
  }

float Sest = 0, Xprev = 0; // Variable for the speedEstimate function
unsigned long timePrev = 0, time = 0, diff = 10;

float speedEstimate(){
  float Xenc = readEncoderData(); //Distance reported by the encoder
  float Ximu = pitch; //Distance reported by the IMU
  float X = Xenc - Ximu; //Actual distance covered by the robot
  float X2; //Speed of the robot
  time = millis();
  if(time - timePrev > diff){
    X2 = (X - Xprev);
    Serial.println(X2);
    timePrev = time;
    Xprev = X;
    };
  
  
  Sest = 0.6*Sest - X2;
  float estSpeed = -0.4*Sest + X2;
  return estSpeed;
  }

float positionEstimate(){
  float Xenc = readEncoderData(); //Distance reported by the encoder
  float Ximu = pitch; //Distance reported by the IMU
  float X = Xenc - Ximu; //Actual distance covered by the robot
  return X;
  }

float readSpeed(){//Derives the speed based on the encoder readings in m/s
  int time1, time2;
  
  int speedT = 0;
  float speedA = 0;

  time2 = millis();
  if(time2 - time1 > 100){
    speedT = counter2 - encoderCounter;
    time1 = time2;
    counter2 = encoderCounter;
    speedA = speedT / 60 * PI * 22;
    return speedA;
    }
  }

void printData(){
  float VBat = getBatteryVoltage();
  float upTime = millis()/1000.0;
  float speed = speedEstimate();
    String out = "";
    out += pitch;out +=",";
    out += speed;out +=",";
    out += global_voltage;out +=",";
    out += VBat;out +=",";
    out += sysState;out +=",";
    out += upTime;
    
  Serial.println(out);
  }
  
void MPU6050_initialization() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
}

struct Vec {
  float x, y, z;
  Vec(float x, float y, float z) : x(x), y(y), z(z) {}
};

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

  // izmerite g
  // izmeriti osaRotacije
  float accLen = sqrt(AccX*AccX + AccY*AccY + AccZ*AccZ);
  Vec acc(AccX / accLen, AccY / accLen, AccZ / accLen);
  Vec g(1, 0, 0);
  Vec osaRotacije(0.0195, -0.985, 0.068);

  float dot = acc.x * g.x + acc.y * g.y + acc.z * g.z;
  float theta = acos(dot);

  float crossX = acc.y * g.z - acc.z * g.y;
  float crossY = acc.z * g.x - acc.x * g.z;
  float crossZ = acc.x * g.y - acc.y * g.x;
  float sign = osaRotacije.x * crossX + osaRotacije.y * crossY + osaRotacije.z * crossZ;
  sign = sign > 0 ? 1 : -1;

//  Serial.println(sign * theta);
  return sign * theta;
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

float readEncoderData() {
  float steps_per_revolution = 600;
  return (encoderCounter*2.0*PI)/steps_per_revolution;
}

float degree_to_radian(float degree) {return (degree * PI)/180.0;}
float radian_to_degree(float radian) {return (radian * 180.0)/PI;}
