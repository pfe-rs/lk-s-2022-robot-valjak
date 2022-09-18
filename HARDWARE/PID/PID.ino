#include <Wire.h>

#define interrupt_pin 3
#define encoder_pin   4
#define direction_pin 10
#define motor_pin     9



const float Kp = 6;
const float Ki = 0.000001;
const float Kd = 0;



const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll = 0; 
float pitch = 0;
float yaw = 0;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
long int elapsedTimePID, currentTimePID, previousTimePID;
float error, lastError, rateError, cumError, setpoint, input;
int c = 0;

float global_voltage = 0;

int sysState = 0;
int time1 = 0;
int time2 = 0;
int state = 1;
int i = 0;

unsigned long dt = 0.005;
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
  
  
  for(int i = 10;i > 0;i--){
    delay(1000);
    Serial.println(i);
    };
}
 
void loop() {
  compFilter();
  setpoint = degree_to_radian(30);
  driveMotor(PID(pitch));

  
  
  
  //if(abs(pitch) < 1.0){
    //stepFunct();
  //}else{driveMotor(0);}
  
  Serial.print(pitch);
  Serial.print(',');
  Serial.println(global_voltage);
  // Time control

  
  while(micros() < previous_time + dt) {}
  previous_time = micros();
}

float PID(float input){
  currentTimePID = millis();
  elapsedTimePID = currentTimePID - previousTimePID;

  error = setpoint - input; //offset
  cumError += error * elapsedTimePID; //used for I component
  rateError = (error - lastError)/elapsedTimePID; //used for D component

  float output = Kp * error + Ki * cumError + Kd * rateError; //calculating output

  lastError = error;
  previousTime = currentTime; //updating parameters

  return output;
  }

void compFilter(){
  float accAngleY = read_accel_data();
  float GyroY = read_gyro_data();
  float encoder = read_encoder_data();

  // Complementary filter - combine acceleromter and gyro angle values
  float tau = 0.2;
  float alpha = tau/(tau + 0.005);
  pitch = (1 - alpha)*(pitch + (GyroY * dt)/1000000.0) + alpha * accAngleY;
  }

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
  if(getBatteryVoltage() < 15){errCode = 1;};
  if(pitch > 1){errCode = 2;};

  if(errCode != 0){
    Serial.print("System fail.\nError code: ");
    if(errCode = 1){ Serial.print("Battery Voltage Low");};
    if(errCode = 2){ Serial.print("IMU Angle too High");};
    while(1){Serial.println("SYS HALT");delay(100);};
    }
  }

void encoder_interrupt() {
  if(digitalRead(encoder_pin)) encoder_counter++;
  else encoder_counter--;
}

void driveMotor(float voltage) {
  int limit = 5;
  
  
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
  voltageOut = voltage * 4.23 / 205;
  return voltageOut;
  }

float readSpeed(){//Derives the speed based on the encoder readings in m/s
  int time1, time2;
  int counter2;
  int speedT = 0;
  float speedA = 0;

  time2 = millis();
  if(time2 - time1 > 100){
    speedT = counter2 - encoder_counter;
    counter2 = encoder_counter;
    speedA = speedT * 10;
    speedA = speedA / 600 * PI * 22;
    return speedA;
    }
  }

void printData(){
  float printSpeed = readSpeed();
  float VBat = getBatteryVoltage();
  float upTime = millis()/1000;
  
    String out = "";
    out += pitch;out +=",";
    out += encoder_counter;out +=",";
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

float read_encoder_data() {
  float steps_per_revolution = 600;
  return (encoder_counter*2.0*PI)/steps_per_revolution;
}

float degree_to_radian(float degree) {return (degree * PI)/180.0;}
float radian_to_degree(float radian) {return (radian * 180.0)/PI;}
