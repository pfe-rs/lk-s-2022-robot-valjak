#define directionPin 8
#define ledPin 9

volatile int counter; //vrednost enkodera od pocetka koda

void encoder(){
  if(digitalRead(4)) counter++;
  else counter--;
}

float readEncoderData(){
  return counter/(2*PI);
  }

void driveMotor(int speed){
  if(counter>0){digitalWrite(directionPin,LOW);analogWrite(ledPin,abs(speed));};
  if(counter<=0){digitalWrite(directionPin,HIGH);analogWrite(ledPin,255-abs(speed));};
  }

void setup() {
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(directionPin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3), encoder, RISING);
  //Serial.begin(115200);
}

void loop() {
  driveMotor(counter);
}
