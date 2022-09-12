#define directionPin 8
#define ledPin 9

volatile int counter; //vrednost enkodera od pocetka koda

void encoder(){
  if(digitalRead(4)) counter++;
  else counter--;
}

void setup() {
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(directionPin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3), encoder, RISING);
  //Serial.begin(115200);
}

void loop() {
  if(counter>0){digitalWrite(directionPin,LOW);analogWrite(ledPin,abs(counter));};
  if(counter<=0){digitalWrite(directionPin,HIGH);analogWrite(ledPin,255-abs(counter));};
}
