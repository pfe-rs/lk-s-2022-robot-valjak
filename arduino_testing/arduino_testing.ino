float xn1 = 0;
float yn1 = 0;
int k = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // Test signal
  float t = micros()/1.0e6;
  float xn = sin(2*PI*11*t) + 0.2*sin(2*PI*50*t);

  float yn = 0.801*yn1 + 0.099*xn + 0.099*xn1;

  delay(1);
  xn1 = xn;
  yn1 = yn;

  if (k%3 == 0) {
    Serial.print(2*xn);
    Serial.print(" ");
    Serial.println(2*yn);
  }
  k = k+1;

}
