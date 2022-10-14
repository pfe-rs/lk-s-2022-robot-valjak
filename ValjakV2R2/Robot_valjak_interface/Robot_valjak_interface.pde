import processing.serial.*;
Serial port;


float angleReaderX = 0.15;
float angleReaderY = 0.25;
int angleReaderR = 200;

float speedX = 0.15;
float speedY = 0.75;

float motorVoltageX = 0.5;
float motorVoltageY = 0.15;

float batteryVoltageX = 0.5;
float batteryVoltageY = 0.35;

float sysStateX = 0.5;
float sysStateY = 0.55;

float sysUptimeX = 0.5;
float sysUptimeY = 0.75;

float batteryGraphX = 0.32;
float batteryGraphY = 0.15;
float batteryGraph = 30;
float batteryFull = 0;
float batteryEmpty = 464;

float imuAngle = 0;
float speedAngle = 0;
float motorVoltage = 10.5;
float batteryVoltage = 15.6;
float batteryPercentage = 90;
float sysStateInt = 0;
String sysState = "Stable";
float sysUptime = 1250.6;

float windowWidth = 460;

String inBuffer;
boolean single = true;
int ln = 13;
int cr = 10;
String list[] = {"1.00", "1.00", "12", "15", "0", "2500"};
float[] listF = {0, 0, 0, 0, 0, 0};

void setup() {
  size(1080, 640);
  String portName = "COM8";
  port = new Serial(this, portName, 9600);

  port.clear();
  // Throw out the first reading, in case we started reading
  // in the middle of a string from the sender.

  frameRate(100);
}

void draw() {

  if (port.available() > 0) {
    inBuffer = port.readStringUntil(cr);
    if (inBuffer != null) {
      println(inBuffer);
      list = split(inBuffer, ',');
      if (list.length == 6) {
        for (int i = 0; i<6; i++) {
          listF[i] = float(list[i]);
          //print(listF[i]);
          //print(',');
        }
      };


      //println();
    }
  }

  imuAngle = constrain(listF[0], -PI, PI);
  speedAngle = -constrain(listF[1]/50, 0, PI*28/18);
  motorVoltage = listF[2];
  batteryVoltage = listF[3];
  sysStateInt = listF[4];
  sysUptime = listF[5];
  //listF[0] += 0.01;



  background(0);
  stroke(255);
  strokeWeight(5);
  noFill();
  circle(width*angleReaderX, height*angleReaderY, angleReaderR);
  stroke(#FF0000);
  line(width*angleReaderX, height*angleReaderY, width*angleReaderX+sin(imuAngle)*(angleReaderR/2-10), height*angleReaderY+cos(imuAngle)*(angleReaderR/2-10));
  textSize(15);
  fill(255);
  text(imuAngle, width*angleReaderX+sin(imuAngle)*(angleReaderR/2+20), height*angleReaderY+cos(imuAngle)*(angleReaderR/2+20));
  stroke(255);
  fill(0);
  arc(width*speedX, height*speedY, 200, 200, PI*13/18, 2*PI+PI*5/18, OPEN);
  stroke(#FF0000);
  line(width*speedX, height*speedY, width*speedX+sin(speedAngle-PI*4/18)*(angleReaderR/2-10), height*speedY+cos(speedAngle-PI*4/18)*(angleReaderR/2-10));
  fill(255);
  text(speedAngle, width*speedX+sin(speedAngle-PI*4/18)*(angleReaderR/2+20), height*speedY+cos(speedAngle-PI*4/18)*(angleReaderR/2+20));
  fill(100);
  noStroke();
  rect(width*motorVoltageX, height*motorVoltageY, windowWidth, 80);
  fill(255);
  stroke(255);
  textSize(40);
  text("Motor Voltage : "+str(motorVoltage) + "V", width*motorVoltageX+10, height*motorVoltageY+50);
  fill(100);
  noStroke();
  rect(width*batteryVoltageX, height*batteryVoltageY, windowWidth, 80);
  fill(255);
  text("Battery Voltage : "+str(batteryVoltage) + "V", width*batteryVoltageX+10, height*batteryVoltageY+50);
  fill(100);
  noStroke();
  rect(width*sysStateX, height*sysStateY, windowWidth, 80);
  fill(255);
  text("System State : "+ sysState, width*sysStateX+10, height*sysStateY+50);
  noStroke();
  fill(100);
  rect(width*sysUptimeX, height*sysUptimeY, windowWidth, 80);
  fill(255);
  text("System Uptime : "+str(sysUptime) + " s", width*sysUptimeX+10, height*sysUptimeY+50);
  stroke(255);
  strokeWeight(2);
  fill(0);
  rect(width*batteryGraphX, height*batteryGraphY, 100, height*(sysUptimeY-batteryGraphY)+80);
  stroke(#00ff00);
  fill(#00ff00);
  batteryGraph = map(constrain(batteryVoltage-14.8, 0, 2), 0, 2, 464, 0);
  rect(width*batteryGraphX, height*batteryGraphY+batteryGraph, 100, height*(sysUptimeY-batteryGraphY)+80-batteryGraph);
  fill(255);
  textSize(20);
  batteryPercentage = map(batteryGraph, 0, 464, 100, 0);
  text(str(batteryPercentage) + "%", width*batteryGraphX+105, height*batteryGraphY+batteryGraph+5);
}
