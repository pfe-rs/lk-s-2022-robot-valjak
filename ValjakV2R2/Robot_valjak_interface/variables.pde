//Colors

color background = #ffffff;
color stroke = #000000;

color needles = #00aaaa;
color needleTextColor = #646464;

color batteryFullColor = #9AFF9A;
color batteryEmptyColor = #FF9A9A;
color batteryColor;

color windowColor = #e0e0e0;
color textColor = #00aaaa;



float angleReaderX = 0.15;
float angleReaderY = 0.25;
int angleReaderR = 400;



int needleTextSize = 25;
int needleDist = 40;
float speedX = 0.15;
float speedY = 0.75;

float windowX = 0.55;
float windowHeight = 150;
float windowWidth = 800;
float textHeight = 80;

float motorVoltageX = windowX;
float motorVoltageY = 0.15;

float batteryVoltageX = windowX;
float batteryVoltageY = 0.35;

float sysStateX = windowX;
float sysStateY = 0.55;

float sysUptimeX = windowX;
float sysUptimeY = 0.75;


float batteryEmpty = 14.8;
float batteryFull = 16.8;
float batteryGraphX = 0.35;
float batteryGraphY = 0.15;
float batteryGraph = 0;
float batteryGraphWidth = 200;


float imuAngle;
float speedAngle;
float speedAngleT;
float motorVoltage;
float batteryVoltage;
float batteryPercentage;
int sysStateInt;
String sysState[] = {"Running", "Stopped", "Error"};
String sysUptimeS;
float sysUptime;



String inBuffer;
boolean single = true;
int ln = 13;
int cr = 10;
String list[] = {"1.00", "1.00", "12", "15", "0", "2500"};
float[] listF = {0, 0, 0, 0, 0, 0};


float controlerMax = 15.0;
float dataOut = 0;
