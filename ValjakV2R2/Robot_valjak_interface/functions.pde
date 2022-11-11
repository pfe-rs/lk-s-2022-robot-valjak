void getData() {
  if (!demo) {
    if (port.available() > 0) {
      inBuffer = port.readStringUntil(cr);
      if (inBuffer != null) {
        println(inBuffer);
        list = split(inBuffer, ',');
        if (list.length == 6) {
          for (int i = 0; i<6; i++) {
            listF[i] = float(list[i]);
          }
        };
      }
    }
  }

  if (!demo) {
    imuAngle = constrain(listF[0], -PI, PI);
    speedAngleT = -constrain(listF[1]/14.3, -PI*28/18, PI*28/18);
    speedAngle = -abs(speedAngleT);
    motorVoltage = listF[2];
    batteryVoltage = listF[3];
    sysStateInt = int(listF[4]);
    sysUptime = listF[5];
  };

  if (demo) {
    float alpha = 0.92;
    float alpha2 = 0.7;

    imuAngle = imuAngle*alpha + (constrain(abs(millis()/300.0%(4*PI/6)-2*PI/6)-PI/6, -PI/6, PI/6))*(1-alpha);
    speedAngle = speedAngle*alpha + (-constrain(millis()/3000.0%(PI*14/36), 0, PI*28/18))*(1-alpha);
    speedAngleT = abs(speedAngle);
    motorVoltage =  millis()%20000/10/100.0;
    batteryVoltage = round((batteryVoltage*alpha2+(((abs(millis()/2%4000-2000)+14800)/10)/100.0)*(1-alpha2))*100)/100.0;
    sysStateInt = millis()/1000%3;
    sysUptime = millis()/10/100.0;
  };
}


void getControlerInput() {
  if (controlerPresent) {
    dataOut = map(gpad.getSlider("stick1").getValue(), -1, 1, -controlerMax, controlerMax);
    if(gpad.getButton("RL").pressed()){dataOut *= 2;}
    if(gpad.getButton("RR").pressed()){dataOut *= 2;}
  }
}



void getArrowKeys() {
  if (keyPressed) {
    switch(key) {
    case '1':
      dataOut = -10;
      break;
    case '2':
      dataOut = -20;
      break;
    case '3':
      dataOut = -30;
      break;
    case '7':
      dataOut = 10;
      break;
    case '8':
      dataOut = 20;
      break;
    case '9':
      dataOut = 30;
      break;
    case '5':
      dataOut = 0;
      break;
    case '0':
      dataOut = 0;
      break;
    }
  }
}


void drawScreen() {
  background(background);

  stroke(stroke);
  strokeWeight(5);
  noFill();
  circle(width*angleReaderX, height*angleReaderY, angleReaderR);
  stroke(needles);
  line(width*angleReaderX, height*angleReaderY, width*angleReaderX+sin(imuAngle)*(angleReaderR/2-10), height*angleReaderY+cos(imuAngle)*(angleReaderR/2-10));
  textSize(needleTextSize);
  fill(needleTextColor);
  text(imuAngle, width*angleReaderX+sin(imuAngle)*(angleReaderR/2+needleDist)-needleDist/2, height*angleReaderY+cos(imuAngle)*(angleReaderR/2+needleDist)+needleDist/2);

  stroke(stroke);
  fill(background);
  arc(width*speedX, height*speedY, angleReaderR, angleReaderR, PI*13/18, 2*PI+PI*5/18, OPEN);
  stroke(needles);
  line(width*speedX, height*speedY, width*speedX+sin(speedAngle-PI*4/18)*(angleReaderR/2-10), height*speedY+cos(speedAngle-PI*4/18)*(angleReaderR/2-10));
  fill(needleTextColor);
  text(speedAngleT*14.3, width*speedX+sin(speedAngle-PI*4/18)*(angleReaderR/2+needleDist)-needleDist/2, height*speedY+cos(speedAngle-PI*4/18)*(angleReaderR/2+needleDist)+needleDist/2);

  textSize(textHeight);

  fill(windowColor);
  noStroke();
  rect(width*motorVoltageX, height*motorVoltageY, windowWidth, windowHeight);
  fill(textColor);
  stroke(stroke);
  text("Motor Voltage : "+str(motorVoltage) + "V", width*motorVoltageX+10, height*motorVoltageY+(textHeight/2+windowHeight)/2);

  fill(windowColor);
  noStroke();
  rect(width*batteryVoltageX, height*batteryVoltageY, windowWidth, windowHeight);
  fill(textColor);
  text("Battery Voltage : "+str(batteryVoltage) + "V", width*batteryVoltageX+10, height*batteryVoltageY+(textHeight/2+windowHeight)/2);

  fill(windowColor);
  noStroke();
  rect(width*sysStateX, height*sysStateY, windowWidth, windowHeight);
  fill(textColor);
  text("System State : "+ sysState[sysStateInt], width*sysStateX+10, height*sysStateY+(textHeight/2+windowHeight)/2);

  if (sysUptime >= 1000) {
    sysUptimeS = str(round(sysUptime));
  } else if (sysUptime >= 100) {
    sysUptimeS = str(round(sysUptime*10)/10.0);
  } else {
    sysUptimeS = str(sysUptime);
  };

  noStroke();
  fill(windowColor);
  rect(width*sysUptimeX, height*sysUptimeY, windowWidth, windowHeight);
  fill(textColor);
  text("System Uptime : "+ sysUptimeS + " s", width*sysUptimeX+10, height*sysUptimeY+(textHeight/2+windowHeight)/2);

  stroke(stroke);
  strokeWeight(2);
  fill(background);
  rect(width*batteryGraphX, height*batteryGraphY, batteryGraphWidth, height*(sysUptimeY-batteryGraphY)+windowHeight);
  batteryGraph = map(constrain(batteryVoltage-batteryEmpty, 0, 2), 0, 2, height*(sysUptimeY-batteryGraphY)+windowHeight, 0);
  batteryPercentage = map(batteryGraph, 0, height*(sysUptimeY-batteryGraphY)+windowHeight, 1, 0);
  int R = int(red(batteryFullColor)*batteryPercentage+red(batteryEmptyColor)*(1-batteryPercentage));
  int G = int(green(batteryFullColor)*batteryPercentage+green(batteryEmptyColor)*(1-batteryPercentage));
  int B = int(blue(batteryFullColor)*batteryPercentage+blue(batteryEmptyColor)*(1-batteryPercentage));
  batteryColor = color(R, G, B);
  stroke(batteryColor);
  fill(batteryColor);
  rect(width*batteryGraphX, height*batteryGraphY+batteryGraph, batteryGraphWidth, height*(sysUptimeY-batteryGraphY)+windowHeight-batteryGraph);
  fill(needleTextColor);
  textSize(20);
  text(str(round(batteryPercentage*10000)/100.0) + "%", width*batteryGraphX+batteryGraphWidth + 5, height*batteryGraphY+batteryGraph+5);
}
