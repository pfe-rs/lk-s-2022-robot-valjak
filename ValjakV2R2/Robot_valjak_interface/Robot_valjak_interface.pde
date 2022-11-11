import processing.serial.*;
Serial port;

import org.gamecontrolplus.gui.*;
import org.gamecontrolplus.*;
import net.java.games.input.*;

ControlIO control;
Configuration config;
ControlDevice gpad;


boolean demo = true;
boolean controlerPresent = true;



void setup() {
  //fullScreen();
  size(1000, 500);
  surface.setResizable(true);
  
  System.setProperty("net.java.games.input.useDefaultPlugin", "false");
  
  control = ControlIO.getInstance(this);
  gpad = control.filter(1).getMatchedDevice("speed");

  //controller check
  if (gpad == null) {
    println("No controller detected, arrow keys only");
    controlerPresent = false;
  }

  
  if (!demo) {
    String portName = "COM8";
    port = new Serial(this, portName, 9600);
    port.clear();
    // Throw out the first reading, in case we started reading
    // in the middle of a string from the sender.
  };


  frameRate(1000);
}

void draw() {
  getData();
  getControlerInput();
  getArrowKeys();
  println(dataOut);
  port.write(str(dataOut));
  port.write('\n');
  drawScreen();
}
