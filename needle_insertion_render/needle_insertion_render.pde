import processing.serial.*;

// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
  -A wider line was used. strokeWeight(4);
  -Continuous line instead of vertical lines.
  -Bigger Window size 600x400.
-------------------------------------------------------------------------------
This program takes ASCII-encoded strings
from the serial port at 9600 baud and graphs them. It expects values in the
range 0 to 1023, followed by a newline, or newline and carriage return


*/

import processing.serial.*;

Serial myPort;        // The serial port

//initialize variables
float inByte = 0;
float lastByte = 0;
Float userPos = 0.0;

void setup () {
  // set the window size:
  size(1000, 600);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[2], 115200);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  stroke(127,34,255);     //stroke color
  strokeWeight(2);        //stroke wider
  
  // START EDITING HERE
  
  // Virtual Wall
  // map the wall position from units of Arduino simulation to the screen width.
  // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels
  // draw the wall as a line
  float fatpos = map(-0.03,-0.052,0.07,400,1000);
  float mempos = map(0.02,-0.052,0.07,400,1000);
  float livpos = map(0.03,-0.052,0.07,400,1000);
  line(fatpos,0,fatpos,600);
  //line(mempos,0,mempos,600);
  float deform = max(mempos,inByte);
  if (inByte >= livpos) {
    line(mempos,0,mempos,295);
    line(mempos,305,mempos,600);
  } else {
    line(deform,300,mempos,600);
    line(deform,300,mempos,0);
  }
  line(inByte-600,300,inByte,300);
  rect(inByte-800,290,200,20);
  textSize(32);
  text("fat",fatpos+100,200);
  fill(0,408,612);
  text("membrane",mempos-70,400);
  fill(0,408,612);
  text("liver",livpos,200);
  fill(0,408,612);
}

void serialEvent (Serial myPort) {
  // get the ASCII string:

  // read the input string
  // HINT: use myPort.readStringUntil() with the appropriate argument
  // trim and convert string to a number
  // if: the number is NaN, set current value to previous value
  // otherwise: map the new value to the screen width
  //           & update previous value variable
  String string = myPort.readStringUntil('\n');
  string = trim(string);
  Float pos = float(string);
  if (Float.isNaN(pos)) {
    inByte = lastByte;
  } else {
    inByte = map(pos,-0.052,0.07,400,1000);
    lastByte = inByte;
  }
  println(pos);
  //STOP EDITING HERE
}
