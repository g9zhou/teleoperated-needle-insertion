// ME 327 Assignment 5 Remote Teleoperation Code
// 5/27/2020 Zonghe Chua + 4/24/2022 Nathan Kau

//-------------------------
// Parameters that define what environment to render
#define BILATERAL
//#define FEEDBACK
#define RESTRICTION


// Includes
#include <math.h>
#include <AS5048A.h>

// the sensor CSn pin is connected to pin 10
AS5048A angleSensor(10);

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor
int clutchPinOut = 3;
int clutchPinIn = 4;

int scalePinOut = 6;
int scalePinIn = 7;

// Position tracking variables
//------------------------------------------------------------------------------------------

//Leader variables
long updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 15000;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 15000;
double OFFSET_NEG = 15;

boolean clutch_stat = false;
boolean clutch_stat_prev = false;
boolean scale_stat = false;
boolean scale_stat_prev = false;

// Kinematic variables
//------------------------------------------------------------------------------------------

// Leader Variables
float xh = 0;           // position of the handle [m]
float theta_s = 0;      // Angle of the sector pulley in deg
float xh_prev;          // Distance of the handle at previous time step
float xh_prev2;
float dxh;              // Velocity of the handle
float dxh_prev;
float dxh_prev_prev;
float dxh_filt;         // Filtered velocity of the handle
float dxh_filt_prev;
float dxh_filt_prev_prev;
float hold_pos = 0;
float hold_pos_prev = 0;
float scale_pos = 0;
float descale_pos = 0;
double xh_min = -0.045;
double xh_max = 0.15;
double scale_dist = 0;

// Follower variables
float f_remote = 0;
float f_remote_prev = 0;
float df_remote_prev = 0;

// Special variables for efficient transmission over serial
// We use a union so that the binary form of the integer shares the same memory space as the int form
 
typedef union {
 int integer;
 byte binary[2];
} binaryInt; 

binaryInt xh_bin; 
binaryInt f_remote_bin;

// Force output variables
// ----------------------------------------------------------------------------------------------------

// Leader Variables
float force = 0;           // force at the handle
float Tp = 0;              // torque of the motor pulley
float duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

//Serial communication variables
bool sending = true; // flip the tx/rx flags
bool receiving = false;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input
  pinMode(clutchPinIn,INPUT);     // set Clutch pin to be an input
  pinMode(scalePinIn,INPUT);

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  pinMode(clutchPinOut,OUTPUT);
  pinMode(scalePinOut,OUTPUT);
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  angleSensor.init();
  // Initialize position valiables
  angleSensor.getRawRotation(); // take reading in case first reading is spurious
  lastLastRawPos = angleSensor.getRawRotation(); // position from Hall Effect sensor
  lastRawPos = angleSensor.getRawRotation(); // position from Hall Effect sensor

  // initialize the values for the follower kinematics
  xh_bin.integer = 0;
  f_remote_bin.integer = 0;
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Leader
  //-------------------------------------------------------------------------------------------------------
  // Get voltage output by MR sensor
  rawPos = int(angleSensor.getRawRotation());  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  updatedPos = rawPos + flipNumber*OFFSET; // need to update pos based on what most recent offset is 

  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE
  digitalWrite(clutchPinOut,HIGH);
  digitalWrite(scalePinOut,HIGH);
  clutch_stat = digitalRead(clutchPinIn);
  scale_stat = digitalRead(scalePinIn);
  // Define kinematic parameters you may need
  double rh = 0.09;   //[m]
  // Step B.1: print updatedPos via serial monitor
//  Serial.println(updatedPos);
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = -0.002017257801552*updatedPos -  0.126903854424927;
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  xh = ts/180*M_PI*rh;

//  if (abs(xh - xh_prev) >= 0.02) {
//    xh = xh_prev;
//  }

  xh -= scale_dist;
  if (scale_stat == true) {
    if (scale_stat_prev == false){
      scale_pos = xh_prev;
    }
    xh = scale_pos - hold_pos + (xh + hold_pos - scale_pos)/2.0;
  } else if (scale_stat == false) {
    if (scale_stat_prev == true) {
      descale_pos = xh_prev;
      scale_dist += (descale_pos-scale_pos);
    }
  }
  scale_stat_prev = scale_stat;
  
  if (clutch_stat == false){
    if (clutch_stat_prev == true){
      hold_pos -= xh;
    }
    xh += hold_pos;
  } else if (clutch_stat == true) {
    xh = xh_prev;
    if (clutch_stat_prev == false) {
//      hold_pos_prev = hold_pos;
      hold_pos = xh_prev;
    }
  }
//  Serial.println(xh);
  clutch_stat_prev = clutch_stat;

  // Calculate velocity with loop time estimation
  dxh = (float)(xh - xh_prev) / 0.001;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = 0.9*dxh + 0.1*dxh_prev; 
    
  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;
  
  dxh_prev_prev = dxh_prev;
  dxh_prev = dxh;
  
  dxh_filt_prev_prev = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;

  //*************************************************************
  //** Teleop Communication. Send and receive handle positions.**
  //*************************************************************
  //--------------------------------------------------------------------------------------------------------------------
  if (sending && Serial.availableForWrite()> sizeof(int)){ //check that we have space in the serial buffer to write
    xh_bin.integer = int(xh*100000.0); // save space by using a integer representation
    Serial.write(xh_bin.binary,2); // write the integer to serial
    sending = false; // flip the tx/rx flags
    receiving = true;
    Serial.flush(); // flush the serial for good measure
  }
//   read our follower position
  if (receiving && Serial.available()> 1){ //if there is at least 2 bytes of data to read from the serial
    f_remote_prev = f_remote; // backup old follower force
    Serial.readBytes(f_remote_bin.binary,2); // read the bytes in
    f_remote = (float)f_remote_bin.integer/100000.0; // convert the integer back into a float
    if (isnan(f_remote)) //if xh_2 is corrupt, just use the old value
      f_remote = f_remote_prev;
    sending = true; // flip the tx/rx flags
    receiving = false;
    Serial.flush(); // flush the serial for good measure
  }
//  while (Serial.available()) { // If data is available to read,
//    f_remote_prev = f_remote;
//    String f_remote_string = Serial.readStringUntil('\n'); // read it and store it in val
//    f_remote_string.trim();
//    f_remote = f_remote_string.toFloat();
//    if (isnan(f_remote)) {
//      f_remote = f_remote_prev;
//    }
//    Serial.flush();
//  }
  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  
#ifdef BILATERAL
//  force = f_remote;
  double kp = 40;
  double kd = 0.0;
  float df_remote = (float)(f_remote - f_remote_prev) / 0.001;
  float df_remote_filt = 0.8*df_remote + 0.2*df_remote_prev;
  force = kp*(f_remote-xh)*(xh > f_remote)+kd*(df_remote_filt-dxh_filt);
  df_remote_prev = df_remote_filt;
#endif

//#ifdef FEEDBACK
//  force = -f_remote*200+3;
//#endif

#ifdef RESTRICTION
  if (xh >= xh_max) {
    force += -500*(xh-xh_max);
  } else if (xh <= xh_min) {
    force += -500*(xh-xh_min);
  } else {
  force += 0;
  }
#endif

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  // ADD YOUR CODE HERE (Use your code from Assignment 4)
  // Define kinematic parameters you may need
  double rp = 0.005;   //[m]
  double rs = 0.074;   //[m] 
  // Step C.1: force = ?; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics
  Tp = force*rh/rs*rp;
//  Serial.println(force);
  
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************

  // Leader
  //------------------------------------------------------------------------------
  // Determine correct direction for motor torque
  if(force < 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal
    
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
