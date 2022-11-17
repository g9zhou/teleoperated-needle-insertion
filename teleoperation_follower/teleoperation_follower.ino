// ME 327 Assignment 5 Remote Teleoperation Code
// 5/27/2020 Zonghe Chua + 4/24/2022 Nathan Kau

//-------------------------
// Parameters that define what environment to render

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

//#define FORCE_SENSOR_PIN A0 // the FSR and 10K pulldown are connected to A0
//double flt_force;
//double flt_force_prev = 0;

// Position tracking variables
//------------------------------------------------------------------------------------------

//Follower variables
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
const int flipThresh = 750;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 920;
double OFFSET_NEG = 15;

// Kinematic variables
//------------------------------------------------------------------------------------------

// Leader Variables
float xt = 0;           // position of the handle [m]
float theta_s = 0;      // Angle of the sector pulley in deg
float xt_prev;          // Distance of the handle at previous time step
float xt_prev2;
float dxt;              // Velocity of the handle
float dxt_prev;
float dxt_prev_prev;
float dxt_filt;         // Filtered velocity of the handle
float dxt_filt_prev;
float dxt_filt_prev_prev;

// Follower variables
float xh_remote = 0;
float xh_remote_prev = 0;
float dxh_remote_prev = 0;
double m = -0.01593740670235;
double b = 1.230078026390457;
double rh = 0.1;   //[m]
double ll = 0.12;
double ln = 0.0;
double xh_min = -0.045;
double xh_max = 0.15;

// Special variables for efficient transmission over serial
// We use a union so that the binary form of the integer shares the same memory space as the int form
 
typedef union {
 int integer;
 byte binary[2];
} binaryInt; 

binaryInt xt_bin; 
binaryInt xh_remote_bin;

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

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);

  // initialize the values for the follower kinematics
  xt_bin.integer = 0;
  xh_remote_bin.integer = 0;
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
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
//  Serial.println(rawPos);

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
//  int analogReading = analogRead(FORCE_SENSOR_PIN);
//  double volt = analogReading/1023.0*5.0;
//  flt_force = 0.5*volt+0.5*flt_force_prev;
//  flt_force_prev = flt_force;
//  Serial.println(flt_force);
  // ADD YOUR CODE HERE
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = m * updatedPos +  b;
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  double xt = ln + rh*sin(ts/180*M_PI)+sqrt(ll*ll-rh*rh*pow(cos(ts/180*M_PI),2))-0.066332495807108;
//  double xt = ts/180*M_PI*rh;
//  Serial.println(updatedPos);


  // Calculate velocity with loop time estimation
  dxt = (float)(xt - xt_prev) / 0.001;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxt_filt = 0.9*dxt + 0.1*dxt_prev; 
    
  // Record the position and velocity
  xt_prev2 = xt_prev;
  xt_prev = xt;
  
  dxt_prev_prev = dxt_prev;
  dxt_prev = dxt;
  
  dxt_filt_prev_prev = dxt_filt_prev;
  dxt_filt_prev = dxt_filt;

  //*************************************************************
  //** Teleop Communication. Send and receive handle positions.**
  //*************************************************************
  //--------------------------------------------------------------------------------------------------------------------
  if (sending && Serial.availableForWrite()> sizeof(int)){ //check that we have space in the serial buffer to write
    xt_bin.integer = int(xt*100000.0); // save space by using a integer representation
//    xt_bin.integer = int(flt_force*1000.0);
    Serial.write(xt_bin.binary,2); // write the integer to serial
    sending = false; // flip the tx/rx flags
    receiving = true;
    Serial.flush(); // flush the serial for good measure
  }
//   read our follower position
  if (receiving && Serial.available()> 1){ //if there is at least 2 bytes of data to read from the serial
    xh_remote_prev = xh_remote; // backup old follower force
    Serial.readBytes(xh_remote_bin.binary,2); // read the bytes in
    xh_remote = (float)xh_remote_bin.integer/100000.0; // convert the integer back into a float
    if (isnan(xh_remote)) {//if xh_2 is corrupt, just use the old value
      xh_remote = xh_remote_prev;
    }
    sending = true; // flip the tx/rx flags
    receiving = false;
    Serial.flush(); // flush the serial for good measure
  }
//  while (Serial.available()) { // If data is available to read,
//    xh_remote_prev = xh_remote;
//    String xh_remote_string = Serial.readStringUntil('\n'); // read it and store it in val
//    xh_remote_string.trim();
//    float xh_remote = xh_remote_string.toFloat();
//    if (isnan(xh_remote)) {
//      xh_remote = xh_remote_prev;
//    }
//    Serial.println(xh_remote,5);
//  }
  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************


  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  // ADD YOUR CODE HERE (Use your code from Assignment 4)
  // Define kinematic parameters you may need
  double rp = 0.005;   //[m]
  double rs = 0.074;   //[m] 
  if (xh_remote <= -0.0463) {
    xh_remote = xh_min;
  } 
  if (xh_remote >= 0.1537) {
    xh_remote = xh_max;
  }
//  Serial.print(rawPos);
//  Serial.print(',');
//  Serial.println(updatedPos);
//  Serial.println(xt,5);
  // Step C.1: force = ?; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  float dxh_remote = (float)(xh_remote - xh_remote_prev) / 0.001;
  float dxh_remote_filt = 0.8*dxh_remote + 0.2*dxh_remote_prev;
  force = 30*(xh_remote-xt)+0.02*(dxh_remote_filt-dxt_filt);
  dxh_remote_prev = dxh_remote_filt;
//  force = 30*(xh_remote-xt);
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics
  Tp = force*rh/rs*rp;
  
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
