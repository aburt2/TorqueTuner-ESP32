// #include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <AccelStepper.h>
#include "haptics.h"


// Motor pin definitions:
#define motorPin1  11      // IN1
#define motorPin2  10      // IN2 
#define motorPin3  9     // IN3 
#define motorPin4  8     // IN4 
// Define the AccelStepper interface type; 4 wire motor in full step mode:
#define MotorInterfaceType 4
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

//---------------------------------------------------------------------------
//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement
float previousAngle = 0; //for speed calculation
float timer = 0;
float elapsed_time = 0;
float prevAngle = 0;

// I2C variables
const uint8_t I2C_BUF_SIZE = 10;
const uint8_t CHECKSUMSIZE = 2;
uint8_t tx_data[I2C_BUF_SIZE];
uint8_t rx_data[I2C_BUF_SIZE];
uint16_t checksum_rx = 0;
uint16_t checksum_tx = 0;

// Timing variables
const uint32_t HAPTICS_UPDATE_RATE = 500 ; // 2 KHz
const uint32_t I2CUPDATE_FREQ = 3400000; // high speed mode;
const uint32_t DEBOUNCE_TIME = 10000; // 10 ms
const uint32_t MAINTENANCE_RATE = 30000000; // 30 s
const uint32_t GUI_RATE = 1000000; //  30 s

// Initialize TorqueTuner
TorqueTuner knob;

// State flags
bool is_playing = true;

int pressure = 0;
int sel = 0;
int sel_min = 0;
int sel_max = 0;

// System variables
int err = 0;
int err_count = 0;
uint32_t last_time = 0;
uint32_t last_time_errprint = 0;
uint32_t last_time_maintenance = 0;
uint32_t last_time_gui = 0;
uint32_t now = 0;

uint16_t calcsum(uint8_t buf[], uint8_t length) {
  uint16_t val = 0;
  for (int k = 0; k < length; k++) {
    val += buf[k];
  }
  return val;
}

// Define Pins
int interruptPin = 2;
int switch1 = 0;
int state = 4; //Linear spring

//Set spinning speed
int clockSpeed = 1000;
int pos = 0;
int pos_delta = 0;
float angle_delta = 0;
float accel_v = 0;
float motor_v = 0;

// use 2ms debounce time
#define DEBOUNCE_TICKS (word)microsecondsToClockCycles(15)

extern volatile unsigned long timer0_overflow_count;
word keytick;  // record time of keypress

//int CHANGE_STATE(int cur_state) {
//  int new_state = 0;
//  if (cur_state == 0) {
//    new_state = 1;
//  } 
//  else if (cur_state == 1) {
//    new_state = 2;
//  }
//  else if (cur_state == 2) {
//    new_state = 3;
//  }
//  else if (cur_state == 3) {
//    new_state = 4;
//  }  
//  else if (cur_state == 4) {
//    new_state = 5;
//  }
//  else if (cur_state == 5) {
//    new_state = 6;
//  }
//  else if (cur_state == 6) {
//    new_state = 7;
//  }
//  else if (cur_state == 7) {
//    new_state = 0;
//  }
//  else {
//    state = 0;
//  }
//  return new_state;
//}

int CHANGE_STATE(int cur_state) {
  int new_state = 0;
  if (cur_state == 4) {
    new_state = 7;
  } 
  else if (cur_state == 7) {
    new_state = 4;
  }
  else {
    state = 4;
  }
  return new_state;
}

void KeyPress() {
  keytick=(word)timer0_overflow_count;
}

// returns true if key pressed
boolean KeyCheck() {
  if (keytick!=0) {
    if (((word)timer0_overflow_count-keytick)>DEBOUNCE_TICKS) {
      keytick=0;
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(); //start i2C  
  Wire.setClock(800000L); //fast clock
  knob.set_mode(TorqueTuner::LINSPRING);

  // Pin Mode
  attachInterrupt(digitalPinToInterrupt(interruptPin),KeyPress, FALLING);

  // Check Magnet
  checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  startAngle = ReadRawAngle(); //make a reading so the degAngle gets updated
  prevAngle = startAngle;
  // Set max speed
  stepper.setMaxSpeed(1000);
}

void loop() {
  now = micros();
  elapsed_time = (now - timer)/1000000;
  timer = now;
  
  if (KeyCheck()) {
    state = CHANGE_STATE(state);
    knob.set_mode(state);
  }

  // Recieve Angle and velocity from servo
  degAngle = ReadRawAngle(); //ask the value from the sensor
  knob.angle = correctAngle(degAngle,startAngle)*10; //tare the value
  knob.velocity = (knob.angle-prevAngle)/elapsed_time; // find angular velocity
  prevAngle = knob.angle;
  checkQuadrant(); //check quadrant, check rotations, check absolute angular position
//  Serial.print("Corrected angle: ");
//  Serial.println(knob.angle, 2); //print the corrected/tared angle

  // Update torque if valid angle measure is recieved.
  if (is_playing) {
    knob.update();
  } else {
    // OBS: Consider not updating? assign last last value instead? //
    knob.torque = 0;
    knob.target_velocity = 0;
  }
  if (state == 7) {
    motor_v = knob.target_velocity;
  }
  else {
    motor_v = knob.torque;
  }
  accel_v = motor_v * 32;
  if (accel_v > 1000)  {
    accel_v = 1000;
  }
//  Serial.println(accel_v);
  stepper.setSpeed(accel_v);
  stepper.runSpeed();
  Serial.print("velocity ");
  Serial.println(knob.velocity);
  Serial.print("angle ");
  Serial.println(knob.angle_out);
}

float ReadRawAngle()
{ 
//  Serial.println("Reading Raw Value");
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 
  
//  Serial.print("Deg angle: ");
//  Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  return degAngle;
  
}

float correctAngle(float degAngle,float startAngle)
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
//  Serial.print("Corrected angle: ");
//  Serial.println(correctedAngle, 2); //print the corrected/tared angle
  return correctedAngle;
}

void checkQuadrant()
{
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  
  }
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!");
  //delay(1000);  
}
