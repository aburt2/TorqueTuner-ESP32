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


 #define TSTICKJOINT 1

const int SEL_PIN = 0;

#ifdef TSTICKJOINT
const int SDA_PIN = 26;
const int SCL_PIN = 25;
#else
const int SDA_PIN = 21;
const int SCL_PIN = 22;
#endif

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
int analogPin = 1;
int interruptPin = 2;
int val = 0;     // variable to store the read value
int switch1 = 0;
int state = 0;

//Set spinning speed
int clockSpeed = 1000;
int pos = 0;
int pos_delta = 0;
float angle_delta = 0;

// use 2ms debounce time
#define DEBOUNCE_TICKS (word)microsecondsToClockCycles(15)

extern volatile unsigned long timer0_overflow_count;
word keytick;  // record time of keypress

int CHANGE_STATE(int cur_state) {
  int new_state = 0;
  Serial.print("Current State: ");
  Serial.println(cur_state);
  if (cur_state == 0) {
    new_state = 1;
  } 
  else if (cur_state == 1) {
    new_state = 2;
  }
  else if (cur_state == 2) {
    new_state = 3;
  }
  else if (cur_state == 3) {
    new_state = 4;
  }  
  else if (cur_state == 4) {
    new_state = 5;
  }
  else if (cur_state == 5) {
    new_state = 6;
  }
  else if (cur_state == 6) {
    new_state = 7;
  }
  else if (cur_state == 7) {
    new_state = 0;
  }
  else {
    state = 0;
  }
  
  
  Serial.print("New State: ");
  Serial.println(new_state);
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

//  Wire.begin(SDA_PIN, SCL_PIN);
//  Wire.setClock(I2CUPDATE_FREQ); // Fast mode plus
  knob.set_mode(TorqueTuner::MAGNET);

  // Pin Mode
  attachInterrupt(digitalPinToInterrupt(interruptPin),KeyPress, FALLING);
}

void loop() {
  val = analogRead(analogPin);
  val = val * 3600/1023;
  now = micros();
  if (KeyCheck()) {
    state = CHANGE_STATE(state);
    knob.set_mode(state);
  }

  // Recieve Angle and velocity from servo
  pos_delta = pos - stepper.currentPosition();
  pos = stepper.currentPosition();
//  Serial.print("Current Position: ");
//  Serial.println(pos);
//  angle_delta = pos_delta*(3600/200);
  angle_delta = 18;
  knob.angle = knob.angle + angle_delta;

  // Update torque if valid angle measure is recieved.
  if (is_playing) {
    knob.update();
  } else {
    // OBS: Consider not updating? assign last last value instead? //
    knob.torque = 0;
    knob.target_velocity = 0;
  }
  float motor_v = knob.target_velocity;
  stepper.setSpeed(motor_v);
//    stepper.runSpeed();

//  if (now - last_time_gui > GUI_RATE) {
//   Serial.print("Angle Change: ");
//   Serial.println(angle_delta);
//   Serial.print("Current Angle: ");
//   Serial.println(knob.angle_out);
//   Serial.print("Target velocity: ");
//   Serial.println(knob.torque);
//   Serial.println("");
//  last_time_gui = now;
//  }
 Serial.print("Index: ");
 Serial.println(knob.magnet.idx);
 Serial.print("Current Angle: ");
 Serial.println(knob.angle_out);
 Serial.print("Target velocity: ");
 Serial.println(knob.torque);
 Serial.println("");
 delay(500);
}
