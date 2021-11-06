// #include <SPI.h>
#include <Wire.h>
#include <cmath>
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
const uint32_t GUI_RATE = 33000; //  30 FPS

// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();

// Initialize TorqueTuner
TorqueTuner knob;

// State flags
int connected = 0;
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



int read_param(float * param, uint8_t*  data, uint8_t length) {
  memcpy(param, data, length);
  if ( std::isnan(*param)) {
    return 1;
  } else {
    return 0;
  }
}

int receiveI2C(TorqueTuner * knob_) {
  Wire.requestFrom(8, I2C_BUF_SIZE + CHECKSUMSIZE);
  uint8_t k = 0;
  while (Wire.available()) {
    rx_data[k] = Wire.read();
    k++;
  }
  if (k != I2C_BUF_SIZE + CHECKSUMSIZE) { // check if all data is recieved
    printf("Error in recieved data. Bytes missing :  %i\n", I2C_BUF_SIZE + CHECKSUMSIZE - k);
    return 1;
  }
  else {
    memcpy(&checksum_rx, rx_data + I2C_BUF_SIZE, 2); // read checksum
    if (checksum_rx != calcsum(rx_data, I2C_BUF_SIZE)) { // error in recieved data
      return 2;
    }
    else { // Succesfull recieve
      memcpy(&knob_->angle, rx_data, 2);
      memcpy(&knob_->velocity, rx_data + 4, 4);
      return 0; //Return 0 if no error has occured
    }
  }
}

void sendI2C(TorqueTuner * knob_) {
  Wire.beginTransmission(8); // transmit to device #8
  memcpy(tx_data, &knob_->torque, 2);
  memcpy(tx_data + 2, &knob_->target_velocity, 4);
  memcpy(tx_data + 6, &knob_->active_mode->pid_mode, 1);
  checksum_tx = calcsum(tx_data, I2C_BUF_SIZE);
  memcpy(tx_data + I2C_BUF_SIZE, &checksum_tx, 2);
  int n = Wire.write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
  Wire.endTransmission();    // stop transmitting
}

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2CUPDATE_FREQ); // Fast mode plus

  // Make a reading for initilization
  int err = 1;
  while (err) {
    err = receiveI2C(&knob);
  }
  knob.set_mode(TorqueTuner::LINSPRING);

  pinMode(SEL_PIN, INPUT);
}

void loop() {

  now = micros();
  if (now - last_time > HAPTICS_UPDATE_RATE) {


    // Recieve Angle and velocity from servo
    err = receiveI2C(&knob);

    if (err) {printf("i2c error \n");}
    else {

      // Update torque if valid angle measure is recieved.
      if (is_playing) {
        knob.update();
      } else {
        // OBS: Consider not updating? assign last last value instead? //
        knob.torque = 0;
        knob.target_velocity = 0;
      }
      sendI2C(&knob);
      float speed = abs(knob.velocity);

    }
    last_time = now;
  }


  /* ------------------------------*/
  /* -------- GUI update  ---------*/
  /* ------------------------------*/

  if (now - last_time_gui > GUI_RATE) {
    // printf("DATAREADY %i, %i \n", knob.angle_out);
    // printf("Target velocity: %f \n", knob.target_velocity);
    last_time_gui = now;
  }
