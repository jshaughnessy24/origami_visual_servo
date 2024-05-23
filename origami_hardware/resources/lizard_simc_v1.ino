/**
   To be uploaded on motor driver module boards - Handle motor motion and control internally
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <PID_v1.h>

#define ADDRESS 0x0A // SMART MOTOR DRIVER I2C ADDRESS
#define NSLEEP 5
#define M1 9 // MOTOR PINS
#define M2 10
#define CURRENT_SENSOR 17 // CURRENT SENSOR PIN
#define ENCODER1 2        // ENCODER PINS
#define ENCODER2 3

#define PID_SAMPLE_TIME 10 // PID SAMPLE TIME (IN MS)

#define CURRENT_THRESHOLD 0.0 // CURRENT THRESHOLD FOR MOTOR STALL //TODO: SET THRESHOLD FOR MOTOR STALL CURRENT

// MOTOR PROPERTIES
#define MAX_PWM 255              // MAXIMUM PWM VALUE
#define GEAR_RATIO 110           // MOTOR GEAR RATIO (110:1 OR 150:1)
#define ENCODER_TICKS_PER_REV 12 // NO OF HIGH PULSES PER ROTATION
#define ENCODER_TICKS_PER_MS_TO_RPM (1000.0 * 60.0) / (double)(ENCODER_TICKS_PER_REV * GEAR_RATIO) // MULTIPLIER TO CONVERT FROM ENCODER TICKS PER MS TO RPM

#define COMMAND_BYTES 3 // NUMBER OF MOTOR COMMAND BYTES
#define RETURN_BYTES 8 // NUMBER OF RETURN DATA BYTES

// MOTOR DRIVER CONTROL MODES
#define POSITION 0
#define VELOCITY 1

// CURRENT SENSOR VARIABLES
int16_t motorCurrent = 0; // returnData variable representing motor current sent through I2C to mainboard

// MOTOR CONTROL VARIABLES
uint8_t controlMode = POSITION;      // MOTOR CONTROL MODE
uint8_t motorCommand[COMMAND_BYTES]; // MOTOR COMMAND RECEIVED FROM CONTROLLER

uint8_t returnData[RETURN_BYTES]; // RETURN DATA SENT TO THE CONTROLLER UPON REQUEST

double encoderCount = 0.0; // CURRENT ENCODER COUNT
double lastCount = 0.0;    // PREVIOUS ENCODER COUNT
double countDiff = 0.0;    // DIFFERENCE IN ENCODER COUNT FROM lastCount TO CURRENT encoderCount
long now = 0;              // TIME OF PREVIOUS ENCODER READING
long lastTime = 0;         // TIME OF PREVIOUS ENCODER READING
double dt = 0.0;

// PID PARAMETERS
double Kp = 0.5, Ki = 0.005, Kd = 0.005;
double target[2];     // TARGET POSITION AND SPEED
double effort = 0.0;  // CONTROL EFFORT
double dEffort = 0.0; // CONTROL EFFORT
double speed = 0.0;   // MOTOR SPEED IN RPM
PID pid[2] = {
  PID(&encoderCount, &effort, &target[POSITION], Kp, Ki, Kd, DIRECT), // POSITION PID
  // PID(&encoderCount, &effortof, &target[POSITION], Kp, Ki, Kd, REVERSE), // POSITION PID
  PID(&speed, &dEffort, &target[VELOCITY], Kp, Ki, Kd, DIRECT)         // VELOCITY PID
};

void setup() {
  // INIT I2C W/ GIVEN ADDRESS
  Wire.begin(ADDRESS);

  // SET SENSOR PINS
  pinMode(CURRENT_SENSOR, INPUT);
  pinMode(ENCODER1, INPUT_PULLUP);
  pinMode(ENCODER2, INPUT_PULLUP);

  // SET OUTPUT PINS
  pinMode(NSLEEP, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // ATTACH INTERRUPTS FOR ENCODER READINGS
  attachInterrupt(digitalPinToInterrupt(ENCODER1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2), isr2, CHANGE);

  // SET I2C CALLBACKS
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);

  digitalWrite(NSLEEP, HIGH); //  NSLEEP SHOULD BE KEPT HIGH FOR NORMAL OPERATION OF THE MD9927 MOTOR DRIVER

  pid[POSITION].SetMode(AUTOMATIC);                                      // SET POSITION PID MODE
  pid[POSITION].SetOutputLimits((double)-MAX_PWM, (double)MAX_PWM); // CONSTRAIN POSITION PID OUTPUT
  pid[POSITION].SetSampleTime(PID_SAMPLE_TIME);                          // CONSTRAIN POSITION PID OUTPUT
  pid[VELOCITY].SetMode(AUTOMATIC);                                      // SET VELOCITY PID MODE
  pid[VELOCITY].SetOutputLimits((double)-MAX_PWM, (double)MAX_PWM); // CONSTRAIN VELOCITY PID OUTPUT
  pid[VELOCITY].SetSampleTime(PID_SAMPLE_TIME);                          // CONSTRAIN POSITION PID OUTPUT
}

void loop() {
  now = millis();
  dt = now - lastTime;

  // CALCULATE EFFORT, READ CURRENT, AND CALCULATE SPEED EVERY 10 MS
  if (dt >= PID_SAMPLE_TIME) {
    // UPDATE SPEED AND CURRENT
    calcSpeed();
    readCurrent();
    
    // CALCULATE MOTOR EFFORT
    pid[controlMode].Compute(); 
    if (controlMode == VELOCITY) effort += dEffort;
    effort = constrain(effort, -MAX_PWM, MAX_PWM);
    effort = round(effort);

    lastCount = encoderCount;
    lastTime = now; // UPDATE TIMER

    drive(effort); // SET MOTOR EFFORT
  }
}

// ISRs split up for each hall-effect latch sensor to decrease processing time within the ISR and allow I2C to work every time without interruption

/**
   ISR for encoder 1 - reads registers instead of digital read for increased speed and increments/decrements a encoderCount variable
*/
void isr1() {
  uint8_t encread1 = PIND >> 2 & 0x01;
  uint8_t encread2 = PIND >> 3 & 0x01; // GET THE TWO BIT ENCODER READ
  if (encread1 == encread2) encoderCount++;
  else encoderCount--;
}

/**
   ISR for encoder 2 - reads registers instead of digital read for increased speed and increments/decrements a encoderCount variable
*/
void isr2() {
  uint8_t encread1 = PIND >> 2 & 0x01;
  uint8_t encread2 = PIND >> 3 & 0x01; // GET THE TWO BIT ENCODER READ
  if (encread1 != encread2) encoderCount++;
  else encoderCount--;
}

/**
   Function to set the motorCurrent variable to the current sensor reading and check if it is stalled
*/
void readCurrent() {
  motorCurrent = analogRead(CURRENT_SENSOR);
}

void calcSpeed() {
  countDiff = encoderCount - lastCount;                   // CHANGE IN ENCODER ENCODERCOUNT
  speed = (countDiff / dt) * ENCODER_TICKS_PER_MS_TO_RPM; // CALCULATE MOTOR RPM
}

/**
   Function to home the cables using the current threshold
   Once homed, the motors reverse to limit current
*/
void homeCables() {
  if (motorCurrent >= CURRENT_THRESHOLD) {
    reverse(150);
    // TODO: RESET ENCODER COUNTER
  }
}

/**
   This function drives the motors forward
   @param effort - the integer speed of the motor
*/
void forward(int16_t effort) {
  analogWrite(M1, 0);
  analogWrite(M2, effort);
}

/**
   This function drives the motors reverse
   @param effot - the integer speed of the motor
*/
void reverse(int16_t effort) {
  analogWrite(M1, effort);
  analogWrite(M2, 0);
}

/**
   This function stops the motors
*/
void brake() {
  analogWrite(M1, 255);
  analogWrite(M2, 255);
}

void drive(int16_t effort) {
  // TODO: READ CURRENT AND BRAKE IF STALLED
  // readCurrent();
  if (effort > 0) forward(effort);
  else if (effort < 0) reverse(-effort);
  else brake();
}

/**
   Callback function upon receiving a request for returnData via I2C from master
   This will request a set number of bytes as a message that will be formed for interpretation by the mainboard
*/
void onRequest() {
  // readCurrent();
  returnData[0] = ((long)encoderCount >> 24); // SPLIT ENCODER COUNT INTO 4 BYTES
  returnData[1] = ((long)encoderCount >> 16);
  returnData[2] = ((long)encoderCount >> 8);
  returnData[3] = ((long)encoderCount) & 0xFF;
  returnData[4] = ((int16_t)speed >> 8); // SPLIT MOTOR VELOCITY INTO 2 BYTES
  returnData[5] = (int16_t)speed & 0xFF;
  // returnData[6] = ((int16_t)effort >> 8);
  // returnData[7] = (int16_t)effort & 0xFF;
  returnData[6] = (motorCurrent >> 8); // SPLIT MOTOR CURRENT INTO 2 BYTES
  returnData[7] = motorCurrent & 0xFF;
  Wire.write(returnData, RETURN_BYTES); // SEND DATA
}

/**
   callback function for recieving messages and setting the appropriate I2C status
*/
void onReceive(int numBytes) {
  // READ DATA ONE BYTE AT A TIME
  int index = 0;
  while (Wire.available() > 0) motorCommand[index++] = Wire.read();

  controlMode = motorCommand[0];
  int16_t val = (motorCommand[1] << 8) | motorCommand[2];
  target[controlMode] = (double)val; // SET THE TARGET VALUE (SPEED OR POSITION)
}
