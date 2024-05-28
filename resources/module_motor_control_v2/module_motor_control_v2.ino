#include <Wire.h>
#include <Arduino.h>

const int numMotors = 3;
const int motorAddresses[numMotors] = {1,2,3}; // MOTOR ADDRESSES, transmits to addresses 1, 2, 3 respectively. Must change if different addresses used

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
int motorCommands[2*numMotors];

int sentCmd[numMotors];

long motorSpeed = 0;

int bytes[8];

char *token;

boolean newData = false;

String measuredOutputsStr = "";

/**
   sets up wire and serial
*/
void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
}

/**
    loops indefinitely
    - receives commands
    - transmits commands to motors
   - reads measured data from motors
   - prints measured data to serial
*/
void loop() {
  recvWithEndMarker();  //receives commands
  showNewData();

  /*
   * Send motor commands to respective motors
   */
  for(int i = 0; i < numMotors; i++) {
    setRPM(motorCommands[i]);
    Wire.beginTransmission(motorAddresses[i]);  // transmit to address of motor i
    Wire.write(sentCmd[0]);             // sends commandMode
    Wire.write(sentCmd[1]);             // sends targetVal1 (either pos or veloc)
    Wire.write(sentCmd[2]);             // sends targetVal2 (either pos or veloc)
    Wire.endTransmission();             // stop transmitting
  }

  /*
   * Receive and print motor info
   */
  measuredOutputsStr = "";
  for(int i = 0; i < numMotors; i++) {
    Wire.requestFrom(motorAddresses[i], 8);    // request 8 bytes from peripheral device i
    readMeasuredOutputs();
  }
  Serial.println();
}


/**
 * records requested bytes from i2c.
 */
void readMeasuredOutputs() {
  int counter = 0;
  while (Wire.available()) { // peripheral may send less than requested
    bytes[counter] = Wire.read();
    Serial.print(bytes[counter]);
    Serial.print(" ");
    counter++;
  }
  Serial.print(",");
}

/**
   sets the commands to be sent to a motor for a specific rpm
   @param rpm the rotations per minute a motor should rotate at
*/
void setRPM(long rpm) {
  sentCmd[0] = 1;
  sentCmd[1] = (rpm >> 8);
  sentCmd[2] = rpm & 0xFF;
}

/**
   receives data with end marker.
   code modified from
      example code by C. Goulding (AKA groundFungus)
      credit to Robin2, author of the serial input basics tutorial
      https://forum.arduino.cc/t/reading-splitting-multiple-values-in-one-line-received-in-the-serial-connection/700206/2
*/
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

/**
   parses data into ints.
   code modified from
      example code by C. Goulding (AKA groundFungus)
      credit to Robin2, author of the serial input basics tutorial
      https://forum.arduino.cc/t/reading-splitting-multiple-values-in-one-line-received-in-the-serial-connection/700206/2
*/
void showNewData() {
  char delimiter[] = " ";

  if (newData == true) {
    token = strtok(receivedChars, delimiter);
    int count = 0;
    while (token != NULL) {

      motorCommands[count] = atoi(token);

      token = strtok(NULL, delimiter);
      count++;
    }
    newData = false;
  }
}
