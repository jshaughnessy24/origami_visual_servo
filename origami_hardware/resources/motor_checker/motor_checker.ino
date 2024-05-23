// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI Peripheral device
// Refer to the "Wire Peripheral Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
}

void loop()
{
  Wire.beginTransmission(19); // transmits to device with address 19 TODO: change to your motor address!
  Wire.write(0);       
  Wire.write(-100);           // position
  Wire.write(-100);           // position
  Wire.endTransmission();     // stop transmitting

  delay(500);
}
