/* Demo code for reading the encoders of the robot.

  Note: this code requires the following libraries (install them through the library manager):
     - SparkFun I2C Mux Arduino Library
     - AS5600 library
*/

#include <Arduino.h>
// Include the current library
#include "MecatroUtils.h"

// Include the SensorBar library (for Line Follower) and Sparkfun I2C Mux (for multiplexer)
#include "sensorbar.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// Header for I2C communication
#include "Wire.h"

// Wire1 is used for the Qwiic connector on this Arduino board. 
//SensorBar mySensorBar(SX1509_ADDRESS);
SensorBar mySensorBar(0x3E);

// Define the port numbers
#define SENSORBAR_PIN 3

extern QWIICMUX multiplexer;

void setupSensor()
{
  //Don't forget to call .begin() to get the bar ready.  This configures HW.
  multiplexer.setPort(SENSORBAR_PIN);
  uint8_t returnStatus = mySensorBar.begin();
  if(!returnStatus)
  {
    Serial.println("sx1509 IC communication FAILED!");
    while (true);; //We put this line so that the code is blocked infinitely at this line, which is what we want if some initialization has gone wrong
  }
  //Command for the IR to run all the time
  mySensorBar.clearBarStrobe();
  mySensorBar.clearInvertBits();
}

float readSensor() {
  int8_t linePosRaw;
  float linePosition;
  // Set multiplexer to talk to line follower (we have to recall it in case we used another port meanwhile)
  multiplexer.setPort(SENSORBAR_PIN);
  linePosRaw = mySensorBar.getPosition();
  Serial.print("linePosRaw : ");
  Serial.print(linePosRaw);

  linePosition = ((float)linePosRaw) * 4.585e-2 / 127; // We convert the raw position to meters
  return linePosition;
}