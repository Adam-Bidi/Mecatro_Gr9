// The global code of what will the robot do
// TODO : Wifi communication
// TODO : Extrapolate the position of the line when the line follower is all white. Problem : the getPosition function from sensorbar.cpp does not distinguish the case where the line is exactly under the sensor and the one where there is no line under it. Possible solution : code my own getposition function
// TODO : Implement the interruptions in the code

#include <Arduino.h>
#include "MecatroUtils.h"
#include "encodersModule.h"
#include "sensorData.h"
#include "controleur.h"

// Define the name and password of the wifi network
#define WIFI_SSID "ArduinoMecatroGr9"
#define WIFI_PASSWRD "12345678"
#define CONTROL_LOOP_PERIOD 5 // en ms

float gains[4] = {8.0, 0.2, 50.0, 2.75}; //Les valeurs par défaut des gains du PID

void setup() {
  Wire1.begin();

  Serial.begin(230400);

  setupEncoders();
  setupSensor();

  Wire1.setClock(400000);
  mecatro::configureArduino(CONTROL_LOOP_PERIOD);

  unsigned int const nVariables = 5;
  String variableNames[nVariables] = {"leftAngle" , "rightAngle" , "leftSpeed", "rightSpeed", "linePosition"};

  // mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
  // mecatro::recieveGains(4, gains);
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop() {
  // Read the data from the encoders
  EncoderData data = readEncoders();

  // Read the data from the sensor
  int8_t position = readSensor();

  // The first argument is the variable (column) id ; recall that in C++, numbering starts at 0.
  // mecatro::log(0,  data.leftAngle);
  // mecatro::log(1,  data.rightAngle);
  // mecatro::log(2,  data.leftSpeed);
  // mecatro::log(3,  data.rightSpeed);
  // mecatro::log(4, position);

  MotorPWM pwm = controleur(data.leftAngle, data.rightAngle, position, gains);

  mecatro::setMotorDutyCycle(pwm.left, pwm.right);
}
