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

float psi_ref;
float gains[5] = {672, 274, 672, 0.275, 0.01}; //Les valeurs par défaut des gains du PID : T, T_i, T_d, U+, U_bar

void setup() {
  Wire1.begin();

  Serial.begin(230400);

  psi_ref = setupEncoders();
  setupSensor();

  Wire1.setClock(400000);
  mecatro::configureArduino(CONTROL_LOOP_PERIOD);

  unsigned int const nVariables = 5;
  String variableNames[nVariables] = {"leftAngle" , "rightAngle" , "linePosition", "U_", "psi"};

  mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
  mecatro::recieveGains(5, gains);
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop() {
  // Read the data from the encoders
  EncoderData data = readEncoders();

  // Read the data from the sensor
  float position = readSensor();

  // The first argument is the variable (column) id ; recall that in C++, numbering starts at 0.
  // mecatro::log(0,  data.leftAngle);
  // mecatro::log(1,  data.rightAngle);
  // mecatro::log(2, position);

  MotorPWM pwm = controleur(data, position, gains, psi_ref);

  mecatro::setMotorDutyCycle(pwm.left, pwm.right);
  Serial.print("pwm :");
  Serial.print(pwm.left);
  Serial.print(" ");
  Serial.println(pwm.right);
}
