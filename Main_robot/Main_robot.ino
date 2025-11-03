// The global code of what will the robot do

#include <Arduino.h>
#include "MecatroUtils.h"
#include "encodersModule.h"
#include "sensorData.h"
#include "controleur.h"

// Define the name and password of the wifi network
#define WIFI_SSID "ArduinoMecatroGr9"
#define WIFI_PASSWRD "12345678"
#define CONTROL_LOOP_PERIOD 5 // en ms

int32_t psi_ref;
float gains[6] = {672, 274, 672, 39, 0.01}; //Les valeurs par défaut des gains du PID : T, T_i, T_d, S_i, U_bar

int lostCounter = 0;
const int LOST_LIMIT = 200;

void setup() {
  Wire1.begin();
  Serial.begin(230400);

  int32_t sum_ref;
  setupEncoders(&psi_ref, &sum_ref);
  setupSensor();


  unsigned int const nVariables = 5;
  String variableNames[nVariables] = {"Position ligne", "Uplus", "Umoins", "IntU", "Vitesse"};
  mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
  mecatro::recieveGains(5, gains);

  Wire1.setClock(400000);
  mecatro::configureArduino(CONTROL_LOOP_PERIOD);

  unsigned long currentTime = micros();
  prevTime = currentTime - 5000; // Once the setup is over, we reinitialize the value of prevTime, so that the first dt in the integral is not too big
  last_T = sum_ref;
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop() {
  // Read the data from the encoders
  EncoderData data = readEncoders();

  // Read the data from the sensor
  int32_t position = readSensor();
  Serial.println(position);

  if (abs(position) > 127) {
    lostCounter++;
  } else {
    lostCounter = 0;
  }

  if (lostCounter > LOST_LIMIT) {
    mecatro::setMotorDutyCycle(0.0, 0.0);
    Serial.println("Ligne perdue trop longtemps, arrêt de sécurité.");
    while (true);;
  }

  MotorPWM taux = controleur(data, position, gains, psi_ref);

  mecatro::setMotorDutyCycle(taux.left, taux.right);
}
