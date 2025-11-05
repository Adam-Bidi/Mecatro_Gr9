//Le code global

#include <sys/_stdint.h>
#include <Arduino.h>
#include "MecatroUtils.h"
#include "encodersModule.h"
#include "sensorData.h"
#include "controleur.h"

//On définit le nom et le mot de passe du réseau Wifi
#define WIFI_SSID "ArduinoMecatroGr9"
#define WIFI_PASSWRD "12345678"
#define CONTROL_LOOP_PERIOD 5 // en ms

//D'une manière générale, nous essayons de garder les valeurs brutes pour pouvoir garder des entiers qui occupent moins d'espace mémoire que les flottants.
int32_t psi_ref;
float gains[5] = {672, 274, 672, 39, 0.01}; //Les valeurs par défaut des gains du PID : T, T_i, T_d, S_i, U_bar

int lostCounter = 0;
const int LOST_LIMIT = 200;

void setup() {
  Wire1.begin();
  Serial.begin(230400);

  int32_t sum_ref;
  setupEncoders(&psi_ref, &sum_ref);
  setupSensor();

  unsigned int const nVariables = 6;
  String variableNames[nVariables] = {"Position ligne", "Uplus", "Uminus", "IntU", "Vitesse", "dt"};
  mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
  mecatro::recieveGains(5, gains);

  Wire1.setClock(400000);
  mecatro::configureArduino(CONTROL_LOOP_PERIOD);

  unsigned long currentTime = micros();
  prevTime = currentTime; //On réinitialise la valeur de prevTime une fois le setup fini, car sinon le premier dt serait trop grand
  last_T = sum_ref;
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop() {
  EncoderData data = readEncoders();
  int32_t position = readSensor();

  if (abs(position) > 127) {
    lostCounter++;
  } else {
    lostCounter = 0;
  }

  if (lostCounter > LOST_LIMIT) { //Si le robot a perdu la ligne pendant 1 seconde, on arrête les moteurs par sécurité
    mecatro::setMotorDutyCycle(0.0, 0.0);
    Serial.println("Ligne perdue trop longtemps, arrêt de sécurité.");
    while (true);;
  }

  MotorPWM taux = controleur(data, position, gains, psi_ref);

  mecatro::setMotorDutyCycle(taux.left, taux.right);
}
