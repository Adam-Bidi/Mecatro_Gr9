//Le code global

#include <sys/_stdint.h>
#include <Arduino.h>
#include "MecatroUtils.h"
#include "encodersModule.h"
#include "sensorData.h"
#include "controleur.h"
#include "teleguidage.h"

//On définit le nom et le mot de passe du réseau Wifi
#define WIFI_SSID "ArduinoMecatroGr9"
#define WIFI_PASSWRD "12345678"
#define CONTROL_LOOP_PERIOD 5 // en ms

//D'une manière générale, nous essayons de garder les valeurs brutes pour pouvoir garder des entiers qui occupent moins d'espace mémoire que les flottants.
int32_t psi_ref;
float PID_1[3] = {672, 274, 672}; // T, T_i, T_d
float PID_2[3] = {672, 274, 672};
float gains[4] = {39, 0.01, 0.01, 1.}; //Les valeurs par défaut des gains du PID : S_i, U_bar1, U_bar2, psiDotSeuil

int lostCounter = 0;
const int LOST_LIMIT = 200;

void setup() {
  Wire1.begin();
  Serial.begin(230400);

  int32_t psi_ref, sum_ref;
  setupEncoders(&psi_ref, &sum_ref);
  setupSensor();

  unsigned int const nVariables = 9;
  String variableNames[nVariables] = {"Mode actif", "Uplus", "Umoins", "IntU", "dt", "psi", "intLambda", "lambda", "moyPsiDot"};
  mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
  Serial.println("Telemetry");

  const int nGains = 10;
  float recv[nGains];
  mecatro::recieveGains(nGains, recv);
  for (int i = 0; i < 3; i++) {
    PID_1[i] = recv[i];
    PID_2[i] = recv[i + 3];
    gains[i] = recv[i + 6];
  }
  gains[3] = recv[9];

  mecatro::configureArduino(CONTROL_LOOP_PERIOD);
  mecatro::setMotorDutyCycle(0., 0.);

  unsigned long currentTime = micros();
  prevTime = currentTime; //On réinitialise la valeur de prevTime une fois le setup fini, car sinon le premier dt serait trop grand
  last_T = sum_ref;
  Serial.println("Init");
}

void loop() {
  mecatro::run();
}

void mecatro::controlLoop() {
  EncoderData data = readEncoders();
  int32_t position = readSensor();

  // compteur en cas de perte de ligne
  if (abs(position) > 127) {
    lostCounter++;
  } else {
    lostCounter = 0;
  }

  if (lostCounter > LOST_LIMIT) { //Si le robot a perdu la ligne pendant trop longtemps, on arrête les moteurs par sécurité
    mecatro::setMotorDutyCycle(0.0, 0.0);
    Serial.println("Ligne perdue trop longtemps, arrêt de sécurité.");
    while (true);;
  }

  MotorPWM taux = controleur(data, position, PID_1, PID_2, gains, psi_ref);

  mecatro::setMotorDutyCycle(taux.left, taux.right);
}
