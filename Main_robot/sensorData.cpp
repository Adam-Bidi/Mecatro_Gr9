//Le code pour intialiser le capteur de ligne et lire ses données

#include <sys/_stdint.h>
#include <Arduino.h>
#include "MecatroUtils.h"
#include "sensorData.h"
#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"
#include "sensorbar.h"

SensorBar mySensorBar(0x3E);

#define SENSORBAR_PIN 3

extern QWIICMUX multiplexer;

int32_t previousPos = 0;

void setupSensor() {
  multiplexer.setPort(SENSORBAR_PIN);
  uint8_t returnStatus = mySensorBar.begin();
  if(!returnStatus) {
    Serial.println("sx1509 IC communication FAILED!");
    while (true);; //Si l'initialisation se passe mal, le code reste indéfiniment dans cette ligne
  }
  mySensorBar.clearBarStrobe();
  mySensorBar.clearInvertBits();
}

int32_t readSensor() {
  int32_t linePosRaw;
  multiplexer.setPort(SENSORBAR_PIN);
  linePosRaw = mySensorBar.getPosition(); //get.position() renvoie un entier entre -127 et 127, où +-127 correspond à une position extrême de la ligne, et 0 indique que la ligne est soit parfaitement centrée, soit que tout est blanc.
  
  //Méthode d'extrapolation de la position : si le capteur perd la ligne, il essaie de la retrouver à partir de sa dernière position
  if (linePosRaw == 0 && abs(previousPos) >= 127) { //Condition de perte de la ligne
    linePosRaw = (previousPos / abs(previousPos)) * 150; //On redéfinit arbitrairement linePosRaw à +-150 pour que le robot se replace plus rapidement sur la ligne
  }
    previousPos = linePosRaw;

  if (mySensorBar.getDensity() == 8) {
    mecatro::setMotorDutyCycle(0.0, 0.0);
    Serial.print("Ligne perdue, arrêt de sécurité. ");
    while (true);;
  }

  return linePosRaw;
}

float linePositionIntToFloat(int32_t linePosRaw) {
  return linePosRaw * 4.585e-2 / 127; //On convertit la position brute en position en mètres. 4.585e-2 correspond à la demi-longueur du capteur de ligne
}