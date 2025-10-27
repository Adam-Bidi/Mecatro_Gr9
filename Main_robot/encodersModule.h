#ifndef ENCODERS_MODULE_H
#define ENCODERS_MODULE_H

#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

struct EncoderData {
  float leftAngle;
  float rightAngle;
  float leftSpeed;
  float rightSpeed;
};

// Fonctions d'interface publiques
void setupEncoders();
EncoderData readEncoders();

#endif
