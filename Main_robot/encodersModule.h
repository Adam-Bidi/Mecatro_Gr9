#ifndef ENCODERS_MODULE_H
#define ENCODERS_MODULE_H

#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

struct EncoderData {
  float leftAngle;
  float rightAngle;
};

// Fonctions d'interface publiques
float setupEncoders();
EncoderData readEncoders();

#endif
