#include <sys/_stdint.h>
#ifndef ENCODERS_MODULE_H
#define ENCODERS_MODULE_H

#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

struct EncoderData {
  int32_t leftAngle;
  int32_t rightAngle;
};

// Fonctions d'interface publiques
int32_t setupEncoders(int32_t* psi_ref, int32_t* sum_ref);
EncoderData readEncoders();

#endif
