#ifndef CONTROLEUR_H
#define CONTROLEUR_H

#include "encodersModule.h"

extern unsigned long prevTime;

struct MotorPWM {
  float left;
  float right;
};

extern float integral;
extern int32_t last_T;

MotorPWM controleur(EncoderData data, int32_t linePosition, float gains[4], int32_t psi_ref);
float integrale(float lambda);

#endif
