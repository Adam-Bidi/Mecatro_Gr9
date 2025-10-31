#ifndef CONTROLEUR_H
#define CONTROLEUR_H

#include "encodersModule.h"

extern unsigned long prevTime;

struct MotorPWM {
  float left;
  float right;
};

extern float U_batterie;
extern float integral;

MotorPWM controleur(EncoderData data, float linePosition, float gains[4], float psi_ref);
float integrale(float lambda);

#endif
