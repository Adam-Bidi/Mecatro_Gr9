#ifndef CONTROLEUR_H
#define CONTROLEUR_H

struct MotorPWM {
  float left;
  float right;
};

extern float U_bar;
extern float integral;
extern unsigned long prevTime;

MotorPWM controleur(float leftAngle, float rightAngle, float linePosition, float gains[4]);
float integrale(float lambda);

#endif
