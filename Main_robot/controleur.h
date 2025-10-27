#ifndef CONTROLEUR_H
#define CONTROLEUR_H

struct MotorPWM {
  float left;
  float right;
};

extern float T_1;
extern float T_2;
extern float T_d;
extern float T_i;

extern float eta;
extern unsigned long prevTime_der;
extern unsigned long prevTime_int;
extern float U_plus;

extern float gamma_prev;

MotorPWM controleur(float leftAngle, float rightAngle, float linePosition);
float integrale(float eta_prim);
float derivee(float gamma);

#endif
