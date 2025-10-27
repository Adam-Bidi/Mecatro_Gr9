#include "controleur.h"
#include <Arduino.h>

float T_1 = 1;
float T_2 = 1;
float T_i = 1;
float T_d = 1;

float U_plus = 10;

float eta = 0;
unsigned long prevTime_der = 0;
unsigned long prevTime_int = 0;

float gamma_prev = 0;

float derivee(float gamma) {
    unsigned long currentTime = micros();
    float dt = (currentTime - prevTime_der) * 1e-6; // On convertie les microsecondes en secondes
    float gamma_prim = (gamma - gamma_prev) / dt;
    gamma_prev = gamma;
    prevTime_der = currentTime;
    return gamma_prim;
}


float integrale(float eta_prim) { // On calcule l'intégrale de eta_prim soit eta
    unsigned long currentTime = micros(); // On utilise micros() pour plus de précision
    float dt = (currentTime - prevTime_int) * 1e-6; // On convertit les microsecondes en secondes
    eta += eta_prim * dt;
    prevTime_int = currentTime;
    return eta;
}


MotorPWM controleur(float leftAngle, float rightAngle, float linePosition) {
  float psi = leftAngle - rightAngle; // les signes dépendent de l'orientation des encodeurs
  float lambda = linePosition;

  // PI(D)
  float eta_prim = T_i * lambda;
  float psi_c = - T_1 * lambda - integrale(eta_prim);

  // PD
  float gamma = psi_c - psi;
  float U_ = - T_2 * gamma - T_d * derivee(gamma);

  //U_ = tension_moteur_g - tension_moteur_d

  float rot_mot_g = (U_plus + U_) / (2 * U_plus);
  float rot_mot_d = (U_plus - U_) / (2 * U_plus);

  return MotorPWM{rot_mot_g, rot_mot_d};
}
