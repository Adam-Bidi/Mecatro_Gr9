#include "controleur.h"
#include <Arduino.h>

float U_bar = 0.1;

float integral = 0;
unsigned long prevTime = 0;

float integrale(float lambda) { // On calcule l'intégrale de eta_prim soit eta
    unsigned long currentTime = micros(); // On utilise micros() pour plus de précision
    float dt = (currentTime - prevTime) * 1e-6; // On convertit les microsecondes en secondes
    integral += lambda * dt;
    prevTime = currentTime;
    return integral;
}

MotorPWM controleur(float leftAngle, float rightAngle, float linePosition, float gains[4]) {
  float T = gains[0];
  float T_i = gains[1]; 
  float T_d = gains[2];
  float U_plus = gains[3]; 

  float psi = leftAngle - rightAngle; // les signes dépendent de l'orientation des encodeurs
  float lambda = linePosition;

  // PID
  float U_ = - T * lambda - T_d * U_bar * psi - T_i * integrale(lambda);

  //U_ = tension_moteur_g - tension_moteur_d

  float rot_mot_g = (U_plus + U_) / (2 * U_plus);
  float rot_mot_d = (U_plus - U_) / (2 * U_plus);

  return MotorPWM{rot_mot_g, rot_mot_d};
}
