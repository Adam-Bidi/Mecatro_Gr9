#include "controleur.h"
#include <Arduino.h>
#include "MecatroUtils.h"

float U_batterie = 11.2;

float integral = 0;
unsigned long prevTime = 0;


float integrale(float lambda) { // On calcule l'intégrale de eta_prim soit eta
    unsigned long currentTime = micros(); // On utilise micros() pour plus de précision
    float dt = (currentTime - prevTime) * 1e-6; // On convertit les microsecondes en secondes
    integral += lambda * dt;
    prevTime = currentTime;
    return integral;
}

MotorPWM controleur(EncoderData data, float linePosition, float gains[5], float psi_ref) {
  float leftAngle = data.leftAngle;
  float rightAngle = data.rightAngle;

  float T = gains[0];
  float T_i = gains[1]; 
  float T_d = gains[2];
  float U_plus = gains[3]; 
  float U_bar = gains[4];

  float psi = leftAngle - rightAngle - psi_ref; // les signes dépendent de l'orientation des encodeurs
  float lambda = linePosition;

  // PID
  float U_ = - T * lambda - T_d * U_bar * psi - T_i * integrale(lambda);

  //U_ = tension_moteur_g - tension_moteur_d

  float rot_mot_g = (U_plus + U_) / (2 * U_batterie);
  float rot_mot_d = (U_plus - U_) / (2 * U_batterie);

  Serial.print("U+ : ");
  Serial.print(U_plus);
  Serial.print("U_ : ");
  Serial.print(U_);
  Serial.print("psi : ");
  Serial.print(psi);
  Serial.print("line position : ");
  Serial.print(linePosition);
  mecatro::log(0,leftAngle);
  mecatro::log(1, rightAngle);
  mecatro::log(2, linePosition);
  mecatro::log(3, U_);
  mecatro::log(4, psi);

  return MotorPWM{rot_mot_g, rot_mot_d};
  
}
