#include "controleur.h"
#include <Arduino.h>
#include "MecatroUtils.h"

float U_batterie = 11.2;

float integral = 0;
unsigned long prevTime;

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
  float U_moins = - T * lambda - T_d * U_bar * psi - T_i * integrale(lambda);

  //U_moins = tension_moteur_g - tension_moteur_d

  float rot_mot_g = (U_plus + U_moins) / (2 * U_batterie);
  float rot_mot_d = (U_plus - U_moins) / (2 * U_batterie);

  Serial.print("U+ : ");
  Serial.print(U_plus);
  Serial.print("U_ : ");
  Serial.print(U_moins);
  Serial.print("psi : ");
  Serial.print(psi);
  Serial.print("line position : ");
  Serial.print(linePosition);
  mecatro::log(0,leftAngle);
  mecatro::log(1, rightAngle);
  mecatro::log(2, linePosition);
  mecatro::log(3, U_plus);
  mecatro::log(4, U_moins);
  mecatro::log(5, psi);
  mecatro::log(6, psi_ref);
  mecatro::log(7,integrale(lambda));

  return MotorPWM{rot_mot_g, rot_mot_d};
  
}
