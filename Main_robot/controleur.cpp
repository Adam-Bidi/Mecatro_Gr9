#include <sys/_stdint.h>
#include <Arduino.h>
#include "MecatroUtils.h"
#include "AS5600.h"
#include "controleur.h"
#include "sensorData.h"

const float U_batterie = 11.2;

float dt = 5e-3;
unsigned long prevTime;
unsigned long currentTime;

float integral = 0;
float integralU = 0;

int32_t last_T = 0;

float integrale(float lambda) { // On calcule l'intégrale de eta_prim soit eta
  integral += lambda * dt;
  return integral;
}

float integraleU(int32_t sum, float U_bar) {
  integralU += U_bar * dt - (sum - last_T) / 50 * AS5600_RAW_TO_RADIANS;
  last_T = sum;
  return integralU;
}

MotorPWM controleur(EncoderData data, int32_t linePosition, float gains[5], int32_t psi_ref) {
  int32_t leftAngle = data.leftAngle;
  int32_t rightAngle = data.rightAngle;

  float T = gains[0];
  float T_i = gains[1]; 
  float T_d = gains[2];
  float S_i = gains[3];
  float U_bar = gains[4];

  // Gain de l/rho sur la mesure des angles
  float psi = (leftAngle - rightAngle - psi_ref) * 0.2 * AS5600_RAW_TO_RADIANS; // les signes dépendent de l'orientation des encodeurs
  float lambda = linePositionIntToFloat(linePosition);

  // PID
  float U_moins = - T * lambda - T_d * psi - T_i * integrale(lambda);
  // float U_moins = 0;

  float U_i = integraleU(leftAngle + rightAngle - psi_ref, U_bar);
  float U_plus = S_i * U_i;

  U_plus = min(12, max(0, U_plus));

  //U_moins = tension_moteur_g - tension_moteur_d

  float rot_mot_g = (U_plus + U_moins) / (2 * U_batterie);
  float rot_mot_d = (U_plus - U_moins) / (2 * U_batterie);

  mecatro::log(0, linePosition);
  mecatro::log(1, U_plus);
  mecatro::log(2, U_moins);
  mecatro::log(4, U_i);

  prevTime = currentTime;
  currentTime = micros(); // On utilise micros() pour plus de précision
  dt = (currentTime - prevTime) * 1e-6; // On convertit les microsecondes en secondes
  return MotorPWM{rot_mot_g, rot_mot_d};
}
