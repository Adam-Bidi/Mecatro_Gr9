//Le code du contrôleur

#include <sys/_stdint.h>
#include <Arduino.h>
#include "MecatroUtils.h"
#include "controleur.h"
#include "AS5600.h"
#include "encodersModule.h"
#include "sensorData.h"

const float U_battery = 11.2;

float dt = 5e-3; //On initialise le dt à 5 ms, qui correspond à CONTROL_LOOP_PERIOD
unsigned long prevTime;
unsigned long currentTime;

float integral = 0; //On initialise les integrales
float integralU = 0;
float lastPsi = 0;
float psiDot;

int32_t last_T = 0;

int nLoop = 0;
const int nValues = 20;
int values[nValues];
int iValues = 0;
int moyLambda = 0;

float* Active_PID;
float U_bar;

float integrale(float lambda) {
  integral += lambda * dt;
  return integral;
}

float integraleU(int32_t sum, float U_bar) {
  integralU += U_bar * dt - (sum - last_T) / 50 * AS5600_RAW_TO_RADIANS;
  last_T = sum;
  return integralU;
}

float derivee(float psi) {
  psiDot = (psi - lastPsi) / dt;
  lastPsi = psi;
  return psiDot;
}

MotorPWM controleur(EncoderData data, int32_t linePosition, float PID_1[3], float PID_2[3], float gains[3], int32_t psi_ref) {
  int32_t leftAngle = data.leftAngle;
  int32_t rightAngle = data.rightAngle;

  int32_t NIter = 200;

  float S_i = gains[0];
  float U_bar1 = gains[1];
  float U_bar2 = gains[2];

  // Gain de l/rho sur la mesure des angles
  float psi = (leftAngle - rightAngle - psi_ref) * 0.2 * AS5600_RAW_TO_RADIANS; // Les signes dépendent de l'orientation des encodeurs
  float psiDot = derivee(psi);

  // Calcul de la moyenne des lambda
  moyLambda -= values[iValues];
  values[iValues] = linePosition;
  moyLambda += linePosition;
  iValues += 1;
  if (iValues >= nValues) iValues = 0;

  // PID_1 : ligne droite
  if (psiDot > 0.5 & NIter == 200) {
    Active_PID = PID_2;
    U_bar = U_bar2;
    NIter = 0;
  }
  else if (psiDot <= 0.5 & NIter == 200) {
    Active_PID = PID_1;
    U_bar = U_bar1;
    NIter = 0;
  }

  NIter += 1;

  float lambda = linePositionIntToFloat(linePosition);

  float T = Active_PID[0];
  float T_i = Active_PID[1]; 
  float T_d = Active_PID[2];
  Serial.print(T); Serial.print(T_i); Serial.println(T_d);

  // PID
  float U_minus = - T * lambda - T_d * psi - T_i * integrale(lambda);
  // float U_minus = 0;

  float U_i = integraleU(leftAngle + rightAngle - psi_ref, U_bar * nLoop / 200);
  float U_plus = S_i * U_i;

  U_plus = min(12, max(0, U_plus));

  //U_minus = tension_moteur_g - tension_moteur_d

  float rot_mot_l = (U_plus + U_minus) / U_battery / 2;
  float rot_mot_r = (U_plus - U_minus) / U_battery / 2;

  mecatro::log(0, moyLambda);
  mecatro::log(1, U_plus);
  mecatro::log(2, U_minus);
  mecatro::log(3, integralU);
  mecatro::log(4, U_i);
  mecatro::log(5, psi);
  mecatro::log(6, integral);
  mecatro::log(7, dt);
  mecatro::log(8, psiDot);

  prevTime = currentTime;
  currentTime = micros(); // On utilise micros() pour plus de précision
  dt = min(10e-3, (currentTime - prevTime) * 1e-6); // On convertit les microsecondes en secondes
  if (nLoop < 200) {
    nLoop += 1;
  }
  return MotorPWM{rot_mot_l, rot_mot_r};
}
