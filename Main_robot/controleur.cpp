//Le code du contrôleur

#include <sys/_stdint.h>
#include <Arduino.h>
#include "MecatroUtils.h"
#include "controleur.h"
#include "AS5600.h"
#include "encodersModule.h"
#include "sensorData.h"

const float U_battery = 10.8; //Tension de la batterie en V
int32_t NIter = 200; //Nombre minimal d'itérations dans le mode virage

float dt = 5e-3; //On initialise le dt à 5 ms, qui correspond à CONTROL_LOOP_PERIOD
unsigned long prevTime;
unsigned long currentTime;

float integral = 0; //On initialise les integrales
float integralU = 0;
float lastPsi = 0;
float psiDot;

int32_t last_T = 0;

int nLoop = 0;

//On initialise le moyennage sur PsiDot servant au changement de mode
const int nValues = 40;
int values[nValues];
int iValues = 0;
int moyPsiDot = 0;

//PID actif initialisé en mode ligne droite (2 : PID_1->PID_2, 1 : PID_2->PID_1)
float Active_PID[3];
int Active = 1;

float U_bar;
const int nTransition = 5;  // Nombre de points pour la transition entre modes

// Calcul intégrale sur lambda
float integrale(float lambda) {
  integral += lambda * dt;
  return integral;
}

// Calcul intégrale de u
float integraleU(int32_t sum, float U_bar) {
  integralU += U_bar * dt - (sum - last_T) / 50 * AS5600_RAW_TO_RADIANS;
  last_T = sum;
  return integralU;
}

// Calcul dérivée de psi
float derivee(float psi) {
  psiDot = (psi - lastPsi) / dt;
  lastPsi = psi;
  return psiDot;
}


MotorPWM controleur(EncoderData data, int32_t linePosition, float PID_1[3], float PID_2[3], float gains[4], int32_t psi_ref) {
  int32_t leftAngle = data.leftAngle;
  int32_t rightAngle = data.rightAngle;

  float S_i = gains[0];
  float U_bar1 = gains[1];
  float U_bar2 = gains[2];
  float psiDotSeuil = gains[3];

  // Calcul de psi (gain de l/rho = 0.2 sur la mesure des angles)
  float psi = (leftAngle - rightAngle - psi_ref) * 0.2 * AS5600_RAW_TO_RADIANS; // Les signes dépendent ici de l'orientation des encodeurs sur notre robot
  float psiDot = derivee(psi);

  // Calcul de la moyenne des lambda
  moyPsiDot -= values[iValues];
  values[iValues] = psiDot;
  moyPsiDot += psiDot;
  iValues += 1;
  if (iValues >= nValues) iValues = 0;

  // Test déclenchant le changement de mode (si moyPsiDot est suffisament élevé et si le robot est resté suffisamment dans le mode virage le cas échéant)
  if (abs(moyPsiDot) > psiDotSeuil * nValues && Active == 1) {
    Active = 2;
    NIter = 0;
  }
  else if (abs(moyPsiDot) <= psiDotSeuil * nValues && NIter >= 200 && Active == 2) {
    Active = 1;
    NIter = 0;
  }
  NIter += 1;

  // Détermination des gains du PID actif avec une combinaison linéaire au cours du passage d'un mode à l'autre
  float t = min(NIter, nTransition) / nTransition;
  switch (Active) {
    case 2:
      for (int i = 0; i < 3; i++) { Active_PID[i] = PID_1[i] + t * (PID_2[i] - PID_1[i]); }
      U_bar = U_bar1 + t * (U_bar2 - U_bar1);
      break;
    case 1:
      for (int i = 0; i < 3; i++) { Active_PID[i] = PID_2[i] + t * (PID_1[i] - PID_2[i]); }
      U_bar = U_bar2 + t * (U_bar1 - U_bar2);
      break;
    default:
      break;
  }

  // Pour les 100 premières itérations, on va tout droit et on force donc le mode ligne droite
  if (nLoop < 100) {
    psiDot = 0;
    Active = 1;
    U_bar = U_bar1;
    for (int i = 0; i < 3; i++) { Active_PID[i] = PID_1[i]; }
  }


  float lambda = linePositionIntToFloat(linePosition);

  float T = Active_PID[0];
  float T_i = Active_PID[1]; 
  float T_d = Active_PID[2];

  // PID sur Umoins
  float U_minus = - T * lambda - T_d * psi - T_i * integrale(lambda);

  // Correcteur intégral pour Uplus
  float U_i = integraleU(leftAngle + rightAngle - psi_ref, U_bar * nLoop / 100);
  float U_plus = S_i * U_i;

  U_plus = min(24, max(0, U_plus));

  // Limitation de la rotation au départ
  if (nLoop < 100) U_minus = min(2, max(-2, U_minus));

  // Récupération des commandes Ul et Ur à partir de Uplus et Umoins
  float rot_mot_l = (U_plus + U_minus) / U_battery / 2;
  float rot_mot_r = (U_plus - U_minus) / U_battery / 2;

  // Envoi de grandeurs par télémétrie
  mecatro::log(0, Active * 10);
  mecatro::log(1, U_plus);
  mecatro::log(2, U_minus);
  mecatro::log(3, integralU);
  mecatro::log(4, dt);
  mecatro::log(5, psi);
  mecatro::log(6, integral);
  mecatro::log(7, lambda);
  mecatro::log(8, moyPsiDot);

  prevTime = currentTime;
  currentTime = micros(); // On utilise micros() pour plus de précision
  dt = min(10e-3, (currentTime - prevTime) * 1e-6); // On convertit les microsecondes en secondes

  // Compteur sur les premières itérations
  if (nLoop < 100) {
    nLoop += 1;
  }

  return MotorPWM{rot_mot_l, rot_mot_r};
}
