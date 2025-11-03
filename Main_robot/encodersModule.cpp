#include <sys/_stdint.h>
#include <Arduino.h>
#include "MecatroUtils.h"
#include "encodersModule.h"
#include "Wire.h"


// Définition des constantes
#define RIGHT_ENCODER_PIN 0
#define LEFT_ENCODER_PIN 3

// Objets globaux internes à ce module
QWIICMUX multiplexer;
AS5600 rightEncoder(&Wire1);
AS5600 leftEncoder(&Wire1);

int32_t setupEncoders(int32_t* psi_ref, int32_t* sum_ref) {
  if (!multiplexer.begin(0x70, Wire1)) {
    Serial.println("Error: I2C multiplexer not found. Check wiring.");
    while (true);;
  }

  bool isInit = true;

  // Initialisation du code d’origine
  multiplexer.setPort(RIGHT_ENCODER_PIN);
  rightEncoder.begin();
  if (!rightEncoder.isConnected()) {
    Serial.println("Error: could not connect to right encoder. Check wiring.");
    isInit = false;
  }

  multiplexer.setPort(LEFT_ENCODER_PIN);
  leftEncoder.begin();
  if (!leftEncoder.isConnected()) {
    Serial.println("Error: could not connect to left encoder. Check wiring.");
    isInit = false;
  }

  if (!isInit) {
    while (true);;
  }

  // On établit le psi de référence et on le retranche aux valeurs suivantes de psi
  multiplexer.setPort(RIGHT_ENCODER_PIN);
  int32_t rightAngleRef = rightEncoder.getCumulativePosition();
  multiplexer.setPort(LEFT_ENCODER_PIN);
  int32_t leftAngleRef = leftEncoder.getCumulativePosition();
  *psi_ref = leftAngleRef - rightAngleRef;
  *sum_ref = leftAngleRef + rightAngleRef;
}

EncoderData readEncoders() {
  EncoderData data;

  multiplexer.setPort(RIGHT_ENCODER_PIN);
  data.rightAngle = rightEncoder.getCumulativePosition();

  multiplexer.setPort(LEFT_ENCODER_PIN);
  data.leftAngle = leftEncoder.getCumulativePosition();
  
  return data;
}