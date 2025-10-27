#include <Arduino.h>
#include "Config.h"
#include "MecatroUtils.h"
#include "encodersModule.h"
#include "Wire.h"


// Définition des constantes
#define RIGHT_ENCODER_PIN 3
#define LEFT_ENCODER_PIN 0

// Objets globaux internes à ce module
QWIICMUX multiplexer;
AS5600 rightEncoder(&Wire1);
AS5600 leftEncoder(&Wire1);

void setupEncoders() {
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
}

EncoderData readEncoders() {
  EncoderData data;

  multiplexer.setPort(RIGHT_ENCODER_PIN);
  data.rightAngle = rightEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  data.rightAngle = rightEncoder.getAngularSpeed();

  multiplexer.setPort(LEFT_ENCODER_PIN);
  data.leftAngle = leftEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  data.leftSpeed = leftEncoder.getAngularSpeed();

  return data;
}