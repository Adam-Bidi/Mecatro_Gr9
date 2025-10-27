#ifndef SENSORS_MODULE_H
#define SENSORS_MODULE_H

#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"
#include "sensorbar.h"
#include "Wire.h"

// Fonctions d'interface publiques
void setupSensor();
int8_t readSensor();

#endif
