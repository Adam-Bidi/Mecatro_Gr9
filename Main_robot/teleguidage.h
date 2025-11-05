#ifndef _TELEGUIDAGE_H_
#define _TELEGUIDAGE_H_

#include <WiFi.h>

void checkForNewCommands(float* gains, int n_gains, WiFiClient client);

#endif