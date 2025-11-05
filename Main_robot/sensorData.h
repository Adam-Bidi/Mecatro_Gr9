#ifndef SENSORS_MODULE_H
#define SENSORS_MODULE_H

void setupSensor();
int32_t readSensor();
float linePositionIntToFloat(int32_t linePosRaw);

#endif
