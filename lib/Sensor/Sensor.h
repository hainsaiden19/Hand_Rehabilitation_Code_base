#ifndef SENSOR_H
#define SENSOR_H

#include "Motor.h"

int readForceRaw(uint8_t addr);
void handleSerial();

#endif