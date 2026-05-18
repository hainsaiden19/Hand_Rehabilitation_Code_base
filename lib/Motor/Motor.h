#ifndef MOTOR_H
#define MOTOR_H

#include "Config.h"

void motorInit();
void motorAllOn();
void individualMotorOn(int motor);
void individualMotorOff(int motor);
void updateStimMotors();
void processLine(const char* s);

#endif