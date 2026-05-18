/*
 This file is for implementing the external variables declared in config.h
 this is where you will change the functionality of the board
 everything in config.h is set in stone via the addresses and pin connections
 and cannot be changed, this file is where everything for the whole project can be
 changed.
*/

#include "Config.h"

// Sampling Rate Configuration //
uint8_t sampleRate = 200; // Hz

// Motor Configuration //
// Defining the pwm signal for each motor
// Low value = Low intensity, High value = High intensity
uint8_t MOTOR_INDEX_PWM  = 200; // Acceptable range 50 - 255
uint8_t MOTOR_MIDDLE_PWM = 200; // Acceptable range 50 - 255
uint8_t MOTOR_RING_PWM   = 200; // Acceptable range 50 - 255
uint8_t MOTOR_PINKY_PWM  = 240; // Acceptable range 50 - 255

uint16_t STIM_ON_MS  = 150;   // Motor drive time during stimulation ON (ms)

