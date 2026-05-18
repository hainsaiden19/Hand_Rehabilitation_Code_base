#ifndef CONFIG_H
#define CONFIG_H


// Standard includes
#include <Arduino.h>
#include <Wire.h>


//  I2C read frame spec (from your code style) 
#define READ_OFFSET  128       // Read start offset 
#define READ_LENGTH  6         // Requested number of bytes (minimum 6 bytes required)

//  I2C sensor addresses (from your code) 
#define FSR_INDEX  0x05
#define FSR_MIDDLE 0x06
#define FSR_RING   0x07
#define FSR_PINKY  0x08

// For Defining pin mapping
#define PIN_MOTOR_INDEX  11 // D11
#define PIN_MOTOR_MIDDLE 10 // D10
#define PIN_MOTOR_RING   9 // D9
#define PIN_MOTOR_PINKY  6 // D6


// For individually accessing motor pins in the array
#define MOTOR_INDEX 0
#define MOTOR_MIDDLE 1
#define MOTOR_RING 2
#define MOTOR_PINKY 3


// Sampling Rate Declerations
extern uint8_t sampleRate;

// Vibration Motor Declerations
extern uint8_t MOTOR_INDEX_PWM;
extern uint8_t MOTOR_MIDDLE_PWM;
extern uint8_t MOTOR_RING_PWM;
extern uint8_t MOTOR_PINKY_PWM;
extern uint16_t STIM_ON_MS;

#endif