
#include "Motor.h"


uint8_t motorPins[4] = {PIN_MOTOR_INDEX, PIN_MOTOR_MIDDLE, PIN_MOTOR_RING, PIN_MOTOR_PINKY};
uint8_t STIM_PWM[4] = {MOTOR_INDEX_PWM, MOTOR_MIDDLE_PWM, MOTOR_RING_PWM, MOTOR_PINKY_PWM};
uint16_t stimOffAt[4] = {0,0,0,0};  // Scheduled automatic STIM off time (0 = currently off)


void motorInit(){
  // timer 0 is already ~1khz
  TCCR1B = (TCCR1B & 0b11111000) | 0x03; // ~ 1khz
  TCCR2B = (TCCR2B & 0b11111000) | 0x03; // ~ 1khz

  for (int i=0; i<4; ++i){ pinMode(motorPins[i], OUTPUT); }
}


// This function doesnt really work, since current draw from
// all 4 motors is way too much for the darlington driver
// and they are very weak, better to have only one on at a time
void motorAllOn(){
  for (int i=0; i<4; ++i) 
  {
    analogWrite(motorPins[i], STIM_PWM[i]);
  }
}


void individualMotorOn(int motor){
  analogWrite(motorPins[motor], STIM_PWM[motor]);
}


void individualMotorOff(int motor){
  analogWrite(motorPins[motor], 0);
}


//  STIM control 
void triggerStimMotor(int laneIdx) {
  if (laneIdx < 0 || laneIdx > 3) return;
  analogWrite(motorPins[laneIdx], STIM_PWM[laneIdx]);           
  stimOffAt[laneIdx] = millis() + STIM_ON_MS;          // 一定時間後に自動OFF
}

void updateStimMotors() {
  unsigned long now = millis();
  for (int i=0; i<4; ++i) {
    if (stimOffAt[i] && (long)(now - stimOffAt[i]) >= 0) {
      analogWrite(motorPins[i], 0);
      stimOffAt[i] = 0;
    }
  }
}

// Called to trigger motor changes from the python script
void processLine(const char* s) {
  // "STIM:n"
  if (strncmp(s, "STIM:", 5) == 0) {
    int lane = atoi(s + 5);
    if (lane >= 1 && lane <= 4) {
      triggerStimMotor(lane - 1);
    }
    return;
  }

  // Optional: Full stop
  if (strcmp(s, "STOP") == 0) {
    for (int i=0; i<4; ++i) {
      analogWrite(motorPins[i], 0);
      stimOffAt[i] = 0;
    }
    return;
  }
  // Ignore other messages
}