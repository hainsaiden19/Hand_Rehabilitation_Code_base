
#include "Sensor.h"


uint8_t sensorAddresses[4] = {FSR_INDEX, FSR_MIDDLE, FSR_RING, FSR_PINKY};
static char rxLine[64];
static uint8_t rxLen = 0;


//  I2C: raw 16-bit read, combine lower 2 bytes
int readForceRaw(uint8_t sensor) {
  uint8_t addr = sensorAddresses[sensor];
  Wire.beginTransmission(addr);
  Wire.write(READ_OFFSET);

  if (Wire.endTransmission(false) != 0) {return -1;} // NACK/Error

  int n = Wire.requestFrom((int)addr, (int)READ_LENGTH, (int)true);
  if (n < 6) return -2;

  uint8_t buf[READ_LENGTH];
  int count = 0;
  while (Wire.available() && count < READ_LENGTH) {
    buf[count++] = Wire.read();
  }
  if (count < 6) return -3;

  // Combine byte 4 and 5 
  int forceRaw = ((int)buf[4] << 8) | (int)buf[5];
  return forceRaw;  // Send downstream to python file
}


//  Serial RX (line based) 
void handleSerial() {
  while (Serial.available() > 0) 
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') 
    {
      if (rxLen > 0) 
      {
        rxLine[rxLen] = '\0';
        processLine(rxLine);
        rxLen = 0;
      }
    } 
    
    else 
    {
      if (rxLen < sizeof(rxLine) - 1) 
      { 
        rxLine[rxLen++] = c; 
      } 
      // overflow -> reset
      else { rxLen = 0; }
    }
  }
}