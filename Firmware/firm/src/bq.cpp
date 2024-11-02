#include "bq.hpp"

int writeToReg(uint8_t reg, uint8_t data)
{
    int rtn;
    Wire.beginTransmission((uint8_t)ADDR);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)data);
    rtn = Wire.endTransmission();

    return !rtn;
}

int read_byte(uint8_t *data, uint8_t reg)
{

    int rtn;
    Wire.beginTransmission((uint8_t)ADDR);
    Wire.write(reg);
    rtn = Wire.endTransmission();

    Wire.requestFrom((uint8_t)ADDR, (uint8_t)1);
    delayMicroseconds(100);
    *data = Wire.read();

    return !rtn;
}

int readRawReg(byte reg)
{
    uint8_t val;

    read_byte(&val, reg);

    return val;
}


bool regWriteCheck() {
  if (readRawReg(0x01) == reg01) {
    return true;
  }
  else {
    return false;
  }
}

bool writeToAllRegs() {
    for(uint8_t i = 0; i < 12;i++) {
        bool rtn = writeToReg(i, regDataList[i]);
        if(rtn == 0) {
            return 0;
        }
        delay(100);
    }
    return 1;
}



void enableBQADC() {
  Wire.beginTransmission(ADDR);
  Wire.write((uint8_t)0x02);
  Wire.write(reg02_ADC_EN);
  Wire.endTransmission();
}

uint8_t getChrgStatus() { //REG0B
  enableBQADC();
  bool b[2];
  uint8_t regData = readRawReg(0x0B);
  //b[0] = 0 != (regData & 1);
  //b[1] = 0 != (regData & 2);
  //b[2] = 0 != (regData & 4);
  b[1] = 0 != (regData & 8);
  b[0] = 0 != (regData & 16);
  //b[5] = 0 != (regData & 32);
  //b[6] = 0 != (regData & 64);
  //b[7] = 0 != (regData & 128);
  if(b[0] == 0 && b[1] == 0) {
    return 1;
  }

  else if(b[0] == 0 && b[1]) {
    return 2;
  }

  else if(b[0] && b[1] == 0) {
    return 3;
  }

  else {
    return 4;
  }

}
 
uint16_t getBatVoltage() { //REG0E
  enableBQADC();
  bool b[7];
  uint8_t regData = readRawReg(0x0E);
  //b[7] = 0 != (regData & 1);
  b[6] = 0 != (regData & 2);
  b[5] = 0 != (regData & 4);
  b[4] = 0 != (regData & 8);
  b[3] = 0 != (regData & 16);
  b[2] = 0 != (regData & 32);
  b[1] = 0 != (regData & 64);
  b[0] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  if(b[0]) {
    adcVal += 20;
  }
  if(b[1]) {
    adcVal += 40;
  }
  if(b[2]) {
    adcVal += 80;
  }
  if(b[3]) {
    adcVal += 160;
  }
  if(b[4]) {
    adcVal += 320;
  }
  if(b[5]) {
    adcVal += 640;
  }
  if(b[6]) {
    adcVal += 1280;
  }
  /*adcVal += b[0] * 20;
  adcVal += b[1] * 40;
  adcVal += b[2] * 80;
  adcVal += b[3] * 160;
  adcVal += b[4] * 320;
  adcVal += b[5] * 640;
  adcVal += b[6] * 1280;*/
  return adcVal;
}

uint16_t getSysVoltage() { //REG0F
  enableBQADC();
  bool b[7];
  uint8_t regData = readRawReg(0x0F);
  //b[7] = 0 != (regData & 1);
  b[6] = 0 != (regData & 2);
  b[5] = 0 != (regData & 4);
  b[4] = 0 != (regData & 8);
  b[3] = 0 != (regData & 16);
  b[2] = 0 != (regData & 32);
  b[1] = 0 != (regData & 64);
  b[0] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  if(b[0]) {
    adcVal += 20;
  }
  if(b[1]) {
    adcVal += 40;
  }
  if(b[2]) {
    adcVal += 80;
  }
  if(b[3]) {
    adcVal += 160;
  }
  if(b[4]) {
    adcVal += 320;
  }
  if(b[5]) {
    adcVal += 640;
  }
  if(b[6]) {
    adcVal += 1280;
  }
  /*adcVal += b[0] * 20;
  adcVal += b[1] * 40;
  adcVal += b[2] * 80;
  adcVal += b[3] * 160;
  adcVal += b[4] * 320;
  adcVal += b[5] * 640;
  adcVal += b[6] * 1280;*/
  return adcVal;
}

uint16_t getVBUSVoltage() { //REG11
  enableBQADC();
  bool b[7];
  uint8_t regData = readRawReg(0x11);
  //b[7] = 0 != (regData & 1);
  b[6] = 0 != (regData & 2);
  b[5] = 0 != (regData & 4);
  b[4] = 0 != (regData & 8);
  b[3] = 0 != (regData & 16);
  b[2] = 0 != (regData & 32);
  b[1] = 0 != (regData & 64);
  b[0] = 0 != (regData & 128);

  uint16_t adcVal = 2600;
  if(b[0]) {
    adcVal += 100;
  }
  if(b[1]) {
    adcVal += 200;
  }
  if(b[2]) {
    adcVal += 400;
  }
  if(b[3]) {
    adcVal += 800;
  }
  if(b[4]) {
    adcVal += 1600;
  }
  if(b[5]) {
    adcVal += 3200;
  }
  if(b[6]) {
    adcVal += 6400;
  }
  /*adcVal += b[0] * 100;
  adcVal += b[1] * 200;
  adcVal += b[2] * 400;
  adcVal += b[3] * 800;
  adcVal += b[4] * 1600;
  adcVal += b[5] * 3200;
  adcVal += b[6] * 6400;*/
  return adcVal;
}

bool getVBUSAttached() { //REG11
  bool b = 0;
  uint8_t regData = readRawReg(0x11);
  b = 0 != (regData & 1);
  return b;
}

uint16_t getChrgCurrent() { //REG12
  enableBQADC();
  bool b[7];
  uint8_t regData = readRawReg(0x12);
  //b[7] = 0 != (regData & 1);
  b[6] = 0 != (regData & 2);
  b[5] = 0 != (regData & 4);
  b[4] = 0 != (regData & 8);
  b[3] = 0 != (regData & 16);
  b[2] = 0 != (regData & 32);
  b[1] = 0 != (regData & 64);
  b[0] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  if(b[0]) {
    adcVal += 50;
  }
  if(b[1]) {
    adcVal += 100;
  }
  if(b[2]) {
    adcVal += 200;
  }
  if(b[3]) {
    adcVal += 400;
  }
  if(b[4]) {
    adcVal += 800;
  }
  if(b[5]) {
    adcVal += 1600;
  }
  if(b[6]) {
    adcVal += 3200;
  }
  /*adcVal += b[0] * 50;
  adcVal += b[1] * 100;
  adcVal += b[2] * 200;
  adcVal += b[3] * 400;
  adcVal += b[4] * 800;
  adcVal += b[5] * 1600;
  adcVal += b[6] * 3200;*/
  return adcVal;
}

uint16_t getICOILIM() { //REG13
  enableBQADC();
  bool b[6];
  uint8_t regData = readRawReg(0x12);
  //b[7] = 0 != (regData & 1);
  //b[6] = 0 != (regData & 2);
  b[5] = 0 != (regData & 4);
  b[4] = 0 != (regData & 8);
  b[3] = 0 != (regData & 16);
  b[2] = 0 != (regData & 32);
  b[1] = 0 != (regData & 64);
  b[0] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  if(b[0]) {
    adcVal += 50;
  }
  if(b[1]) {
    adcVal += 100;
  }
  if(b[2]) {
    adcVal += 200;
  }
  if(b[3]) {
    adcVal += 400;
  }
  if(b[4]) {
    adcVal += 800;
  }
  if(b[5]) {
    adcVal += 1600;
  }
  /*adcVal += b[0] * 50;
  adcVal += b[1] * 100;
  adcVal += b[2] * 200;
  adcVal += b[3] * 400;
  adcVal += b[4] * 800;
  adcVal += b[5] * 1600;*/
  return adcVal;
}
