#include <Arduino.h>
#include <Wire.h>
const uint8_t reg00 = 0x3F; //00111111 = 63 CHECK

const uint8_t reg01 = 0xC5; //11000101 = 197 NOT, READ 00000101 = 5

const uint8_t reg02 = 0x30; //00110000 = 48 (No ADC) CHECK
const uint8_t reg02_ADC_EN = 176; //or 10110000 = 176(Start ADC Conversion)

const uint8_t reg03 = 0x1A; //00011010 = 26 CHECK
const uint8_t reg04 = 0x7F;  //01111111 = 127 NOT, READ 01001111 = 79
const uint8_t reg05 = 0x13; //00010011 = 19 CHECK
const uint8_t reg06 = 0x62; //01100010 = 98 CHECK
const uint8_t reg07 = 0xC9; //11001001 = 201 CHECK
const uint8_t reg08 = 0x03; //00000011 = 3 CHECK
const uint8_t reg09 = 0x40; //01000000 = 64 NOT, READ 01000100 = 68
const uint8_t reg0A = 0x90; //10010000 = 144 CHECK

const uint8_t reg0D = 0xFF; // 11111111

/* uint8_t reg0B = 0; //READ ONLY //xxxBBxxx CHRG_STAT REGISTER, CHARGING MODE, 
uint8_t reg0C = 0; //READ ONLY

uint8_t reg0E = 0; //READ ONLY //Battery Voltage ADC, lower 7 bits, p.40 of DS

uint8_t reg0F = 0; //READ ONLY // SYSTEM VOLTAGE REGISTER, lower 7 bits
uint8_t reg10 = 0; //READ ONLY, USELESS
uint8_t reg11 = 0; //READ ONLY, First bit VBUS attached, lower 7 bits VBUS Voltage
uint8_t reg12 = 0; //READ ONLY, Lower 7 bits charge current
uint8_t reg13 = 0; //READ ONLY, lower 6 bits ICO ILIM*/


const uint8_t regDataList[] = {reg00, reg01, reg02, reg03, reg04, reg05, reg06, reg07, reg08, reg09, reg0A, reg0D};

const uint8_t ADDR = 0x6A;



int writeToReg(uint8_t reg, uint8_t data);
int read_byte(uint8_t *data, uint8_t reg);
int readRawReg(byte reg);
bool regWriteCheck();
bool writeToAllRegs();
void enableBQADC();
uint8_t getChrgStatus();
uint16_t getBatVoltage();
uint16_t getSysVoltage();
uint16_t getVBUSVoltage();
bool getVBUSAttached();
uint16_t getChrgCurrent();
uint16_t getICOILIM();