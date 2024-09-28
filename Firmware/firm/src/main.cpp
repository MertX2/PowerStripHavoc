#include <Arduino.h>
#include <Wire.h>

// put function declarations here:

#define BTN_PIN 1
#define EN_PIN 15
#define SDA_PIN 8
#define SCL_PIN 9
#define FAN_PIN 0
#define NTC_PIN 14

int ADDR = 106;

void writeToReg(uint8_t	REG_ADDR, uint8_t REG_DATA) {
  Wire.beginTransmission(ADDR);
  Wire.write((uint8_t)REG_ADDR);
  Wire.write(REG_DATA);
  Wire.endTransmission();
}
int readRawReg(uint8_t REG_ADDR) {
  
    Wire.beginTransmission(ADDR);
    Wire.write(REG_ADDR);
    Wire.endTransmission();

    Wire.requestFrom(ADDR, (uint8_t)1);
    delayMicroseconds(100);
    return Wire.read();
}
void writeToAllRegs() {
  writeToReg(0x00, reg00);
  writeToReg(0x01, reg01);
  writeToReg(0x02, reg02);
  writeToReg(0x03, reg03);
  writeToReg(0x04, reg04);
  writeToReg(0x05, reg05);
  writeToReg(0x06, reg06);
  writeToReg(0x07, reg07);
  writeToReg(0x08, reg08);
  writeToReg(0x09, reg09);
  writeToReg(0x0A, reg0a);
}

void enableADC() {
  Wire.beginTransmission(ADDR);
  Wire.write((uint8_t)0x02);
  Wire.write(reg02_ADC_EN);
  Wire.endTransmission();
}
int getChrgStatus(); //REG0B
float getBatVoltage(); //REG0E
float getSysVoltage(); //REG0F
float getVBUSVoltage(); //REG11
int getVBUSAttached(); //REG11
int getChrgCurrent(); //REG12
int getICOILIM(); //REG13



uint8_t reg00 = 63;

uint8_t reg01 = 197; //11000101

uint8_t reg02 = 48; //00110000  (No ADC)
uint8_t reg02_ADC_EN = 176; //or 10110000 (Start ADC Conversion)

uint8_t reg03 = 26; //00011010
uint8_t reg04 = 127;  //01111111
uint8_t reg05 = 19; //00010011
uint8_t reg06 = 98; //01100010
uint8_t reg07 = 201; //11001001
uint8_t reg08 = 3; //00000011
uint8_t reg09 = 64; //01000000
uint8_t reg0a = 144; //10010000

uint8_t reg0B = 0; //READ ONLY //xxxBBxxx CHRG_STAT REGISTER, CHARGING MODE, 
uint8_t reg0C = 0; //READ ONLY

uint8_t reg0E = 0; //READ ONLY //Battery Voltage ADC, lower 7 bits, p.40 of DS

uint8_t reg0F = 0; //READ ONLY // SYSTEM VOLTAGE REGISTER, lower 7 bits
uint8_t reg10 = 0; //READ ONLY, USELESS
uint8_t reg11 = 0; //READ ONLY, First bit VBUS attached, lower 7 bits VBUS Voltage
uint8_t reg12 = 0; //READ ONLY, Lower 7 bits charge current
uint8_t reg13 = 0; //READ ONLY, lower 6 bits ICO ILIM

uint8_t reg0D = 0; // USeless

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(ADDR);
  writeToAllRegs();
  Serial.println("REGS SETUP!");
  if (readRawReg(1) == 197) {
    Serial.println("REG WRITE SUCCESS!");
  }
  else {
    Serial.println("ERROR OCCURED DURING WRITE!");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  
  
} 

// put function definitions here:
