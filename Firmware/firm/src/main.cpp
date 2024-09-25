#include <Arduino.h>


// put function declarations here:

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
uint8_t reg10 = 128;
uint8_t reg11 = 4;
uint8_t reg12 = 67;
uint8_t reg13 = 9;
uint8_t reg14 = 134;
uint8_t reg15 = 88;
uint8_t reg16 = 90;
uint8_t reg17 = 30;
uint8_t reg18 = 90;
uint8_t reg19 = 30;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(reg00);
  Serial.println(reg01);
  Serial.println(reg02);
  Serial.println(reg03);
  Serial.println(reg04);
  Serial.println(reg05);
  Serial.println(reg06);
  Serial.println(reg07);
  Serial.println(reg08);
  
}

// put function definitions here:
