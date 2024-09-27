#include <Arduino.h>


// put function declarations here:

#define BTN_PIN 1
#define EN_PIN 15
#define SDA_PIN 8
#define SCL_PIN 9
#define FAN_PIN 0
#define NTC_PIN 14

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
