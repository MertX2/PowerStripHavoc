#include <Arduino.h>
#include <Wire.h>
#include <tinyNeoPixel_Static.h>

// put function declarations here:

#define BTN_PIN PIN_PA5
#define EN_PIN PIN_PA2
#define SDA_PIN 8
#define SCL_PIN 9
#define FAN_PIN PIN_PA4
#define NTC_PIN PIN_PA1

// KELVIN = 273.15

bool fanOn = 0;

//neopixel static uses less ressources, so less power??? see megatinycore extra docs
byte pixels[12];
tinyNeoPixel leds = tinyNeoPixel(4, PIN_PA7, NEO_GRB, pixels);

const uint8_t reg00 = 63;

const uint8_t reg01 = 197; //11000101

const uint8_t reg02 = 48; //00110000  (No ADC)
const uint8_t reg02_ADC_EN = 176; //or 10110000 (Start ADC Conversion)

const uint8_t reg03 = 26; //00011010
const uint8_t reg04 = 127;  //01111111
const uint8_t reg05 = 19; //00010011
const uint8_t reg06 = 98; //01100010
const uint8_t reg07 = 201; //11001001
const uint8_t reg08 = 3; //00000011
const uint8_t reg09 = 64; //01000000
const uint8_t reg0a = 144; //10010000

uint8_t reg0B = 0; //READ ONLY //xxxBBxxx CHRG_STAT REGISTER, CHARGING MODE, 
uint8_t reg0C = 0; //READ ONLY

uint8_t reg0E = 0; //READ ONLY //Battery Voltage ADC, lower 7 bits, p.40 of DS

uint8_t reg0F = 0; //READ ONLY // SYSTEM VOLTAGE REGISTER, lower 7 bits
uint8_t reg10 = 0; //READ ONLY, USELESS
uint8_t reg11 = 0; //READ ONLY, First bit VBUS attached, lower 7 bits VBUS Voltage
uint8_t reg12 = 0; //READ ONLY, Lower 7 bits charge current
uint8_t reg13 = 0; //READ ONLY, lower 6 bits ICO ILIM

uint8_t reg0D = 0; // USeless

uint32_t intermediate = 1023UL * 1500;

uint8_t ADDR = 106;

uint16_t getAnalogValue() {
 return ADC0.RES / 2; // return the most recent result on channel A
}

void writeToReg(uint8_t	REG_ADDR, uint8_t REG_DATA) {
  Wire.beginTransmission(ADDR);
  Wire.write((uint8_t)REG_ADDR);
  Wire.write(REG_DATA);
  Wire.endTransmission();
}

uint8_t readRawReg(uint8_t REG_ADDR) {
  
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
  b[0] = 0 != (regData & 8);
  b[1] = 0 != (regData & 16);
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
  b[0] = 0 != (regData & 1);
  b[1] = 0 != (regData & 2);
  b[2] = 0 != (regData & 4);
  b[0] = 0 != (regData & 8);
  b[1] = 0 != (regData & 16);
  b[5] = 0 != (regData & 32);
  b[6] = 0 != (regData & 64);
  //b[7] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  adcVal += b[0] * 20;
  adcVal += b[1] * 40;
  adcVal += b[2] * 80;
  adcVal += b[3] * 160;
  adcVal += b[4] * 320;
  adcVal += b[5] * 640;
  adcVal += b[6] * 1280;
  return adcVal;
}

uint16_t getSysVoltage() { //REG0F
  enableBQADC();
  bool b[7];
  uint8_t regData = readRawReg(0x0F);
  b[0] = 0 != (regData & 1);
  b[1] = 0 != (regData & 2);
  b[2] = 0 != (regData & 4);
  b[0] = 0 != (regData & 8);
  b[1] = 0 != (regData & 16);
  b[5] = 0 != (regData & 32);
  b[6] = 0 != (regData & 64);
  //b[7] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  adcVal += b[0] * 20;
  adcVal += b[1] * 40;
  adcVal += b[2] * 80;
  adcVal += b[3] * 160;
  adcVal += b[4] * 320;
  adcVal += b[5] * 640;
  adcVal += b[6] * 1280;
  return adcVal;
}

uint16_t getVBUSVoltage() { //REG11
  enableBQADC();
  bool b[7];
  uint8_t regData = readRawReg(0x11);
  b[0] = 0 != (regData & 1);
  b[1] = 0 != (regData & 2);
  b[2] = 0 != (regData & 4);
  b[0] = 0 != (regData & 8);
  b[1] = 0 != (regData & 16);
  b[5] = 0 != (regData & 32);
  b[6] = 0 != (regData & 64);
  //b[7] = 0 != (regData & 128);

  uint16_t adcVal = 2600;
  adcVal += b[0] * 100;
  adcVal += b[1] * 200;
  adcVal += b[2] * 400;
  adcVal += b[3] * 800;
  adcVal += b[4] * 1600;
  adcVal += b[5] * 3200;
  adcVal += b[6] * 6400;
  return adcVal;
}

bool getVBUSAttached() { //REG11
  bool b = 0;
  uint8_t regData = readRawReg(0x11);
  b = 0 != (regData & 128);
  return b;
}

uint16_t getChrgCurrent() { //REG12
  enableBQADC();
  bool b[7];
  uint8_t regData = readRawReg(0x12);
  b[0] = 0 != (regData & 1);
  b[1] = 0 != (regData & 2);
  b[2] = 0 != (regData & 4);
  b[0] = 0 != (regData & 8);
  b[1] = 0 != (regData & 16);
  b[5] = 0 != (regData & 32);
  b[6] = 0 != (regData & 64);
  //b[7] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  adcVal += b[0] * 50;
  adcVal += b[1] * 100;
  adcVal += b[2] * 200;
  adcVal += b[3] * 400;
  adcVal += b[4] * 800;
  adcVal += b[5] * 1600;
  adcVal += b[6] * 3200;
  return adcVal;
}

uint16_t getICOILIM() { //REG13
  enableBQADC();
  bool b[6];
  uint8_t regData = readRawReg(0x12);
  b[0] = 0 != (regData & 1);
  b[1] = 0 != (regData & 2);
  b[2] = 0 != (regData & 4);
  b[0] = 0 != (regData & 8);
  b[1] = 0 != (regData & 16);
  b[5] = 0 != (regData & 32);
  //b[6] = 0 != (regData & 64);
  //b[7] = 0 != (regData & 128);

  uint16_t adcVal = 2304;
  adcVal += b[0] * 50;
  adcVal += b[1] * 100;
  adcVal += b[2] * 200;
  adcVal += b[3] * 400;
  adcVal += b[4] * 800;
  adcVal += b[5] * 1600;
  return adcVal;
}

uint8_t getTemp() {
  analogReference(INTERNAL2V5); // set reference to the desired voltage, and set that as the ADC reference.
  analogReference(VDD); // Set the ADC reference to VDD. Voltage selected previously is still the selected, just not set as the ADC reference.
  uint16_t vdd = intermediate / analogRead(ADC_INTREF);
  analogReference(INTERNAL2V5);

  uint16_t average = getAnalogValue();
  average = 1023 / average - 1;
  average = 10000 / average;

  float steinhart;
  steinhart = average / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3380;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  return steinhart;    
}


void setup() {
  ADC0.MUXPOS = 0x01; //reads from PA6/arduino pin 2, ADC0 channel 6
  ADC0.CTRLA = ADC_ENABLE_bm|ADC_FREERUN_bm; //start in freerun
  ADC0.CTRLB = ADC_SAMPNUM_ACC2_gc;
  ADC0.COMMAND=ADC_STCONV_bm; //start first conversion!

  // Energysaving Measure, see megatinycore extra docs
  pinModeFast(PIN_PA3, OUTPUT);
  pinModeFast(PIN_PC1, OUTPUT);
  pinModeFast(PIN_PC2, OUTPUT);
  pinModeFast(PIN_PC3, OUTPUT);
  pinModeFast(PIN_PB4, OUTPUT);
  pinModeFast(PIN_PB5, OUTPUT);
  //Dont know why, just felt right :)
  turnOffPWM(PIN_PA3);
  turnOffPWM(PIN_PC1);
  turnOffPWM(PIN_PA5);
  //Neopixel pin setup
  pinModeFast(PIN_PA7, OUTPUT);
  leds.setPixelColor(0, 255, 0, 0); // first LED full RED
  leds.show(); 
  
  pinModeFast(FAN_PIN, OUTPUT);

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

void fanCheck() {
  if(getTemp() > 333) {
    if(fanOn == 0) {
      analogWrite(FAN_PIN, HIGH);
      fanOn = 1;
    }
  }
  else if(getTemp() < 320) {
    if(fanOn == 1) {
      analogWrite(FAN_PIN, LOW);
      fanOn = 0;
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
} 

// put function definitions here:
