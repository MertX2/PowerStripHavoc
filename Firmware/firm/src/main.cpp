#include <Arduino.h>
#include <Wire.h>
#include <tinyNeoPixel_Static.h>
#include <avr/sleep.h>
#include <bq.hpp>

#define DEBUG

#define BTN_PIN PIN_PA5
#define EN_PIN PIN_PA2
#define SDA_PIN 8
#define SCL_PIN 9
#define FAN_PIN PIN_PA4
#define NTC_PIN PIN_PA1
#define U10_PIN PIN_PB3

// KELVIN = 273.15

bool fanOn = 0;

//neopixel static uses less ressources, so less power??? see megatinycore extra docs
byte pixels[12];
tinyNeoPixel leds = tinyNeoPixel(4, PIN_PA7, NEO_GRB, pixels);

uint8_t batStat = 0;
bool cycle = 0;

void neoPixelTask() {
  switch (batStat)
  {
  case 6:
    if(cycle) {
      cycle = 0;
      leds.setPixelColor(0, 0, 255, 0);
      leds.show();
    }
    else {
      cycle = 1;
      leds.setPixelColor(0, 0, 0, 0);
      leds.show();
    }
    break;
  case 5:
    leds.setPixelColor(0, 0, 255, 0);
    leds.show();
    break;
  case 4:
    leds.setPixelColor(0, 127, 255, 0);
    leds.show();
    break;
  case 3:
    leds.setPixelColor(0, 255, 127, 0);
    leds.show();
    break;
  case 2:
    leds.setPixelColor(0, 255, 0, 0);
    leds.show();
    break;
  case 1:
    if(cycle) {
      cycle = 0;
      leds.setPixelColor(0, 255, 0, 0);
      leds.show();
    }
    else {
      cycle = 1;
      leds.setPixelColor(0, 0, 0, 0);
      leds.show();
    }
    break;
  default:
    break;
  }
}

uint32_t intermediate = 1023UL * 1500;

uint16_t readSupplyVoltage() { // returns value in millivolts to avoid floating point //NUR NÖTIG, WENN BQ VSYS SICH NICHT FESTLEGEN LÄSST !!!
    analogReference(VDD);
    VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;
    // there is a settling time between when reference is turned on, and when it becomes valid.
    // since the reference is normally turned on only when it is requested, this virtually guarantees
    // that the first reading will be garbage; subsequent readings taken immediately after will be fine.
    // VREF.CTRLB|=VREF_ADC0REFEN_bm;
    // delay(10);
    uint16_t reading = analogRead(ADC_INTREF); // Crumple the reading up into a ball and toss in the general direction of the trash.
    reading = analogRead(ADC_INTREF);          // Now we take the *real* reading.
    uint32_t intermediate = 1023UL * 1500;     // This would overflow a 16-bit variable.
    reading = intermediate / reading;          // Long division sucks! This single line takes about 600 clocks to execute!
    // And **this** is why the reference voltages on the 0/1-series parts suck! The comparable math on the 2-series takes maybe
    // 40 clocks, at most!
    return reading;
}

uint16_t getAnalogValue() {
 return ADC0.RES / 2; // return the most recent result on channel A
}

uint8_t getTemp() {
  uint16_t vdd = intermediate / readSupplyVoltage(); //NUR NÖTIG, WENN BQ VSYS SICH NICHT FESTLEGEN LÄSST !!!
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
//This will propably get axed.
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

void readAllRegs() {
  Serial.println("STARTING READ");
  Serial.println("00");
  Serial.println(readRawReg(0x00));
  Serial.println("01");
  Serial.println(readRawReg(0x01));
  Serial.println("02");
  Serial.println(readRawReg(0x02));
  Serial.println("03");
  Serial.println(readRawReg(0x03));
  Serial.println("04");
  Serial.println(readRawReg(0x04));
  Serial.println("05");
  Serial.println(readRawReg(0x05));
  Serial.println("06");
  Serial.println(readRawReg(0x06));
  Serial.println("07");
  Serial.println(readRawReg(0x07));
  Serial.println("08");
  Serial.println(readRawReg(0x08));
  Serial.println("09");
  Serial.println(readRawReg(0x09));
  Serial.println("0A");
  Serial.println(readRawReg(0x0A));
  Serial.println("0B");
  Serial.println(readRawReg(0x0B));
  Serial.println("0C");
  Serial.println(readRawReg(0x0C));
  Serial.println("0E");
  Serial.println(readRawReg(0x0E));
  Serial.println("0F");
  Serial.println(readRawReg(0x0F));
  Serial.println("READ END!");
}
//Interrupt to wake the chip up from sleep whenever power button is pressed.
/*ISR(PORTA_PORT_vect)
{
  return;
}*/

//Button timing variable
unsigned long startMillis = 0;


void goToSleep();

void wakeSetup() {
  Serial.println("WK0");
  startMillis = millis();
  while(digitalReadFast(BTN_PIN) == LOW) {
    if(uint16_t(millis - startMillis) > 2000) {
      Serial.println("WK1");
      ADC0.CTRLA |= 1;
      digitalWriteFast(EN_PIN, HIGH);
      delay(150);
      digitalWriteFast(FAN_PIN, HIGH);
      //PORTA.PIN2CTRL = 0x08;
      //VPORTA.INTFLAGS = 255; 
      Serial.println("RT");
      return;
    }
  }
  goToSleep(); 
}

void goToSleep() {
  ADC0.CTRLA &= ~ADC_ENABLE_bm;
  PORTA.PIN2CTRL |= PORT_ISC_FALLING_gc;
  if(!regWriteCheck) {
    writeToAllRegs();
  }
  Serial.println("SLP");
  Serial.flush();
  delay(100);
  sleep_cpu();
  //wakeSetup();
}

void powerButtonCheck() {
  startMillis = millis();
  while(digitalReadFast(BTN_PIN) == LOW) {
    if(uint16_t(millis - startMillis) > 2000) {
        goToSleep();
    }
  }
  return;
}

void setup() {
  //Sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  //ADC setup
  ADC0.MUXPOS = 0x01; //reads from PA1
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
  //Neopixel pin
  pinModeFast(PIN_PA7, OUTPUT);
  //Rest of the pins
  pinModeFast(BTN_PIN, INPUT_PULLUP);
  pinModeFast(EN_PIN, OUTPUT);
  pinModeFast(FAN_PIN, OUTPUT);
  pinModeFast(NTC_PIN, INPUT);
  pinModeFast(U10_PIN, OUTPUT);
  //Serial and I2C init
  Serial.begin(9600);
  Wire.swap(0);
  Wire.begin();
  Wire.setClock(100000);
  //BQ Write
  Serial.println("SET");
  if(writeToAllRegs() == 0) {
    Serial.println("WERR");
    delay(100);
    Serial.println("WERR");
    delay(100);
    Serial.println("WERR");
    delay(100);
    Serial.println("WERR");
    delay(100);
    Serial.println("WERR");
  }
  //readAllRegs();
  attachInterrupt(digitalPinToInterrupt(BTN_PIN),wakeSetup,FALLING);
  goToSleep();
}




//Tasktimer.
unsigned long loopMillis = 0;

void loop() {
  if(millis() - loopMillis > 2000) {
    loopMillis = millis();
    //fanCheck();
    neoPixelTask();
    #ifdef DEBUG
    Serial.print("ICHG");
    Serial.println(String(getChrgCurrent()));
    Serial.print("VBA");
    Serial.println(String(getVBUSAttached()));
    Serial.print("VBV");
    Serial.println(String(getVBUSVoltage()));
    Serial.print("SYSV");
    Serial.println(String(getSysVoltage()));
    Serial.print("BAT");
    Serial.println(String(getBatVoltage()));
    #endif
  }
  powerButtonCheck();
  uint16_t batV = getBatVoltage();
  if (3610 < batV) {
    batStat = 1;
  }
  if (3750 < batV) {
    batStat = 2;
  }
  if (3840 < batV) {
    batStat = 3;
  }
  if (3980 < batV) {
    batStat = 4;
  }
  if (4150 < batV) {
    batStat = 5;
  }
  uint8_t chrgStat = getChrgStatus();
  if(chrgStat == 3 || chrgStat == 4) {
    cycle = 1;
    batStat = 6;
    neoPixelTask();
  }
  
  
}
  // put your main code here, to run repeatedly:
  


  
  


// put function definitions here:
