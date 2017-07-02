/*
A simple temperature / humidity logger, using the Si7021.


ATtiny85 Hardware connections:

                               +--\/--+
                          D5  -|1    8|-  Vcc
  to FT230X pin 4 (RX)    D3  -|2    7|-  D2    to Si7021 SCL
  to FT230X pin 1 (TX)    D4  -|3    6|-  D1    to button
                         GND  -|4    5|-  D0    to Si7021 SDA
                               +------+

See also https://github.com/SpenceKonde/TinyWireM/blob/be34c42359b2142c347a1b7740291b6afff1249e/USI_TWI_Master.h
*/

#define RX_pin 3
#define TX_pin 4

/*
The following Si7021 "Weather" code was adapted from Sparkfun's library.
See https://github.com/sparkfun/Si7021_Breakout/tree/master/Libraries
*/

#include <Arduino.h>
#include "TinyWireM.h"

#define ADDRESS      0x40

#define TEMP_MEASURE_HOLD  0xE3
#define HUMD_MEASURE_HOLD  0xE5
#define TEMP_MEASURE_NOHOLD  0xF3
#define HUMD_MEASURE_NOHOLD  0xF5
#define TEMP_PREV   0xE0

#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

#define HTRE        0x02
#define _BV(bit) (1 << (bit))

#define CRC_POLY 0x988000 // Shifted Polynomial for CRC check

// Error codes
#define I2C_TIMEOUT   998
#define BAD_CRC   999

class Weather {
public:
  // Constructor
  Weather();

  bool  begin();

  // Si7021 & HTU21D Public Functions
  float getRH();
  float readTemp();
  float getTemp();
  void  heaterOn();
  void  heaterOff();
  void  changeResolution(uint8_t i);
  void  reset();
  uint8_t  checkID();

private:
  //Si7021 & HTU21D Private Functions
  uint16_t makeMeasurment(uint8_t command);
  void     writeReg(uint8_t value);
  uint8_t  readReg();
};

//Initialize
Weather::Weather(){}

bool Weather::begin(void) {
  TinyWireM.begin();

  uint8_t ID_Temp_Hum = checkID();

  int x = 0;

  if(ID_Temp_Hum == 0x15)//Ping CheckID register
    x = 1;
  else if(ID_Temp_Hum == 0x32)
    x = 2;
  else
    x = 0;

  if(x == 1) {
    return true;
  } else if(x == 2) {
    return true;
  } else {
    return false;
  }
}

float Weather::getRH() {
  // Measure the relative humidity
  uint16_t RH_Code = makeMeasurment(HUMD_MEASURE_NOHOLD);
  float result = (125.0*RH_Code/65536)-6;
  return result;
}

float Weather::readTemp() {
  // Read temperature from previous RH measurement.
  uint16_t temp_Code = makeMeasurment(TEMP_PREV);
  float result = (175.25*temp_Code/65536)-46.85;
  return result;
}

float Weather::getTemp() {
  // Measure temperature
  uint16_t temp_Code = makeMeasurment(TEMP_MEASURE_NOHOLD);
  float result = (175.25*temp_Code/65536)-46.85;
  return result;
}

void Weather::heaterOn() {
  // Turns on the ADDRESS heater
  uint8_t regVal = readReg();
  regVal |= _BV(HTRE);
  //turn on the heater
  writeReg(regVal);
}

void Weather::heaterOff() {
  // Turns off the ADDRESS heater
  uint8_t regVal = readReg();
  regVal &= ~_BV(HTRE);
  writeReg(regVal);
}

void Weather::changeResolution(uint8_t i) {
  // Changes to resolution of ADDRESS measurements.
  // Set i to:
  //      RH         Temp
  // 0: 12 bit       14 bit (default)
  // 1:  8 bit       12 bit
  // 2: 10 bit       13 bit
  // 3: 11 bit       11 bit

  uint8_t regVal = readReg();
  // zero resolution bits
  regVal &= 0b011111110;
  switch (i) {
    case 1:
      regVal |= 0b00000001;
      break;
    case 2:
      regVal |= 0b10000000;
      break;
    case 3:
      regVal |= 0b10000001;
    default:
      regVal |= 0b00000000;
      break;
  }
  // write new resolution settings to the register
  writeReg(regVal);
}

void Weather::reset() {
  //Reset user resister
  writeReg(SOFT_RESET);
}

uint8_t Weather::checkID() {
  uint8_t ID_1;

  // Check device ID
  TinyWireM.beginTransmission(ADDRESS);
  TinyWireM.write(0xFC);
  TinyWireM.write(0xC9);
  TinyWireM.endTransmission();

    TinyWireM.requestFrom(ADDRESS,1);

    ID_1 = TinyWireM.read();

    return(ID_1);
}

uint16_t Weather::makeMeasurment(uint8_t command) {
  // Take one ADDRESS measurement given by command.
  // It can be either temperature or relative humidity
  // TODO: implement checksum checking

  uint16_t nBytes = 3;
  // if we are only reading old temperature, read olny msb and lsb
  if (command == 0xE0) nBytes = 2;

  TinyWireM.beginTransmission(ADDRESS);
  TinyWireM.write(command);
  TinyWireM.endTransmission();
  // When not using clock stretching (*_NOHOLD commands) delay here
  // is needed to wait for the measurement.
  // According to datasheet the max. conversion time is ~22ms
   delay(100);

  TinyWireM.requestFrom(ADDRESS,nBytes);
  //Wait for data
  int counter = 0;
  while (TinyWireM.available() < nBytes){
    delay(1);
    counter ++;
    if (counter >100){
      // Timeout: Sensor did not return any data
      return 100;
    }
  }

  unsigned int msb = TinyWireM.read();
  unsigned int lsb = TinyWireM.read();
  // Clear the last to bits of LSB to 00.
  // According to datasheet LSB of RH is always xxxxxx10
  lsb &= 0xFC;
  unsigned int mesurment = msb << 8 | lsb;

  return mesurment;
}

void Weather::writeReg(uint8_t value) {
  // Write to user register on ADDRESS
  TinyWireM.beginTransmission(ADDRESS);
  TinyWireM.write(WRITE_USER_REG);
  TinyWireM.write(value);
  TinyWireM.endTransmission();
}

uint8_t Weather::readReg() {
  // Read from user register on ADDRESS
  TinyWireM.beginTransmission(ADDRESS);
  TinyWireM.write(READ_USER_REG);
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(ADDRESS,1);
  uint8_t regVal = TinyWireM.read();
  return regVal;
}


// a CRC-8 implementation, generated using pycrc, see https://pycrc.org/tutorial.html
// ./pycrc.py --model=crc-8 --algorithm=bbb --generate h -o crc8.h
// ./pycrc.py --model=crc-8 --algorithm=bbb --generate c -o crc8.c
// see also http://crccalc.com/
#include "crc8.h"

// a struct representing one message to send over the serial connection.
struct message_t {
  float temp_c;
  float humidity;
  uint8_t crc8;
};

// a union allowing easy access to the individual bytes in a message_t.
union packing_t {
  struct message_t msg;
  uint8_t bytes[9];
};

#include <SoftwareSerial.h>
SoftwareSerial serial = SoftwareSerial(RX_pin, TX_pin);

// format and transmit one datapoint over the serial connection.
void send(float temp_c, float humidity) {
  // assemble the message (initially with an empty CRC)
  struct message_t msg = { .temp_c = temp_c, .humidity = humidity, .crc8 = 0x0 };

  // package the message as an array bytes
  union packing_t packed = { .msg = msg };

  // generate the CRC and add it to the packed message
  uint8_t crc = crc_update(crc_init(), packed.bytes, 8);
  packed.msg.crc8 = crc_finalize(crc);

  // transmit the packed bytes
  for (int8_t i=0; i < 9; i++) {
    serial.write(packed.bytes[i]);
  }
}

// busy-wait until the specified interval since the given reference.
void delay_since(uint32_t interval, uint32_t then) {
  while (true) {
    // note: unsigned arithmetic automatically accounts for millis() roll-over.
    uint32_t now = millis();
    uint32_t elapsed = now - then;
    if (elapsed >= interval) {
      break;
    }
  }
}

// "main"

Weather sensor;

void setup() {
  pinMode(RX_pin, INPUT);
  pinMode(TX_pin, OUTPUT);
  serial.begin(9600);
  sensor.begin();
  delay(1);
  serial.write("Si7021-attiny\n");
}

void loop() {
  uint32_t then = millis();
  float humidity = sensor.getRH();
  float temp_c = sensor.getTemp();
  send(temp_c, humidity);
  delay_since(1000, then);
}

