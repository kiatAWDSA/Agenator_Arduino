/*
  Library for SPI communication over RS-485 lines with Arduino.
  As of right now, this library only emulates sending clock pulses
  and receiving the data. Full SPI functionality is not implemented yet.

  Readings:
  http://www.ti.com/lit/an/slyt441/slyt441.pdf
  http://www.deathbylogic.com/2014/11/spi-over-rs-485-testing/
  http://avrbeginners.net/architecture/spi/spi.html
  https://www.arduino.cc/en/Tutorial/SPIEEPROM

  Usage:
  1) Connect the outgoing clock wire to clkPin
  2) Connect the incoming clock and data wire to the SCK and MOSI pins
  *    of the Arduino. Refer to https://www.arduino.cc/en/Reference/SPI
  3) Call begin() in the setup function. Call receiveByte for a single byte.

  Copyright 2018 Soon Kiat Lau

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef _RS485SPI_h
#define _RS485SPI_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define SPIMODE_0   0b00000000
#define SPIMODE_1   0b00000100
#define SPIMODE_2   0b00001000
#define SPIMODE_3   0b00001100
#define SPI_MASTER  0b00000001
#define SPI_SLAVE   0b00000000


class RS485SPI
{
public:
  RS485SPI();
  
  /*
    Starts the SPI bus.
    Choose from SPI_MASTER or SPI_SLAVE for SPI_ROLE.
    Choose from SPIMODE_0 thru SPIMODE_3 for SPI_MODE. Refer to Arduino SPI documentation.
  */
  void begin(uint8_t clkOutPin, uint8_t clkInPin, uint8_t misoPin, uint8_t mosiPin, uint8_t ssPin, uint8_t spiRole, unsigned long timeOut);
  void end();
  void enableSPI();
  void disableSPI();
  void setTimeOut(unsigned long timeOut);
  //bool receiveByte(uint8_t& buffer);
  void setSPIMode(bool idleHigh, bool sampleTrailing);
  bool receiveByte(uint8_t& buffer, uint8_t bitOrder = MSBFIRST);
  int readMOSI();
  void writeClockHigh();
  void writeClockLow();
  void pulseClock();


private:
  uint8_t clkOutPin_;
  uint8_t mosiPin_;
  unsigned long timeOut_;
  unsigned long startTime_;
  void setBitOrder(uint8_t bitOrder);
};

extern RS485SPI SPI485;

#endif

