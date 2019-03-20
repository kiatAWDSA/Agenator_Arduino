/**********************************************************************
Obtain tacho readings (e.g. a fan) and convert it to RPM.
Utilizes the following parts:
- Delta Electronics AUB0812L-9X41 DC fan
- 74HC590 (counts the pulses from tachometer)
- 74HC165 (summarizes the counted pulses into a byte)

  Copyright (C) 2019 Soon Kiat Lau

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
***********************************************************************/

#ifndef _TachoSensor_h
#define _TachoSensor_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <RS485SPI.h>
#include <I2C.h>
#include <PCA9685_customI2C.h>

typedef enum
{
  TACHO_STATUS_OK                   = 0,   // No problemo
  TACHO_STATUS_I2C_START            = 1,   // I2C timeout while waiting for successful completion of a Start bit
  TACHO_STATUS_I2C_ACKNACK_MODE     = 2,   // I2C timeout while waiting for ACK/NACK while addressing slave in transmit mode (MT)
  TACHO_STATUS_I2C_ACKNACK          = 3,   // I2C timeout while waiting for ACK/NACK while sending data to the slave
  TACHO_STATUS_I2C_REPSTART         = 4,   // I2C timeout while waiting for successful completion of a Repeated Start
  TACHO_STATUS_I2C_ACKNACK_RECMODE  = 5,   // I2C timeout while waiting for ACK/NACK while addressing slave in receiver mode (MR)
  TACHO_STATUS_I2C_ACKNACK_REC      = 6,   // I2C timeout while waiting for ACK/NACK while receiving data from the slave
  TACHO_STATUS_I2C_STOP             = 7,   // I2C timeout while waiting for successful completion of the Stop bit
  TACHO_STATUS_I2C_OTHER            = 8,   // "See datasheet [of microcontroller chip] for exact meaning"
  TACHO_STATUS_SPI_TIMEOUT          = 9,   // Timeout while waiting to receive a byte from SPI
  TACHO_STATUS_OTHER                = 10,  // Something not documented yet
} TACHO_STATUS;

class TachoSensor
{
public:
  TachoSensor(PCA9685& PCA9685Manager,
              const uint8_t& rclk,
              const uint8_t& shld,
              const uint8_t& cclr,
              int bitOrder); // Determines if bytes should be read Most Significant Bit first, or Least Significant Bit first);
  ~TachoSensor();
  TACHO_STATUS getMeasurement(double& buffer, unsigned long& readTimeStart);
  TACHO_STATUS clearCounts();

private:
  PCA9685 * PCA9685Manager_;
  const uint8_t RETRIES_MAX = 3;
  const uint8_t rclk_;
  const uint8_t shld_;
  const uint8_t cclr_;
  const uint8_t bitOrder_;
  uint8_t readBuffer_ = 0;
  PCA9685_STATUS pca9685StatusBuffer_;

  TACHO_STATUS getTachoStatus(PCA9685_STATUS pca9685Status);
  PCA9685_STATUS setPinHigh(uint8_t pin);
  PCA9685_STATUS setPinLow(uint8_t pin);
};

#endif

