/**********************************************************************
  Measure fluid velocity using the IST FS5 sensor. This library assumes
  the FS5 is connected to the LTC 2452 ADC.

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

#ifndef _FS5_h
#define _FS5_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <RS485SPI.h>

// Uncomment only if calibrating FS5 sensor
// Do the same for the HumidityChamber.h file.
// #define CALIBRATE_VELOCITY 1

class FS5
{
public:
  FS5(float mean, float stdDev, float x0, float x1, float x2, float x3);
  ~FS5();
  void triggerMeasurement();
  bool getMeasurement(double& buffer);

private:
  uint8_t readBuffer_[2];
  uint16_t dataBuffer_;

  // Variables for the air velocity model
  const float mean_;
  const float stdDev_;
  const float x0_;
  const float x1_;
  const float x2_;
  const float x3_;
};

#endif

