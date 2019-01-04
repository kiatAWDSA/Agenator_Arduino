/**********************************************************************
Measure fluid velocity using the IST FS5 sensor. This library assumes
the FS5 is connected to the LTC 2452 ADC.

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

