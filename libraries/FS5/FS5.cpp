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

#include "FS5.h"

// Initialize with the air velocity` model parameters
FS5::FS5(float mean, float stdDev, float x0, float x1, float x2, float x3)
  :
  mean_(mean),
  stdDev_(stdDev),
  x0_(x0),
  x1_(x1),
  x2_(x2),
  x3_(x3)
{}

FS5::~FS5()
{
}

// Call this function before getMeasurement()
void FS5::triggerMeasurement()
{
  // For 2-wire operation (clk and dat pins; this is NOT I2C!),
  // need to send 16 clock pulses to get a new conversion. This is actually
  // the same procedure as getting a measurement, but we are ignoring
  // whatever comes out on the data line.
  // At the end of the 16th pulse, the ADC will begin a new conversion.
  // See p13 of LTC2452 datasheet for more info.
  // There will be some delay (max 23 ms, p4 of datasheet) after
  // this triggering before the new measurement is available.

  // Make sure clk is low first
  SPI485.writeClockLow();
  
  for (uint8_t i = 0; i < 16; i++)
  {
    SPI485.pulseClock();
  }
}

// Grabs the value in LTC2452, converts it to decimal, and return it.
bool FS5::getMeasurement(double& buffer)
{
  // Make sure clk is low first
  SPI485.writeClockLow();

  // LTC2452 datasheet Figure 4 & 14 suggests that the data is sampled at leading edge
  // of clock pulse if clock idles LOW
  SPI485.setSPIMode(false, false);

  for (uint8_t i = 0; i < 2; i++)
  {
    if (!SPI485.receiveByte(readBuffer_[i]))
    {
      return false;
    }
  }

  dataBuffer_ = readBuffer_[0];
  dataBuffer_ <<= 8; // Shift one byte to left
  dataBuffer_ |= readBuffer_[1];

#ifndef CALIBRATE_VELOCITY
  // Equation determined by calibration
  float scaledSignal = (dataBuffer_ - mean_) / stdDev_;
  buffer = x3_*pow(scaledSignal, 3) + x2_*pow(scaledSignal, 2) + x1_*scaledSignal + x0_;
#else
  // Return raw sensor values if calibrating the FS5 sensor
  buffer = float(dataBuffer_);
#endif
  return true;
}
