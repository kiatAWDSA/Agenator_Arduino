/**********************************************************************
Obtain tacho readings (e.g. a fan) and convert it to RPM.
Utilizes the following chips:
- 74HC590 (counts the pulses from tachometer)
- 74HC165 (summarizes the counted pulses into a byte)

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

#include "TachoSensor.h"

TachoSensor::TachoSensor(PCA9685& PCA9685Manager,
                          const uint8_t& rclk,
                          const uint8_t& shld,
                          const uint8_t& cclr,
                          int bitOrder)
  :
  rclk_(rclk),
  shld_(shld),
  cclr_(cclr),
  bitOrder_(bitOrder)
{
  PCA9685Manager_ = &PCA9685Manager;
}

TachoSensor::~TachoSensor()
{
}

// The PCA9685 has a max speed of approx 1.5 kHz, which means one cycle is about 700 us.
// Therefore, we need to add delays between each command to ensure that at least one cycle is completed
// to give the appropriate signal to the pins
TACHO_STATUS TachoSensor::getMeasurement(double& buffer, unsigned long& readTimeStart)
{
  // Make sure clk is low first
  SPI485.writeClockLow();

  // SN74HC165 datasheet pg 7, section 6.10 (see t_pd, CLK to Q_H) and pg 11, Figure 2
  // suggests that the data is sampled at trailing edge of clock pulse
  // However, trial-and-error showed that sampling at rising edge gives proper readings
  SPI485.setSPIMode(false, false);

  // Trigger 74HC590 to move counts to output pins
  pca9685StatusBuffer_ = setPinHigh(rclk_);
  if (pca9685StatusBuffer_ != PCA9685_STATUS_OK)
  {
    return getTachoStatus(pca9685StatusBuffer_);
  }

  delayMicroseconds(700);

  pca9685StatusBuffer_ = setPinLow(rclk_);
  if (pca9685StatusBuffer_ != PCA9685_STATUS_OK)
  {
    return getTachoStatus(pca9685StatusBuffer_);
  }
  
  delayMicroseconds(700);

  // Tell 74HC165 to grab the data on 74HC590 output pins
  pca9685StatusBuffer_ = setPinLow(shld_);
  if (pca9685StatusBuffer_ != PCA9685_STATUS_OK)
  {
    return getTachoStatus(pca9685StatusBuffer_);
  }

  delayMicroseconds(700);

  pca9685StatusBuffer_ = setPinHigh(shld_);
  if (pca9685StatusBuffer_ != PCA9685_STATUS_OK)
  {
    return getTachoStatus(pca9685StatusBuffer_);
  }

  // Calculate the measurement interval
  unsigned long measurementInterval = millis() - readTimeStart;

  delayMicroseconds(700);

  // Send data in 74HC165 to Arduino
  if (!SPI485.receiveByte(readBuffer_, bitOrder_))
  {
    return TACHO_STATUS_SPI_TIMEOUT;
  }

  // Clear counts on 74HC590 to begin new measurement cycle
  clearCounts();

  // Beginning of a new measurement cycle
  readTimeStart = millis();

  // The AUB0812L-9X41 tacho outputs 2 pulses per rotation; see p7 of its datasheet
  // Instead of dividing the enumerator, multiply the denominator to avoid round-off errors
  //
  // measurementInterval is in ms; so do appropriate conversion to minutes:
  //    measurementInterval / 1000 / 60
  // Therefore, the fan speed in rpm is:
  //    (readBuffer_ / 2) / (measurementInterval / 1000 / 60)
  // Instead of dividing the denominator, multiply the numerator to avoid round-off errors
  // Merge the numbers and we get the code below
  // buffer = float(readBuffer_ )* 30000 / measurementInterval_;

  /*  NOTE: Strangely, the pulseCounter is consistently giving half of the actual value.
      I have probed the tacho output of the fan and determined each pulse is ~6 ms, which
      means one full rotation takes about 24~25 ms, thus giving 2400 rpm as in the datasheet.
      However, the raw data from pulseCounter is consistently 00010100 (00101000 = 40 after fixing the LSBFIRST),
      when it should be 00001010 (01010000 = 80 after fixing the LSBFIRST). I have checked the
      sequence of pin togglings; everything is working as they should.

      Easy band-aid is to just multiply the measurement by 2. This works well because
      the readings are consistently half of the actual reading (at least at full speed; I don't know
      at lower speeds). If I have time in the future, I could try debugging this further.

      New implementation of fanBreakout with isolator and TSSOP chips. Changing back to original equation.
  */
  // buffer = float(readBuffer_ )* 60000 / measurementInterval_;
  buffer = float(readBuffer_) * 30000 / measurementInterval;
  return TACHO_STATUS_OK;
}

// Clears the pulse counter. Can be called externally to reset the count (say, when starting a new measurement)
TACHO_STATUS TachoSensor::clearCounts()
{
  pca9685StatusBuffer_ = setPinLow(cclr_);
  if (pca9685StatusBuffer_ != PCA9685_STATUS_OK)
  {
    return getTachoStatus(pca9685StatusBuffer_);
  }

  delayMicroseconds(700);

  pca9685StatusBuffer_ = setPinHigh(cclr_);
  if (pca9685StatusBuffer_ != PCA9685_STATUS_OK)
  {
    return getTachoStatus(pca9685StatusBuffer_);
  }
}

// Get the tacho error code from a given PCA9685 error code
TACHO_STATUS TachoSensor::getTachoStatus(PCA9685_STATUS pca9685Status)
{
  switch (pca9685Status) {
  case PCA9685_STATUS_I2C_START:
    return TACHO_STATUS_I2C_START;
    break;
  case PCA9685_STATUS_I2C_ACKNACK_MODE:
    return TACHO_STATUS_I2C_ACKNACK_MODE;
    break;
  case PCA9685_STATUS_I2C_ACKNACK:
    return TACHO_STATUS_I2C_ACKNACK;
    break;
  case PCA9685_STATUS_I2C_REPSTART:
    return TACHO_STATUS_I2C_REPSTART;
    break;
  case PCA9685_STATUS_I2C_ACKNACK_RECMODE:
    return TACHO_STATUS_I2C_ACKNACK_RECMODE;
    break;
  case PCA9685_STATUS_I2C_ACKNACK_REC:
    return TACHO_STATUS_I2C_ACKNACK_REC;
    break;
  case PCA9685_STATUS_I2C_STOP:
    return TACHO_STATUS_I2C_STOP;
    break;
  case PCA9685_STATUS_I2C_OTHER:
    return TACHO_STATUS_I2C_OTHER;
  default:
    return TACHO_STATUS_OTHER;
    break;
  }
}

// Attempt to set the given PCA9685 pin to HIGH. As many as RETRIES_MAX attempts are made.
PCA9685_STATUS TachoSensor::setPinHigh(uint8_t pin)
{
  uint8_t attempts = 0;

  while (attempts < RETRIES_MAX)
  {
    pca9685StatusBuffer_ = PCA9685Manager_->setChannelOn(pin);
    attempts++;

    if (pca9685StatusBuffer_ == PCA9685_STATUS_OK)
    {
      return pca9685StatusBuffer_;
    }
  }
  
  // There is an error if we finished the while loop. Let the calling function handle it
  return pca9685StatusBuffer_;
}

// Attempt to set the given PCA9685 pin to LOW. As many as RETRIES_MAX attempts are made.
PCA9685_STATUS TachoSensor::setPinLow(uint8_t pin)
{
  uint8_t attempts = 0;

  while (attempts < RETRIES_MAX)
  {
    pca9685StatusBuffer_ = PCA9685Manager_->setChannelOff(pin);
    attempts++;

    if (pca9685StatusBuffer_ == PCA9685_STATUS_OK)
    {
      return pca9685StatusBuffer_;
    }
  }

  // There is an error if we finished the while loop. Let the calling function handle it
  return pca9685StatusBuffer_;
}
