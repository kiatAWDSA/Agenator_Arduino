/**********************************************************************
Obtain measurements from a Honeywell HIH8000 Series sensor connected to an
Arduino board using I2C protocol (not SPI):
https://sensing.honeywell.com/sensors/humidity-sensors/HIH8000-series

Uses a custom I2C library from DSSCircuits because Arduino Wire is blocking.

To adjust the internal settings of the
sensor such as its I2C address and alarms, use the HIH8000CommandI2C library instead:
https://github.com/kiatAWDSA/HIH8000Command_I2C

Please note that the pins used for I2C communication are different for each
Arduino board. For Arduino Uno, the SDA and SCL pins are A4 and A5, respectively.
A list of boards and corresponding pins can be found at:
https://www.arduino.cc/en/Reference/Wire


Copyright 2017 Soon Kiat Lau

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

#include "HIH8000_customI2C.h"

// Instantiate with device address.
HIH8000_customI2C::HIH8000_customI2C(uint8_t address) {
  setAddress(address);
}

// If user prefers to instantiate class first and set the address later.
HIH8000_customI2C::HIH8000_customI2C() {
  addressSet_ = false;
}

// No specific destruction subroutines necessary (ending I2C here might affect other devices on I2C line).
HIH8000_customI2C::~HIH8000_customI2C() {}

// Set/change I2C address of the sensor
// Address MUST be 7-bits (max value is 0x7F i.e. 127)
void HIH8000_customI2C::setAddress(uint8_t newAddress) {
  if (newAddress <= 0x7F) { // TODO should also check if not conflicting with Arduino's reserved addresses
    address_ = newAddress;
    addressSet_ = true;
  }
}

// To initiate a measurement, need to send a write request to the sensor without any data. This is equivalent to the ping function in the modified custom I2C library.
// NOTE: It typically takes 36.65 ms for a reading to be generated.
HIH8000_STATUS HIH8000_customI2C::triggerMeasurement() {
  if (addressSet_) {
    endTransErr_ = I2c.ping(address_);

    if (endTransErr_ == I2C_STATUS_OK) { // Should return 0 if success, otherwise something is wrong
      return HIH8000_STATUS_OK;
    }
    else {
      return getI2CError(endTransErr_);
    }
  }
  else {
    return HIH8000_STATUS_ADDRESS;
  }
}

// Get the data stored in the sensor's registers and process them. These data is from the most recent triggered measurement.
HIH8000_STATUS HIH8000_customI2C::fetchMeasurement() {
  if (addressSet_) {
    endTransErr_ = I2c.read(address_, SENSOR_BYTES);

    if (endTransErr_ == I2C_STATUS_OK) {
      // Get raw humidity data
      humidityBuffer_ = I2c.receive();
      humidityBuffer_ <<= 8;
      humidityBuffer_ |= I2c.receive();

      // Get raw temperature data
      temperatureBuffer_ = I2c.receive();
      temperatureBuffer_ <<= 8;
      temperatureBuffer_ |= I2c.receive();
      temperatureBuffer_ >>= 2;  // Remove the last two "Do Not Care" bits

                                 // Extract out status bits and clean raw humidity data
      status_ = MASK_STATUS & humidityBuffer_;            // MASK_STATUS is 1100000000000000 in binary
      status_ >>= 14;                                     // Push it to become 2 digits
      humidityBuffer_ = MASK_HUMIDITY & humidityBuffer_;  // MASK_HUMIDITY is 0011111111111111 in binary

                                                          // Ensure raw values are within the range of the sensor (could be distorted due to noise)
      if (humidityBuffer_ > SENSOR_RANGE) {
        humidityBuffer_ = SENSOR_RANGE;
      }

      if (temperatureBuffer_ > SENSOR_RANGE) {
        temperatureBuffer_ = SENSOR_RANGE;
      }

      // Convert raw values to actual readings. See the datasheet.
      // Apply two-point calibration corrections to relative humidity reading, and constrain the value.
      humidity_ = constrain((float(humidityBuffer_) / SENSOR_RANGE * 100) * slopeAdjuster_ + offset_, 0, 100);
      temperature_ = float(temperatureBuffer_) / SENSOR_RANGE * 165 - 40;

      return HIH8000_STATUS_OK;

    }
    else {
      return getI2CError(endTransErr_);
    }
  }
  else {
    return HIH8000_STATUS_ADDRESS;
  }
}

// Get the address set for the sensor.
uint8_t HIH8000_customI2C::getAddress() {
  return address_;
}

// Get the status code given by the sensor.
uint8_t HIH8000_customI2C::getStatus() {
  return status_;
}

// Get the processed humidity value.
float HIH8000_customI2C::getHumidity() {
  return humidity_;
}

// Get the processed temperature value.
float HIH8000_customI2C::getTemperature() {
  return temperature_;
}

// Saves the theoretical humidity and measured humidity, and calculate the correction parameters
void HIH8000_customI2C::applyHumidityCorrection(bool point1, double humidityRaw, double humidityStd) {
  if (point1) {
    humidityRaw1_ = humidityRaw;
    humidityStd1_ = humidityStd;
  }
  else {
    humidityRaw2_ = humidityRaw;
    humidityStd2_ = humidityStd;
  }

  slopeAdjuster_ = (humidityStd2_ - humidityStd1_) / (humidityRaw2_ - humidityRaw1_);
  offset_ = humidityStd2_ - slopeAdjuster_ * humidityRaw2_;
}

HIH8000_STATUS HIH8000_customI2C::getI2CError(uint8_t errorNumber) {
  switch (errorNumber) {
  case I2C_STATUS_START:
    return HIH8000_STATUS_I2C_START;
    break;
  case I2C_STATUS_ACKNACK_MODE:
    return HIH8000_STATUS_I2C_ACKNACK_MODE;
    break;
  case I2C_STATUS_ACKNACK:
    return HIH8000_STATUS_I2C_ACKNACK;
    break;
  case I2C_STATUS_REPSTART:
    return HIH8000_STATUS_I2C_REPSTART;
    break;
  case I2C_STATUS_ACKNACK_RECMODE:
    return HIH8000_STATUS_I2C_ACKNACK_RECMODE;
    break;
  case I2C_STATUS_ACKNACK_REC:
    return HIH8000_STATUS_I2C_ACKNACK_REC;
    break;
  case I2C_STATUS_STOP:
    return HIH8000_STATUS_I2C_STOP;
    break;
  case I2C_STATUS_OTHER:
    return HIH8000_STATUS_I2C_OTHER;
  default:
    return HIH8000_STATUS_OTHER;
    break;
  }
}