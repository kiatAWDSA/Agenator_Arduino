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

#ifndef _HIH8000_CUSTOMI2C_h
#define _HIH8000_CUSTOMI2C_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <I2C.h>


// Error codes
typedef enum
{
  // Errors categorized according to the functions that return them. Commented-out errors are still returned
  // by that function, but is already defined in the list.

  // Errors returned from the custom I2C library
  HIH8000_STATUS_OK                   = 0,   // No problemo
  HIH8000_STATUS_I2C_START            = 1,   // I2C timeout while waiting for successful completion of a Start bit
  HIH8000_STATUS_I2C_ACKNACK_MODE     = 2,   // I2C timeout while waiting for ACK/NACK while addressing slave in transmit mode (MT)
  HIH8000_STATUS_I2C_ACKNACK          = 3,   // I2C timeout while waiting for ACK/NACK while sending data to the slave
  HIH8000_STATUS_I2C_REPSTART         = 4,   // I2C timeout while waiting for successful completion of a Repeated Start
  HIH8000_STATUS_I2C_ACKNACK_RECMODE  = 5,   // I2C timeout while waiting for ACK/NACK while addressing slave in receiver mode (MR)
  HIH8000_STATUS_I2C_ACKNACK_REC      = 6,   // I2C timeout while waiting for ACK/NACK while receiving data from the slave
  HIH8000_STATUS_I2C_STOP             = 7,   // I2C timeout while waiting for successful completion of the Stop bit
  HIH8000_STATUS_I2C_OTHER            = 8,   // "See datasheet [of microcontroller chip] for exact meaning"
  HIH8000_STATUS_ADDRESS              = 9,   // The address for the device associated with this HIH8000_customI2C class has not been given yet
  HIH8000_STATUS_OTHER                = 10   // Any other errors
} HIH8000_STATUS;


class HIH8000_customI2C {
public:
  HIH8000_customI2C(uint8_t address);
  HIH8000_customI2C();
  ~HIH8000_customI2C();
  void setAddress(uint8_t newAddress);
  HIH8000_STATUS triggerMeasurement();
  HIH8000_STATUS fetchMeasurement();
  uint8_t getStatus();
  uint8_t getAddress();
  float getHumidity();
  float getTemperature();
  void applyHumidityCorrection(bool point1, double humidityStd, double humidityRaw);
  HIH8000_STATUS getI2CError(uint8_t errorNumber);

private:
  // This is used when fetching data. The documentation says 4 bytes will be transmitted from the sensor.
  static const uint8_t SENSOR_BYTES = 4;

  // Full range of sensor. 16382 = 2^14 - 2   (see the datasheet).
  static const uint16_t SENSOR_RANGE = 16382;

  // Used for masking binary forms of data
  static const uint16_t MASK_STATUS = 0xC000; // 1100000000000000 in binary
  static const uint16_t MASK_HUMIDITY = 0x3FFF; // 0011111111111111 in binary

  bool addressSet_ = 0;
  uint8_t address_ = 0;
  uint8_t status_ = 0;
  uint8_t endTransErr_ = 0;
  uint16_t humidityBuffer_ = 0;
  uint16_t temperatureBuffer_ = 0;
  float humidity_ = 0;
  float temperature_ = 0;

  // Two-point calibration
  float humidityStd1_ = 33.6;
  float humidityRaw1_ = 33.6;
  float humidityStd2_ = 75.6;
  float humidityRaw2_ = 75.6;
  float slopeAdjuster_ = 1;
  float offset_ = 0;
};

#endif

