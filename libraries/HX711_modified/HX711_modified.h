/**********************************************************************
Library for HX711, originally written by Bogdan Necula, modified by Soon Kiat Lau.
Modified features:
- Merged begin() and set_gain() into the constructor to force PD_SCK, DOUT,
  and GAIN to be constants.
- Integrated with RS485SPI library to allow communication over RS-485 libraries.
  Also added SPI timeout reporting ability to all reading functions.
- Changed the method of obtaining readings: instead of blocking everything while
  waiting for new data conversions, use timing checks to see if it is time to
  grab data.
- Added functionality to filter data. Removes certain amount of data points from
  lower and upper limits of the sorted raw data before averaging the collecte data.

  ***THIS LIBRARY REQUIRES THE CLOCK LINE TO BE PULLED LOW ON IDLE***

  Some HX711 breakout boards (e.g. the red one with a metal shield) have a
  pull-up resistor on the PD_SCK line. Therefore, if the clock line is at
  high impedance state, the HX711 will enter sleep mode due to the pull-up
  resistor (datasheet pg 5). It is necessary to modify these boards to
  use a pull-down resistor on the clock line instead, otherwise the
  library will not work properly. This is because the library relies on
  the HX711 constantly converting data.

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

#ifndef _HX711_MODIFIED_h
#define _HX711_MODIFIED_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <RS485SPI.h>

class HX711
{
public:
  // Gain factors
  // channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
  // depending on the parameter, the channel is also set to either A or B
  // These numbers below are number of additional pulses to be added to the end of a
  // clock pulse train. See pg 4 of datasheet.
  static const uint8_t GAIN_128 = 1;  // channel A, gain factor 128
  static const uint8_t GAIN_64 = 3;    // channel A, gain factor 64
  static const uint8_t GAIN_32 = 2;    // channel B, gain factor 32

  // Timing variables
  static const unsigned long TIME_CYCLE = 12500;      // Time (us) between each sample; for 80 sample/s, this will be 12.5 ms = 12,500 us.
  static const unsigned long DELAY_BEFORE_READ = 2000; // Time (us) delay before a reading can be taken after data is available.
  static const unsigned long DELAY_AFTER_READ = 2000; // Time (us) zone before end of cycle where no data is allowed to be taken. This prevents clocking thru the next conversion.

  // Difference from the median of raw readings to be considered as bad data
  static const long BAD_THRESHOLD = 100000;

  // define clock and data pin, channel, and gain factor
  // channel selection is made by passing the appropriate gain: 128 or 64 for channel A, 32 for channel B
  // gain: 128 or 64 for channel A; channel B works with 32 gain factor only
  // readInterval: Should be at least 1000/80 ~= 13 ms if HX711 is set to 80 samples/s
  // trimSize: Amount of readings at the lower and upper end of collected readings to be discarded. Should be
  //           less than maxReadings/2.
  HX711(uint8_t gain, uint8_t trimLowSize, uint8_t trimHighSize);

  virtual ~HX711();

  void init();

  // check if HX711 has data available for retrieval
  // From the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
  // input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
  bool checkDataAvailability();

  // Check if it is time to get data
  // Set initialize to true to initialize/reset the time at which data is available for retrieval
  // Probably want to reset the time after long periods of time to ensure the timing is still correct
  bool checkTiming(bool initialize);

  // waits for the chip to be ready and returns a reading
  bool read(long& buffer);

  // The main function which should be called on every iteration in the Arduino Loop function
  bool run(bool initialize);

  // Similar to run, but skips checkReadiness. Useful if we need to do something in calling function
  // after knowing data is ready for retrieval, but before actually retrieving it
  bool runDirect();

  // Filter the raw readings by sorting them and removing tailing entries
  double getFilteredReading();

  // Apply offset and scaling to the filtered reading to get the actual weight
  bool calculateWeight(double& buffer);

  // Based on filtered reading, calculate offset, store it, and place new offset-ed reading into the return buffer
  bool tare(double& buffer);

  // Based on filtered reading, calculate scaling value, store it, and place new scaled reading into the return buffer
  bool calibrate(double& buffer, double refWeight);

  // set the scale factor; this value is used to convert the raw data to "human readable" data (measure units)
  void setScaleFactor(double scale = 1.f);

  // get the current scale factor
  double getScaleFactor();

  // set scale offset, the value that's subtracted from the actual reading (tare weight)
  void setScaleOffset(long offset = 0);

  // get the current scale offset
  long getScaleOffset();

  // Resets the readCounter and index; used when trying to restart a measurement cycle
  void resetReadings();

  // puts the chip into power down mode
  void powerDown();

  // wakes up the chip after power down mode
  void powerUp();


private:
  // Max number of readings to be stored. A number that's too small will result in older readings
  // being quickly overwritten by newer ones.
  // Also, this should not be too large (<1,000) because the filtering process sums up all the stored readings before dividing by total to get average;
  // if the summing process exceeds the size of long (+-2,147,483,647), unexpected results would occur.
  static const uint8_t maxReadings_ = 21;

  // The amount of data points to be removed at the lower and upper ends of the sorted raw readings array during the filtering process.
  const uint8_t trimLowSize_;
  const uint8_t trimHighSize_;

  // Resolution of the ADC
  const uint8_t gain_;

  // Settings for converting readings to actual weight units
  double scaleFactor_ = 1;	  // used to return weight in grams, kg, ounces, whatever
  long scaleOffset_ = 0;	    // used for tare weight

  // Variables for obtaining readings
  bool maxReadingsReached_ = false;     // Flag checking if the counter exceeded maxReadings.
  uint8_t readingCounter_;              // Tracks how many readings have been acquired
  uint8_t readingIndex_;                // Tracks the location in readings array for storing new readings
  long readings_[maxReadings_];         // Storage for raw HX711 readings
  long sortedReadings_[maxReadings_];   // Copy of readings_ that will be quicksorted
  long validReadings[maxReadings_];     // Portion of sortedReadings_ that are considered to be close enough to the median to be valid readings
  unsigned long prevAvailableTime_;     // Time (us) at which the data was available in the previous cycle

  // set the GAIN for the next conversion
  void setGain();

  // Function used by qsort() in getFilteredReading() to determine the order of the raw readings
  static int comparator(const void* p1, const void* p2);
};

#endif

