/**********************************************************************
Library for HX711, originally written by bogde, modified by Soon Kiat Lau.
Modified features:
- Integrated multiplexer (muxManager_) for switching between devices.
- chamberID_ is the ID of each device
- Merged begin() and set_gain() into the constructor to force PD_SCK, DOUT,
and GAIN to be constants.
***********************************************************************/

#include "HX711_modified.h"

HX711::HX711(uint8_t gain, uint8_t trimLowSize, uint8_t trimHighSize)
  :
  gain_(gain),
  trimLowSize_(trimLowSize),
  trimHighSize_(trimHighSize)
{
  maxReadingsReached_ = false;
  readingCounter_ = 0;
  readingIndex_ = 0;
  prevAvailableTime_ = 0;
}

HX711::~HX711() {
}

void HX711::init() {
  SPI485.writeClockLow();
  long dummy;

  // Wait for the next available data
  while (checkDataAvailability()) {}
  while (!checkDataAvailability()){}
  read(dummy);
}

bool HX711::checkDataAvailability() {
  return SPI485.readMOSI() == LOW;
}

bool HX711::checkTiming(bool initialize)
{
  if (!initialize)
  {
    // Calculate how much time (us) has passed since the start of data availability.
    // This also covers cases when one or more conversion cycles were skipped over by
    // utilizing the modulo (%) operator, which means we don't necessarily require data
    // to be obtained from consecutive cycles; as long as data is obtained shortly after
    // start of data availability, we are good.
    // Note that all time quantities are in microseconds
    unsigned long excessTime = micros() - prevAvailableTime_;

    // Check if we are still within the same cycle which we have already extracted data out of.
    if (excessTime > TIME_CYCLE)
    {
      unsigned long excessTimeTrimmed = excessTime % TIME_CYCLE;  // microseconds

      // We create a no-read zone immediately after data is available and before next data is available.
      // The former helps prevent errors from mistimings
      // The latter helps prevent us from clocking data thru to the next data conversion cycle
      if (excessTimeTrimmed > DELAY_BEFORE_READ && excessTimeTrimmed < (TIME_CYCLE - DELAY_AFTER_READ))
      {// We are within the valid time zone for acquiring readings
        return true;
      }
      else
      {// Even though we are within a new data conversion, it is not within the zone for reading data.
        // Acquiring data in this zone is risky because a new conversion may begin before we finish
        // acquiring data, thus distorting the data. Therefore, just wait for the next data to be available.
        return false;
      }
    }
    else
    {// We have already acquired data in this data conversion cycle and are waiting for the next cycle
      return false;
    }
  }
  else
  {// Need to initialize the cycle by setting the dataAvailableStartTime_ right after a conversion is completed
    // Wait until the dat line is pulled HIGH (i.e. a conversion has started)
    while (checkDataAvailability()) {}
    // Now wait for the conversion to end
    while (!checkDataAvailability()) {}

    // Set dataAvailableStartTime_ and get data (by returning true, the calling function should now perform data retrieval)
    // Backtrack by one cycle period, since this is supposed to be the "previous" time at which data was available
    prevAvailableTime_ = micros() - TIME_CYCLE;

    return true;
  }
}

bool HX711::read(long& buffer) {
  unsigned long value = 0;
  uint8_t data[3] = { 0 };
  uint8_t filler = 0x00;

  // HX711 datasheet pg 5 suggests that the data is sampled at trailing edge of clock pulse
  SPI485.setSPIMode(false, true);

  // Read data thru 24 clock pulses (3 bytes)
  if (SPI485.receiveByte(data[2], MSBFIRST))
  {
    if (SPI485.receiveByte(data[1], MSBFIRST))
    {
      if (!SPI485.receiveByte(data[0], MSBFIRST))
      {
        // Failed to get data, but still send remaining pulses for next conversion
        // Send additional pulses to set GAIN. See code below for explanation on disabling/enabling SPI
        SPI485.disableSPI();
        setGain();
        delayMicroseconds(10);
        SPI485.enableSPI();

        return false;
      }
    }
    else
    {
      // Failed to get data, but still send remaining pulses for next conversion
      for (unsigned int i = 0; i < 8; i++)
      {
        SPI485.pulseClock();
      }

      // Send additional pulses to set GAIN. See code below for explanation on disabling/enabling SPI
      SPI485.disableSPI();
      setGain();
      delayMicroseconds(10);
      SPI485.enableSPI();

      return false;
    }
  }
  else
  {
    // Failed to get data, but still send remaining pulses for next conversion
    for (unsigned int i = 0; i < 16; i++)
    {
      SPI485.pulseClock();
    }

    // Send additional pulses to set GAIN. See code below for explanation on disabling/enabling SPI
    SPI485.disableSPI();
    setGain();
    delayMicroseconds(10);
    SPI485.enableSPI();

    return false;
  }

  // Set the channel and the gain factor for the next reading using additional pulses
  // First, disable SPI to prevent these pulses from affecting the SPIF flag
  SPI485.disableSPI();

  // Now, send the pulses
  setGain();

  // Replicate the most significant bit to pad out a 32-bit signed integer
  if (data[2] & 0x80) {
    filler = 0xFF;
  }
  else {
    filler = 0x00;
  }

  // Construct a 32-bit signed integer
  value = (static_cast<unsigned long>(filler) << 24
    | static_cast<unsigned long>(data[2]) << 16
    | static_cast<unsigned long>(data[1]) << 8
    | static_cast<unsigned long>(data[0]));

  buffer = static_cast<long>(value);

  // Re-enable SPI; do this after all the calculations so we get some time to ensure the additional clock pulses have completed
  SPI485.enableSPI();

  return true;
}

// The main function which should be called on every iteration in the Arduino Loop function
bool HX711::run(bool initialize)
{
  // Is it time to get next reading and is data available on the HX711?
  if (checkTiming(initialize))
  {
    if (checkDataAvailability())
    {
      return runDirect();
    }
  }
  else
  {
    return true;
  }
}

// Similar to run, but skips checks for timing and data availability. Useful if we need to do something
// in calling function after knowing data is ready for retrieval, but before actually retrieving it
bool HX711::runDirect()
{
  // Datasheet requires min of 0.1 us from DOUT going LOW to start of clock pulses
  delayMicroseconds(1);

  // Calculate the time at which the data was available during this cycle
  // Integer division removes the quotient part, so this is almost like floor() (for positive numbers)
  // The integer division is to ensure we catch all cycles that have been completed.
  uint8_t completedCycles = (micros() - prevAvailableTime_) / TIME_CYCLE;  // microseconds
  prevAvailableTime_ += completedCycles * TIME_CYCLE;  // microseconds

  // Get new raw reading and store it
  if (read(readings_[readingIndex_]))
  {
    // Update total readings stored
    if (!maxReadingsReached_)
    {
      if (readingCounter_ < maxReadings_ - 1)
      {// Increment reading counter
        readingCounter_++;
      }
      else
      {// The storage array is fully filled
        readingCounter_ = maxReadings_;
        maxReadingsReached_ = true;
      }
    }

    // Update where the next reading should be stored
    if (readingIndex_ < maxReadings_ - 1)
    {
      readingIndex_++;
    }
    else
    {// Start replacing entries at beginning of storage array
      readingIndex_ = 0;
    }

    return true;
  }
  else
  {
    return false;
  }
}

// Filter the raw readings by sorting them and removing tailing entries
double HX711::getFilteredReading()
{
  long readingsSum = 0;
  
  // Check if we have enough readings to perform the filtering
  if (readingCounter_ > (trimLowSize_ + trimHighSize_))
  {
    // Prepare the sorting array
    for (uint8_t i = 0; i < readingCounter_; i++)
    {
      sortedReadings_[i] = readings_[i];
    }
    
    // Sort the readings
    qsort(sortedReadings_, readingCounter_, sizeof(long), comparator);

    // Remove readings that are noise.
    // First, get the median
    long median;
    if (readingCounter_ % 2 > 0)
    {// readingCounter_ is even; get average of middle 2 entries
      median = (sortedReadings_[readingCounter_ / 2] + sortedReadings_[readingCounter_ / 2 - 1]) / 2;
    }
    else
    {
      median = sortedReadings_[(readingCounter_ - 1) / 2];
    }

    uint8_t validReadingsCount = 0;

    // Extract out readings which are identified as valid data
    for (uint8_t i = 0; i < readingCounter_; i++)
    {
      if (abs(sortedReadings_[i] - median) < BAD_THRESHOLD)
      {
        validReadings[validReadingsCount] = sortedReadings_[i];
        validReadingsCount++;
      }
    }

    // Check if we have enough valid readings for the removal of tailing readings
    if (validReadingsCount > trimLowSize_ + trimHighSize_)
    {
      // Sum over the valid readings, minus the tailing ones
      for (uint8_t i = trimLowSize_; i < validReadingsCount - trimHighSize_; i++)
      {
        readingsSum += validReadings[i];
      }

      // Average the readings, keeping in mind the reduced sample size due to removed tailing entries
      return readingsSum / (validReadingsCount - trimLowSize_ - trimHighSize_);
    }
    else
    {// Not enough readings
    // TODO: Return an error code??
      return 0;
    }
  }
  else
  {// Not enough readings
    // TODO: Return an error code??
    return 0;
  }
}

// Get the reading that has been filtered from the buffer of raw readings and reset the reading counter
bool HX711::calculateWeight(double& buffer)
{
  if (readingCounter_ > 0)
  {// At least one reading has been obtained
    // Apply offset and scaling to the filtered reading
    buffer = (getFilteredReading() - scaleOffset_) / scaleFactor_;

    return true;
  }
  else
  {// Not a single reading was obtained!
    return false;
  }
}

// Based on filtered reading, calculate offset, store it, and place new offset-ed reading into the return buffer
bool HX711::tare(double& buffer)
{
  if (readingCounter_ > 0)
  {// At least one reading has been obtained
   // Get the filtered reading
    buffer = getFilteredReading();

    // Make the current filtered reading as the offset
    scaleOffset_ = long(buffer);

    // Apply the offset to the current filtered reading
    buffer = (buffer - scaleOffset_) / scaleFactor_;

    return true;
  }
  else
  {// Not a single reading was obtained!
    return false;
  }
}

// Based on filtered reading, calculate scaling value, store it, and place new scaled reading into the return buffer
bool HX711::calibrate(double& buffer, double refWeight)
{
  // Sets the scale factor for the weighing scale (i.e. for a given reading, how does it translate into mass units).
  /* Implicitly assumes a linear relation between reading and mass units. Thus, might give erroneous readings
  * towards the end of the scale range where readings usually start to become nonlinear.
  *
  *  A measurement is calculated as follows ("get_units" function in the HX711 library by bogde):
  *
  *                  m = (r - o)/k
  *
  *  where   m is the actual weight of the sample              [g / kg / lbs / or any units for mass]
  *          r is the averaged reading given out by the HX711  [dimensionless]
  *          o is the offset determined during taring          [dimensionless]
  *          k is the scale factor                             [inverse of the units of the actual weight]
  *
  *  Therefore, k is determined by:
  *
  *                  k = (r - o)/m
  */
  if (readingCounter_ > 0)
  {// At least one reading has been obtained
   // Get the filtered reading and apply the offset
    double offsetReading = getFilteredReading() - scaleOffset_;

    // Calculate the scale based on the offset-ed reading and the actual weight
    scaleFactor_ = offsetReading / refWeight;

    // Now scale the offset reading with the newly-calculated scale factor
    buffer = offsetReading / scaleFactor_;

    return true;
  }
  else
  {// Not a single reading was obtained!
    return false;
  }
}

void HX711::setScaleFactor(double scale) {
  scaleFactor_ = scale;
}

double HX711::getScaleFactor() {
  return scaleFactor_;
}

void HX711::setScaleOffset(long offset) {
  scaleOffset_ = offset;
}

long HX711::getScaleOffset() {
  return scaleOffset_;
}

// Reset the readingCounter_ and maxReadingsExceeded_ flag; used when trying to restart a measurement cycle
void HX711::resetReadings() {
  maxReadingsReached_ = false;
  readingCounter_ = 0;
  readingIndex_ = 0;
}

void HX711::powerDown() {
  SPI485.writeClockLow();
  SPI485.writeClockHigh();
}

// See comment for power_down
void HX711::powerUp() {
  SPI485.writeClockLow();
}

void HX711::setGain()
{
  for (unsigned int i = 0; i < gain_; i++)
  {
    SPI485.pulseClock();
  }
}

// Function used by qsort() in getFilteredReading() to determine the order of the raw readings
int HX711::comparator(const void* p1, const void* p2)
{
  // Cast to long pointer, then dereference it to get actual reading
  if (*(long*)p1 < *(long*)p2)
  {
    return -1;
  }
  else
  {
    return 1; // Even if they are equal, consider one as bigger. Doesn't change the outcome anyway.
  }
}


