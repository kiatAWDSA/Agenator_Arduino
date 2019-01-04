/*  Arduino Library for the PCA9685 16-Channel PWM Driver Module.
Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>
Copyright (C) 2012 Kasper Skårhøj   <kasperskaarhoj@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Created by Kasper Skårhøj, August 3rd, 2012.
Forked by Vitska, June 18th, 2016.
Forked by NachtRaveVL, July 29th, 2016.

Modified by Soon Kiat Lau 2018:
- use a 3rd-party I2C library (Wayne Truchsess) to avoid the blocking issues
  encountered by the Arduino Wire library.

********************************************************************************************
********************************************************************************************
**  The PCA9685 has a max speed of approx 1.5 kHz, which means one cycle is about 700 us.
**  Fig 10 (p19) of datasheet shows that a change of output will only occur on THE NEXT CYCLE!!
**  Therefore, delays must be added between each command to ensure that at least one cycle
**  is completed to get the expected output. If the calling program moves on without
**  waiting for this delay, then the affected PCA9685 pin might still be displaying
**  the state from the previous/current cycle, instead of the new desired output.
**  The duration of the delay can be calculated by taking 1/f, where f is the frequency
**  the PCA9685 operates at (Min 20 Hz, Max 1526 Hz).
********************************************************************************************
********************************************************************************************

PCA9685-Arduino - Version 1.2.10
*/

#ifndef _PCA9685_CUSTOMI2C_h
#define _PCA9685_CUSTOMI2C_h

// Library Setup

// Uncomment this define to enable use of the software i2c library (min 4MHz+ processor required).
//#define PCA9685_ENABLE_SOFTWARE_I2C     1   // http://playground.arduino.cc/Main/SoftwareI2CLibrary

// Uncomment this define if wanting to exclude extended functionality from compilation.
#define PCA9685_EXCLUDE_EXT_FUNC        1

// Uncomment this define if wanting to exclude ServoEvaluator assistant from compilation.
#define PCA9685_EXCLUDE_SERVO_EVAL      1

// Uncomment this define to enable debug output.
//#define PCA9685_ENABLE_DEBUG_OUTPUT     1

// Servo Control Note
// Many 180 degree controlled digital servos run on a 20ms pulse width (50Hz update
// frequency) based duty cycle, and do not utilize the entire pulse width for their
// -90/+90 degree control. Typically, 2.5% of the 20ms pulse width (0.5ms) is considered
// -90 degrees, and 12.5% of the 20ms pulse width (2.5ms) is considered +90 degrees. This
// roughly translates to raw PCA9685 PWM values of 102 and 512 (out of the 4096 value
// range) for -90 to +90 degree control, but may need to be adjusted to fit your specific
// servo (e.g. some I've tested run ~130 to ~525 for their -90/+90 degree control). Also
// be aware that driving some servos past their -90/+90 degrees of movement can cause a
// little plastic limiter pin to break off and get stuck inside of the gearing, which
// could potentially cause the servo to become jammed. See the PCA9685_ServoEvaluator
// class to assist with calculating PWM values from Servo angle values.

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
#include <I2C.h>
#endif

#define PCA9685_MODE_INVRT          (byte)0x10  // Inverts polarity of channel output signal
#define PCA9685_MODE_OUTPUT_ONACK   (byte)0x08  // Channel update happens upon ACK (post-set) rather than on STOP (endTransmission)
#define PCA9685_MODE_OUTPUT_TPOLE   (byte)0x04  // Use a totem-pole (push-pull) style output, typical for boards using this chipset
#define PCA9685_MODE_OUTNE_HIGHZ    (byte)0x02  // For active low output enable, sets channel output to high-impedance state
#define PCA9685_MODE_OUTNE_LOW      (byte)0x01  // Similarly, sets channel output to high if in totem-pole mode, otherwise high-impedance state

#define PCA9685_MIN_CHANNEL         0
#define PCA9685_MAX_CHANNEL         15
#define PCA9685_CHANNEL_COUNT       16

// Error codes
typedef enum
{
  // Errors returned from the custom I2C library
  PCA9685_STATUS_OK                   = 0,  // No problemo
  PCA9685_STATUS_I2C_START            = 1,  // I2C timeout while waiting for successful completion of a Start bit
  PCA9685_STATUS_I2C_ACKNACK_MODE     = 2,  // I2C timeout while waiting for ACK/NACK while addressing slave in transmit mode (MT)
  PCA9685_STATUS_I2C_ACKNACK          = 3,  // I2C timeout while waiting for ACK/NACK while sending data to the slave
  PCA9685_STATUS_I2C_REPSTART         = 4,  // I2C timeout while waiting for successful completion of a Repeated Start
  PCA9685_STATUS_I2C_ACKNACK_RECMODE  = 5,  // I2C timeout while waiting for ACK/NACK while addressing slave in receiver mode (MR)
  PCA9685_STATUS_I2C_ACKNACK_REC      = 6,  // I2C timeout while waiting for ACK/NACK while receiving data from the slave
  PCA9685_STATUS_I2C_STOP             = 7,  // I2C timeout while waiting for successful completion of the Stop bit
  PCA9685_STATUS_I2C_OTHER            = 8,  // "See datasheet [of microcontroller chip] for exact meaning"
  PCA9685_STATUS_PROXY                = 9,  // This chip is set as a proxy device
  PCA9685_STATUS_CHANNEL              = 10, // The given PWM channel is out of range
  PCA9685_STATUS_OTHER                = 11  // Any other errors
} PCA9685_STATUS;

typedef enum {
  PCA9685_PhaseBalancer_None = -1,    // Disables phase balancing, all high phase areas start at beginning of cycle
  PCA9685_PhaseBalancer_Linear = 0,   // Balances all outputs linearly, 256 steps away from previous output
  PCA9685_PhaseBalancer_Weaved,       // Balances first few outputs better, steps away from previous shorten towards last output

  PCA9685_PhaseBalancer_Count
} PCA9685_PhaseBalancer;

class PCA9685 {
public:
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
  // May use a different Wire instance than Wire. Some chipsets, such as Due/Zero/etc.,
  // have a Wire1 class instance that uses the SDA1/SCL1 lines instead.
  // Supported i2c baud rates are 100kHz, 400kHz, and 1000kHz.
  PCA9685(I2C& i2cWire, PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_Linear);
#else
  // Minimum supported i2c baud rate is 100kHz, which means minimum supported processor
  // speed is 4MHz+ while running i2c standard mode. For 400kHz i2c baud rate, minimum
  // supported processor speed is 16MHz+ while running i2c fast mode.
  PCA9685(PCA9685_PhaseBalancer phaseBalancer = PCA9685_PhaseBalancer_Linear);
#endif

  // Should be called only once in setup(), before any init()'s, but after Wire.begin().
  // Only should be called once on any Wire instance to do a software reset, which
  // will affect all devices on that line. This helps when you're constantly rebuilding
  // and reuploading to ensure all the devices on that line are reset properly.
  void resetDevices();

  // Called in setup(). The i2c address here is the value of the A0, A1, A2, A3, A4 and
  // A5 pins ONLY, as the class takes care of its internal base address. i2cAddress
  // should be a value between 0 and 61, since only 62 boards can be addressed.
  // See pg 16 of datasheet for explanation for the mode variable
  // Since the address is defined as 1 A5 A4 A3 A2 A1 A0 (Notice 8th bit is missing because
  // I2C addresses are usually 7-bit, and 7th bit is fixed at 1 due to PCA9685), then the
  // addresses for PCA9685 begins at 64 (0100 0000) thru 127 (0111 1111).
  PCA9685_STATUS init(byte i2cAddress = 0, byte mode = PCA9685_MODE_OUTPUT_ONACK | PCA9685_MODE_OUTPUT_TPOLE);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
  // Called in setup(). Used when instance talks through to AllCall/Sub1-Sub3 instances
  // as a proxy object. Using this method will disable any method that performs a read
  // or conflicts certain states.
  void initAsProxyAddresser(byte i2cAddress = 0xE0);
#endif

  byte getI2CAddress();
  PCA9685_PhaseBalancer getPhaseBalancer();

  // Min: 24Hz, Max: 1526Hz, Default: 200Hz (resolution widens as Hz goes higher)
  PCA9685_STATUS setPWMFrequency(float pwmFrequency);

  // Turns channel either full on or full off
  PCA9685_STATUS setChannelOn(int channel);
  PCA9685_STATUS setChannelOff(int channel);

  // PWM amounts 0 - 4096, 0 full off, 4096 full on
  PCA9685_STATUS setChannelPWM(int channel, uint16_t pwmAmount);
  PCA9685_STATUS setChannelsPWM(int begChannel, int numChannels, const uint16_t *pwmAmounts);

#ifndef PCA9685_EXCLUDE_EXT_FUNC
  // Sets all channels, but won't distribute phases
  void setAllChannelsPWM(uint16_t pwmAmount);

  // Returns PWM amounts 0 - 4096, 0 full off, 4096 full on
  uint16_t getChannelPWM(int channel);

  // Enables multiple talk-through paths via i2c bus (lsb/bit0 must stay 0)
  // To use, create a new class instance using initAsSubAddressed() with said address
  void enableAllCallAddress(byte i2cAddress = 0xE0);
  void enableSub1Address(byte i2cAddress = 0xE2);
  void enableSub2Address(byte i2cAddress = 0xE4);
  void enableSub3Address(byte i2cAddress = 0xE8);
  void disableAllCallAddress();
  void disableSub1Address();
  void disableSub2Address();
  void disableSub3Address();

  // Allows external clock line to be utilized (once enabled cannot be disabled)
  void enableExtClockLine();
#endif

  byte getLastI2CError();

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
  void printModuleInfo();
  void checkForErrors();
#endif

private:
#ifndef PCA9685_ENABLE_SOFTWARE_I2C
  I2C *_i2cWire;          // Wire class instance to use
#endif
  byte _i2cAddress;           // Module's i2c address
  PCA9685_PhaseBalancer _phaseBalancer; // Phase balancer scheme to distribute load
  bool _isProxyAddresser;     // Instance is a proxy for sub addressing (disables certain functionality)
  byte _lastI2CError;         // Last i2c error

  void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);

  byte getRegAddress(int channel);
  PCA9685_STATUS writeChannelPWM(byte regAddress, uint16_t phaseBegin, uint16_t phaseEnd);

  PCA9685_STATUS writeRegister(byte regAddress, byte value);
  PCA9685_STATUS readRegister(byte regAddress, byte& readBuffer);

#ifdef PCA9685_ENABLE_SOFTWARE_I2C
  uint8_t _readBytes;
#endif
  PCA9685_STATUS i2cWire_read(uint8_t bytesCount);
  PCA9685_STATUS i2cWire_write(uint8_t regAddress);
  PCA9685_STATUS i2cWire_write(uint8_t regAddress, uint8_t value);
  PCA9685_STATUS i2cWire_write(uint8_t regAddress, uint8_t value[], uint8_t arrayLen);
  PCA9685_STATUS getI2CError();
};

#ifndef PCA9685_EXCLUDE_SERVO_EVAL

// Class to assist with calculating Servo PWM values from angle values
class PCA9685_ServoEvaluator {
public:
  // Uses a linear interpolation method to quickly compute PWM output value. Uses
  // default values of 2.5% and 12.5% of phase length for -90/+90.
  PCA9685_ServoEvaluator(uint16_t n90PWMAmount = 102, uint16_t p90PWMAmount = 512);

  // Uses a cubic spline to interpolate due to an offsetted zero angle that isn't
  // exactly between -90/+90. This takes more time to compute, but gives a more
  // accurate PWM output value along the entire range.
  PCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount);

  ~PCA9685_ServoEvaluator();

  // Returns the PWM value to use given the angle (-90 to +90)
  uint16_t pwmForAngle(float angle);

private:
  float *_coeff;      // a,b,c,d coefficient values
  bool _isCSpline;    // Cubic spline tracking, for _coeff length
};

#endif

#endif 
