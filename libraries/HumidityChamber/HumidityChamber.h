/********************************************************************** 
  Control humidity and air flow in an enclosed chamber for 
  beef aging project.

  --------------
  IMPORTANT NOTE
  --------------
  Devices that utilize the CLK and DAT lines (HX711, FS5, and fan tacho)
  must not be interacted with until switchConnect() has been called. After
  interacting with these devices, switchDisconnect() must be called to
  disconnect the CLK and DAT lines from this chamber. This routine is
  unfortunately prone to programmer error. One solution is to integrate
  these calls into the device class methods, but that will convolute their
  code...
  
  
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

// TODO: The triggering commands to LTC2452 are not working as expected.
// The functionality is commented out, although the variables still exist
// and are required in the constructor (tVelocityTrigger).
// TODO: No discrimination for the types of timeouts encountered for HX711.
// The first type is the HX711 chip itself, not pulling the DAT line to low.
// The second is the SPI (RS-422) waiting for data for too long.

#ifndef HumidityChamber_H
#define HumidityChamber_H
#include "Arduino.h"
#include <RS485SPI.h>
#include <HIH8000_customI2C.h>
#include <HX711_modified.h>
#include <PID_modified.h>
#include <FS5.h>
#include <TachoSensor.h>
#include <PCA9685_customI2C.h>

// Uncomment only if calibrating FS5 sensor
// Do the same for the FS5.h file.
// #define CALIBRATE_VELOCITY 1


// List of devices that serves two-fold purpose:
// 1) 0 to 15 are the actual mapping to PWM pins of PCA9685.
// 2) 20 and above are devices not directly connected to outputs of PCA9685
// 3) 255 is a special case which is the entire chamber itself (for checking connection)
// During error handling, this list is used to refer to the device that created the error.
typedef enum
{
  DEVICE_CONNCHECK  = 0,  // Not connected to anything; only used for checking chamber connection
  DEVICE_HUMID      = 1,  // MOSFET for humidifier
  DEVICE_SOL_WET    = 2,  // MOSFET for solenoid valve controlling air flow to humidifier
  DEVICE_FAN2_PWM   = 3,  // PWM input to fan 2
  DEVICE_FAN_REG    = 4,  // Pin on SN74HC590A to display current pulse count on output pins
  DEVICE_FAN_CLR    = 5,  // Pin on SN74HC590A to clear pulse counts
  DEVICE_FAN_SHIFT  = 6,  // Pin on SN74HC165 to shift the state of input pins into internal register
  DEVICE_FAN1_PWM   = 7,  // PWM input to fan 1
  DEVICE_SOL_DRY    = 8,  // MOSFET for solenoid valve controlling air flow to dry columns
  DEVICE_PUMP       = 9,  // Relay for air pump
  DEVICE_FAN2_SELX  = 10, // Active LOW for selecting fan 2 on SN74LV125A controlling CLK/DAT lines and SN74LV244A controlling the fan reg, shift, and clr pins
  DEVICE_FAN1_SELX  = 11, // Active LOW for selecting fan 1 on SN74LV125A controlling CLK/DAT lines and SN74LV244A controlling the fan reg, shift, and clr pins
  DEVICE_FS5_SELX   = 12, // Active LOW for selecting FS5 on SN74LV125A controlling CLK/DAT lines
  DEVICE_HX711_SELX = 13, // Active LOW for selecting HX711 on SN74LV125A controlling CLK/DAT lines
  DEVICE_CLKDAT_ENX = 14, // Active LOW for enabling the CLK and DAT lines to this chamber
  DEVICE_CLKDAT_EN  = 15, // Active HIGH for enabling the CLK and DAT lines to this chamber
  DEVICE_HX711      = 18, // HX711
  DEVICE_FS5        = 19, // FS5
  DEVICE_HIH8000    = 20, // Humidity and temperature sensor
  DEVICE_FAN1       = 21, // Fan 1
  DEVICE_FAN2       = 22, // Fan 2
  DEVICE_CHAMBER    = 255 // The chamber itself
} Device;

// Identifiers for the fans
typedef enum
{
  FAN1 = 1,
  FAN2 = 2
} FAN;

// Devices that need to be switched between for the CLK and DAT lines
typedef enum
{
  CLKDATSWITCH_FAN1   = 1,
  CLKDATSWITCH_FS5    = 2,
  CLKDATSWITCH_HX711  = 3,
  CLKDATSWITCH_FAN2   = 4
} CLKDATSWITCH;


class HumidityChamber  {
  public:
    // Characters for communicating with the Agenator C# program on computer
    // When initializing char arrays from strings, need one extra spot for the null character
    static const char SERIAL_SEND_START               = '^';
    static const char SERIAL_SEND_CHAMBERCONN         = 'c';
      static const char SERIAL_SEND_CHAMBERCONN_CONN  = 'y';
      static const char SERIAL_SEND_CHAMBERCONN_DISC  = 'n';
    static const char SERIAL_SEND_SCALEOFFSET         = 'o';
    static const char SERIAL_SEND_SCALEFACTOR         = 'k';
    static const char SERIAL_SEND_ACQUISITION         = 'a';
    static const char SERIAL_SEND_READING_ERR         = 'e';
    static const char SERIAL_SEND_ERROR               = 'e';
    static const char SERIAL_SEND_SEPARATOR           = '|';
    static const char SERIAL_SEND_END                 = '@';
    static const char SERIAL_SEND_EOL                 = '\n';
  
  
    // Functions affecting the object instance
    HumidityChamber(uint8_t chamberID,
                    uint8_t PCA9685Address,
                    uint8_t humidityAddress,
                    I2C& I2CManager,
                    long scaleOffset,
                    double scaleFactor,
                    uint16_t tHumidityTrigger,
                    uint16_t tHumidityFetch,
                    uint16_t tFan1SpeedGet,
                    uint16_t tFan2SpeedGet,
                    uint16_t tWeightFetch,
                    uint16_t tVelocityFetch,
                    uint16_t tControlHumidity,
                    uint16_t tControlFanSpeed,
                    uint16_t tCycle,
                    uint16_t tHumidifierOnDuration,
                    uint16_t tHumidifierOffDuration,
                    double pumpIdleZone,
                    // Parameters for air velocity model
                    float mean,
                    float stdDev,
                    float x0,
                    float x1,
                    float x2,
                    float x3,
                    // Inputs to the PID
                    double humidityKp,
                    double humidityKi,
                    double humidityKd,
                    double fanSpeedKp,
                    double fanSpeedKi,
                    double fanSpeedKd);
    ~HumidityChamber();

    // Initialization functions
    void init();
    void initScale();
    bool checkConnection(bool firstCheck, bool sendSerial, bool forceSend);
    
    /* The chamber can be in either one of 3 modes:
     *    Idle mode (0):          Do nothing
     *    Acquisition mode (1):   Acquires data and sends to computer
     *    Control mode (2):       Acquires data, perform PID control, and sends to computer
    */
    bool startIdleMode(bool sendSerial);
    bool startAcquisitionMode(unsigned long sendDataInterval, unsigned long curTime, bool sendSerial);
    bool startControlMode(double humiditySetpoint, int fanSpeedSetpoint, unsigned long curTime);
    bool changeSetpoint(double humiditySetpoint, int fanSpeedSetpoint);
    void sendReading(unsigned long curTime);
    void resetPID(bool affectHumidity);
    
    // Main function to be called in the loop{} of Arduino sketch
    void checkTiming(unsigned long curTime, bool sendSerial);

    // Calibrate humidity sensor
    void applyHumidityCorrection(bool point1, float humidityRaw, float humidityStd);
    
    // Functions for weighing scale
    void calibrateScale(unsigned long curTime, float refWeight);
    void tareScale(unsigned long curTime);
    void setScaleOffset(long scaleOffset);
    void setScaleFactor(double scaleFactor);
    void sleepScale(bool sendSerial);
    void wakeScale(bool sendSerial);
    
    
    
  
  private:
    // Constants affecting system behavior
    const double PWM_FAN_MIN                  = 0;
    const double PWM_FAN_MAX                  = 4096;
    
    // Thresholds for identifying abnormal readings
    const double ABNORMAL_THRESHOLD_HUMIDITY  = 10; // Maximum deviation (%) from previous RH value before being considered an abnormal reading
    const double ABNORMAL_THRESHOLD_VELOCITY  = 5;  // Maximum deviation (m/s) from previous RH value before being considered an abnormal reading
    const uint8_t maxAbnormalReadings         = 3;  // Number of consecutive abnormal readings before accepting the abnormal readings as the actual reading
    uint8_t abnormalHumidityCount_            = 0;  // Counter for number of consecutive abnormal RH readings
    uint8_t abnormalVelocityCount_            = 0;  // Counter for number of consecutive abnormal velocity readings

    // Error codes for HIH8000 with custom I2C library
    static const uint8_t ERR_HIH8000_I2C_START              = 1;  // I2C timeout while waiting for successful completion of a Start bit
    static const uint8_t ERR_HIH8000_I2C_ACKNACK_MODE       = 2;  // I2C timeout while waiting for ACK/NACK while addressing slave in transmit mode (MT)
    static const uint8_t ERR_HIH8000_I2C_ACKNACK            = 3;  // I2C timeout while waiting for ACK/NACK while sending data to the slave
    static const uint8_t ERR_HIH8000_I2C_REPSTART           = 4;  // I2C timeout while waiting for successful completion of a Repeated Start
    static const uint8_t ERR_HIH8000_I2C_ACKNACK_RECMODE    = 5;  // I2C timeout while waiting for ACK/NACK while addressing slave in receiver mode (MR)
    static const uint8_t ERR_HIH8000_I2C_ACKNACK_REC        = 6;  // I2C timeout while waiting for ACK/NACK while receiving data from the slave
    static const uint8_t ERR_HIH8000_I2C_STOP               = 7;  // I2C timeout while waiting for successful completion of the Stop bit
    static const uint8_t ERR_HIH8000_I2C_OTHER              = 8;  // "See datasheet [of microcontroller chip] for exact meaning"
    static const uint8_t ERR_HIH8000_ADDRESS                = 9;  // The address for the device associated with this HIH8000_customI2C class has not been given yet
    static const uint8_t ERR_HIH8000_OTHER                  = 10; // Any other errors

    // Error codes for Tachometer with custom I2C library
    static const uint8_t ERR_TACHO_I2C_START                = 11; // I2C timeout while waiting for successful completion of a Start bit
    static const uint8_t ERR_TACHO_I2C_ACKNACK_MODE         = 12; // I2C timeout while waiting for ACK/NACK while addressing slave in transmit mode (MT)
    static const uint8_t ERR_TACHO_I2C_ACKNACK              = 13; // I2C timeout while waiting for ACK/NACK while sending data to the slave
    static const uint8_t ERR_TACHO_I2C_REPSTART             = 14; // I2C timeout while waiting for successful completion of a Repeated Start
    static const uint8_t ERR_TACHO_I2C_ACKNACK_RECMODE      = 15; // I2C timeout while waiting for ACK/NACK while addressing slave in receiver mode (MR)
    static const uint8_t ERR_TACHO_I2C_ACKNACK_REC          = 16; // I2C timeout while waiting for ACK/NACK while receiving data from the slave
    static const uint8_t ERR_TACHO_I2C_STOP                 = 17; // I2C timeout while waiting for successful completion of the Stop bit
    static const uint8_t ERR_TACHO_I2C_OTHER                = 18; // "See datasheet [of microcontroller chip] for exact meaning"
    static const uint8_t ERR_TACHO_SPI_TIMEOUT              = 19; // Timeout while waiting to receive a byte from SPI
    static const uint8_t ERR_TACHO_OTHER                    = 20; // Any other errors

    // Error codes for other non-PCA9685 devices
    static const uint8_t ERR_FS5_SPI_TIMEOUT                = 21; // Timeout while waiting to receive a byte from SPI
    static const uint8_t ERR_HX711_SPI_TIMEOUT              = 22; // Timeout while waiting to receive a byte from SPI

    // More SPI-related errors
    static const uint8_t ERR_SPI_CONN_TIMEOUT               = 23; // Timeout while waiting for the SPI lines (i.e. PCA9685) to be activated

    // Problems with DAQ
    static const uint8_t ERR_ABNORMAL_RH                    = 24; // Obtained an RH reading that is abnormally different from the previous RH reading
    static const uint8_t ERR_ABNORMAL_VEL                   = 25; // Obtained a velocity reading that is abnormally different from the previous velocity reading
    static const uint8_t ERR_ABNORMAL_WEIGHT                = 26; // No weight readings were obtained during a measurement cycle
    static const uint8_t ERR_ABNORMAL_WEIGHT_TARE           = 27; // No weight readings were obtained for taring the weighing scale during a measurement cycle
    static const uint8_t ERR_ABNORMAL_WEIGHT_CAL            = 28; // No weight readings were obtained for calibrating the weighing scale during a measurement cycle

    // Error codes for PCA9685 with custom I2C library.
    static const uint8_t ERR_PCA9685_I2C_START              = 1; // I2C timeout while waiting for successful completion of a Start bit
    static const uint8_t ERR_PCA9685_I2C_ACKNACK_MODE       = 2; // I2C timeout while waiting for ACK/NACK while addressing slave in transmit mode (MT)
    static const uint8_t ERR_PCA9685_I2C_ACKNACK            = 3; // I2C timeout while waiting for ACK/NACK while sending data to the slave
    static const uint8_t ERR_PCA9685_I2C_REPSTART           = 4; // I2C timeout while waiting for successful completion of a Repeated Start
    static const uint8_t ERR_PCA9685_I2C_ACKNACK_RECMODE    = 5; // I2C timeout while waiting for ACK/NACK while addressing slave in receiver mode (MR)
    static const uint8_t ERR_PCA9685_I2C_ACKNACK_REC        = 6; // I2C timeout while waiting for ACK/NACK while receiving data from the slave
    static const uint8_t ERR_PCA9685_I2C_STOP               = 7; // I2C timeout while waiting for successful completion of the Stop bit
    static const uint8_t ERR_PCA9685_I2C_OTHER              = 8; // "See datasheet [of microcontroller chip] for exact meaning"
    static const uint8_t ERR_PCA9685_PROXY                  = 9; // This chip is set as a proxy device
    static const uint8_t ERR_PCA9685_CHANNEL                = 10; // The given PWM channel is out of range
    static const uint8_t ERR_PCA9685_OTHER                  = 11; // Any other errors

    // Constants for fan PWM amounts
    static const uint16_t FAN_PWM_ON  = 0x1000;
    static const uint16_t FAN_PWM_OFF = 0x0000;

    // Retries for failed communication
    static const uint8_t RETRIES_MAX = 3;

    // Decimal places for scale factor when sending to computer
    const uint8_t scaleFactorDecimalPlaces = 3;
    
    // ID and addresses
    const uint8_t chamberID_;
    const uint8_t PCA9685Address_;

    // PCA9685 timing
    // The PCA9685 has a max speed of approx 1.5 kHz, which means one cycle is about 700 us.
    // Fig 10 (p19) of datasheet shows that a change of output will only occur on THE NEXT CYCLE!!
    // Therefore, we need to add delays between each command to ensure that at least one cycle is completed
    // to give the appropriate signal to the pins
    // The duration of the delay can be calculated by taking 1/f, where f is the frequency
    // the PCA9685 operates at (Min 20 Hz, Max 1526 Hz).
    const uint16_t PCA9685CycleTimeDelay_ = 700;

    // Chamber connection
    bool connected_;
    bool chamberInitialized_;
    bool scaleInitialized_;
    
    // Chamber mode tracker
    uint8_t chamberMode_;
    
    // Time intervals (ms)
    const uint16_t tHumidityTrigger_;
    const uint16_t tHumidityFetch_;
    const uint16_t tFan1SpeedGet_;
    const uint16_t tFan2SpeedGet_;
    const uint16_t tWeightFetch_;
    const uint16_t tVelocityFetch_;
    const uint16_t tControlHumidity_;
    const uint16_t tControlFanSpeed_;
    const uint16_t tCycle_;
    const uint16_t tHumidifierOnDuration_;
    const uint16_t tHumidifierOffDuration_;
    uint16_t tPumpOn_;	// The off duration is tCycle_ - tPumpOn_
    
    // Humidity control
    const double PUMP_IDLE_ZONE;
    bool humidityControlIdle_;
    bool humidifierOn_;
    bool wetSolenoidOn_;
    bool drySolenoidOn_;
    bool pumpOn_;
    uint8_t pumpDutyCycle;
    unsigned long tHumidifierTurnedOn_;
    unsigned long tHumidifierTurnedOff_;
    unsigned long tPumpTurnedOn_;
    double humidityControlOutput_;
    double humiditySetpoint_;
    double humidityKp_;
    double humidityKi_;
    double humidityKd_;
    
    // Fan speed control
    double fan1SpeedControlOutput_;
    double fan2SpeedControlOutput_;
    double fanSpeedSetpoint_;
    double fanSpeedKp_;
    double fanSpeedKi_;
    double fanSpeedKd_;
    
    // Timing
    bool cycleStart_;
    bool humidityTriggered_;
    bool humidityFetched_;
    bool fan1SpeedGotten_;
    bool fan2SpeedGotten_;
    bool velocityFetched_;
    bool weightFetched_;
    bool humidityControlled_;
    bool fanSpeedControlled_;
    unsigned long cycleStartTime_;
    unsigned long prevDataSentTime_;
    unsigned long sendDataInterval_;

    // Vars for HX711
    bool calibratingScale_;
    bool taringScale_;
    static const uint8_t weightTrimLowSize_ = 3;  // Discard 3 readings from lower end of raw readings
    static const uint8_t weightTrimHighSize_ = 3;  // Discard 3 readings from upper end of raw readings
    double refWeight_;

    // Vars for fan speed
    unsigned long fan1ReadTimeStart_;
    unsigned long fan2ReadTimeStart_;
    
    // Buffer for readings
    double humidity_  = 0; // Accessed and modified directly by the PID class
    double weight_    = 0;
    double velocity_  = 0; // Accessed and modified directly by the PID class
    double fan1Speed_ = 0;
    double fan2Speed_ = 0;

    // Vars for handling new and abnormal readings
    bool firstHumidity_;
    double newHumidity_ = 0;
    bool firstVelocity_;
    double newVelocity_ = 0;
    
    // Serial communication
    char humidityBuffer_[6];      // XXX.X  [%]
    char temperatureBuffer_[6];   // XXX.X  [°C]
    char weightBuffer_[6];        // XXXXX  [g]  (no decimal)
    char velocityBuffer_[6];      // XXX.X  [m/s]
    char fan1SpeedBuffer_[7];     // XXXX.X  [rpm]
    char fan2SpeedBuffer_[7];     // XXXX.X  [rpm]
    char errDeviceBuffer_[4];     // XXX
    char errCodeBuffer_[4];       // XXX
    
    // Error buffers
    HIH8000_STATUS humidityErrBuffer_;
    PCA9685_STATUS PCA9685ErrBuffer_;
    TACHO_STATUS tachoErrBuffer_;
    
    // Internal classes
    HIH8000_customI2C humiditySensor_;
    HX711 scale_;
    FS5 vSensor_;
    TachoSensor fanSpeed_;
    PID humidityPID_;
    PID fan1SpeedPID_;
    PID fan2SpeedPID_;
    PCA9685 PCA9685Manager_;
    
    // Send data
    void printData(bool sendSerial);
    
    // Internal functions
    // Functions controlling the environment in the chamber
    void controlHumidity(unsigned long curTime, bool sendSerial);
    void controlFanSpeed(unsigned long curTime, bool sendSerial);
    bool sleepPumpAndSolenoids(bool sendSerial);
    void setFanPWM(FAN fan, uint16_t pwmAmount, bool sendSerial);
    
    // Functions for obtaining measurements
    void triggerHumidity(bool sendSerial);
    void triggerVelocity(bool sendSerial);
    void fetchHumidityAndTemperature(bool sendSerial);
    void fetchWeight(bool sendSerial);
    void fetchVelocity(bool sendSerial);
    void getFanSpeed(FAN fan, bool sendSerial);
    
    // Identifies errors and prepares the serial write buffers appropriately
    //void processHIHSensorErr(HIH8000_STATUS errCode);
    //void processPCA9685Err(PCA9685_STATUS errCode);

    // Interfaces for controlling states of devices
    bool setDeviceOn(Device device, bool sendSerial);
    bool setDeviceOff(Device device, bool sendSerial);
    bool setDevicePWM(Device device, uint16_t pwmAmount, bool sendSerial);
    bool connectCLKDATDevice(CLKDATSWITCH device, bool sendSerial);
    bool disconnectCLKDATDevice(bool sendSerial);

    // Error handling
    uint8_t getPCA9685ErrorCode(Device device, PCA9685_STATUS rawErrorCode); // PCA9685
    uint8_t getHIH8000ErrorCode(HIH8000_STATUS rawErrorCode); // HIH8000
    uint8_t getTachoErrorCode(TACHO_STATUS rawErrorCode);   // Tacho
    void sendError(Device device, uint8_t errorCode, bool sendSerial);

    // Send update on chamber connection status to computer program
    void sendConnectionStatus(bool connected, bool sendSerial);
};

#endif  // HumidityChamber_H