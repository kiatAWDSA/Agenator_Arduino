#include "HumidityChamber.h"

HumidityChamber::HumidityChamber( uint8_t chamberID,
                                  uint8_t PCA9685Address,
                                  uint8_t humidityAddress,
                                  I2C& I2CManager,
                                  long scaleOffset,
                                  double scaleFactor,
                                  // Timing constants
                                  uint16_t tHumidityTrigger,
                                  uint16_t tHumidityFetch,
                                  uint16_t tFan1SpeedGet,
                                  uint16_t tFan2SpeedGet,
                                  uint16_t tWeightFetch,
                                  uint16_t tVelocityFetch,
                                  uint16_t tControlHumidity,
                                  uint16_t tControlVelocity,
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
                                  double velocityKp,
                                  double velocityKi,
                                  double velocityKd)
                                  :
                                  chamberID_(chamberID),
                                  PCA9685Address_(PCA9685Address),
                                  PCA9685Manager_(I2CManager, PCA9685_PhaseBalancer_Linear),
                                  scale_(HX711::GAIN_128, weightTrimLowSize_, weightTrimHighSize_),
                                  vSensor_(mean, stdDev, x0, x1, x2, x3),
                                  fanSpeed_(PCA9685Manager_, DEVICE_FAN_REG, DEVICE_FAN_SHIFT, DEVICE_FAN_CLR, MSBFIRST),
                                  tHumidityTrigger_(tHumidityTrigger),
                                  tHumidityFetch_(tHumidityFetch),
                                  tFan1SpeedGet_(tFan1SpeedGet),
                                  tFan2SpeedGet_(tFan2SpeedGet),
                                  tVelocityFetch_(tVelocityFetch),
                                  tWeightFetch_(tWeightFetch),
                                  tControlHumidity_(tControlHumidity),
                                  tControlFanSpeed_(tControlVelocity),
                                  tCycle_(tCycle),
                                  tHumidifierOnDuration_(tHumidifierOnDuration),
                                  tHumidifierOffDuration_(tHumidifierOffDuration),
                                  PUMP_IDLE_ZONE(pumpIdleZone),
                                  humidityKp_(humidityKp),
                                  humidityKi_(humidityKi),
                                  humidityKd_(humidityKd),
                                  fanSpeedKp_(velocityKp),
                                  fanSpeedKi_(velocityKi),
                                  fanSpeedKd_(velocityKd),
                                  humiditySensor_(humidityAddress),
                                  humidityPID_(&humidity_, &humidityControlOutput_, &humiditySetpoint_, humidityKp_, humidityKi_, humidityKd_, millis(), P_ON_E, DIRECT),
                                  fan1SpeedPID_(&fan1Speed_, &fan1SpeedControlOutput_, &fanSpeedSetpoint_, fanSpeedKp_, fanSpeedKi_, fanSpeedKd_, millis(), P_ON_E, DIRECT),
                                  fan2SpeedPID_(&fan2Speed_, &fan2SpeedControlOutput_, &fanSpeedSetpoint_, fanSpeedKp_, fanSpeedKi_, fanSpeedKd_, millis(), P_ON_E, DIRECT)
{
  // Start off idle
  chamberMode_ = 0;

  // Initialize connection checks
  connected_          = false;
  chamberInitialized_ = false;
  scaleInitialized_   = false;
  
  // Initialize timing checks to false
  cycleStart_         = true;
  humidityTriggered_  = false;
  humidityFetched_    = false;
  fan1SpeedGotten_    = false;
  fan2SpeedGotten_    = false;
  velocityFetched_    = false;
  weightFetched_      = false;
  humidityControlled_ = false;
  fanSpeedControlled_ = false;

  // Weighing scale
  scale_.setScaleOffset(scaleOffset);
  scale_.setScaleFactor(scaleFactor);
  calibratingScale_ = false;
  taringScale_ = false;
  
  // Initialize control variables
  humidityControlIdle_    = true;
  wetSolenoidOn_          = false;
  drySolenoidOn_          = false;
  pumpOn_                 = false;
  humidityControlOutput_  = 0;
  tPumpOn_                = 0;

  // Set all reading serial buffers to 0
  memset(humidityBuffer_,     '\0', 6);
  memset(temperatureBuffer_,  '\0', 6);
  memset(weightBuffer_,       '\0', 6);
  memset(velocityBuffer_,     '\0', 6);
  memset(fan1SpeedBuffer_,    '\0', 7);
  memset(fan2SpeedBuffer_,    '\0', 7);
  memset(errDeviceBuffer_,    '\0', 4);
  memset(errCodeBuffer_,      '\0', 4);
  humidityBuffer_[0]    = '0';
  temperatureBuffer_[0] = '0';
  weightBuffer_[0]      = '0';
  velocityBuffer_[0]    = '0';
  fan1SpeedBuffer_[0]   = '0';
  fan2SpeedBuffer_[0]   = '0';
  errDeviceBuffer_[0]   = '0';
  errCodeBuffer_[0]     = '0';
  
  // Change PID settings
  humidityPID_.SetOutputLimits(-100, 100);
  humidityPID_.SetMode(AUTOMATIC);
  fan1SpeedPID_.SetOutputLimits(PWM_FAN_MIN, PWM_FAN_MAX);  // The PCA9685 PWM ranges from 0 to 4096
  fan1SpeedPID_.SetMode(AUTOMATIC);
  fan2SpeedPID_.SetOutputLimits(PWM_FAN_MIN, PWM_FAN_MAX);  // The PCA9685 PWM ranges from 0 to 4096
  fan2SpeedPID_.SetMode(AUTOMATIC);
}

// Turn off all devices as precaution when class is destructed
HumidityChamber::~HumidityChamber() {
  // We do not send serial messages while destructing the class
  sleepPumpAndSolenoids(false);

  // Ignore if there are errors
  setFanPWM(FAN1, FAN_PWM_OFF, false);
  setFanPWM(FAN2, FAN_PWM_OFF, false);
}

// Initialize the chamber to idle state
void HumidityChamber::init() {
  // Init PCA9685 with totem pole outputs and defaults the output to change on STOP command. See pg 16 of PCA9685 datasheet.
  // Through trial-and-error, it's not reliable to change output on ACK pulse, maybe because the pulse is easily missed, as compared
  // to a STOP command (which is when the command train ends)
  PCA9685Manager_.init(PCA9685Address_, PCA9685_MODE_OUTPUT_TPOLE);
  PCA9685Manager_.setPWMFrequency(1525);  // Set PWM frequency close to max (1526 Hz)
  startIdleMode(false);

  // Inactivate all selection pins
  setDeviceOn(DEVICE_FAN2_SELX, false);
  setDeviceOn(DEVICE_HX711_SELX, false);
  setDeviceOn(DEVICE_FS5_SELX, false);
  setDeviceOn(DEVICE_FAN1_SELX, false);
  setDeviceOn(DEVICE_CLKDAT_ENX, false);
  setDeviceOff(DEVICE_CLKDAT_EN, false);

  // Turn on the fan to max speed
  // Ignore if there are errors
  setFanPWM(FAN1, FAN_PWM_ON, false);
  setFanPWM(FAN2, FAN_PWM_ON, false);

  // Since fans are at max speed, the velocity control algorithm begins by slowing down the fans from max speed
  fan1SpeedPID_.setOutputSum(PWM_FAN_MAX);
  fan2SpeedPID_.setOutputSum(PWM_FAN_MAX);

  chamberInitialized_ = true;
}

// Initialize the scale. This is done after init() and the OE for PCA9685 has been pulled low.
void HumidityChamber::initScale() {
  if (connected_)
  {
    // Now init weighing scale
    // Don't send serial messages during init of scale
    if (connectCLKDATDevice(CLKDATSWITCH_HX711, false))
    {
      scale_.init();
      disconnectCLKDATDevice(false);

      scaleInitialized_ = true;
    }
    else
    {
      disconnectCLKDATDevice(false);
    }
  }
}

// Check if the chamber is connected by attemping to set a dummy pin on PCA9685 to LOW
// Since the PCA9685 is absolutely vital to operation of the chambers, checking if it works or not
// is almost the same as checking if the chamber is connected.
bool HumidityChamber::checkConnection(bool firstCheck, bool sendSerial, bool forceSend)
{
  bool initiallyConnected = connected_;

  // Note that setDeviceOff has an in-built retrying functionality.
  if (setDeviceOff(DEVICE_CONNCHECK, false))  // Disable serial sending; we will handle the error manually
  {
    connected_ = true;

    if (!initiallyConnected)
    { // Was disconnected before, but now is reconnected.
      if (!firstCheck)
      {
        // This isn't the first time this check is done (when Arduino is booted up), and chamber hasn't been initialized yet.
        // This situation happens if the chamber was disconnected since booting up of Arduino, and it has now just been connected, or
        // if the chamber was disconnected during operation and is now reconnected (see the next if-else block; chamberInitialized_
        // and scaleInitialized_ is set to false if the chamber is disconnected.
        if (!chamberInitialized_)
        {
          init();
        }

        if (!scaleInitialized_)
        {
          initScale();
        }
      }

      // Inform C# program that the chamber is connected
      sendConnectionStatus(true, sendSerial);

      // NOTE: When chamber was disconnected, we forced it to enter idle mode. The C# program should provide the Arduino
      // with the last-known settings of the ongoing control routine, if any, to resume the control routine.
    }
  }
  else
  { 
    // Failed to communicate with PCA9685; this could indicate the chamber is disconnected, chamberShield is disconnected, or PCA9685 address set incorrectly.
    connected_ = false;

    if (initiallyConnected)
    {
      // Initially connected, but now is not. Inform C# program that the chamber is disconnected
      sendConnectionStatus(false, sendSerial);

      // Require the chamber and weighing scale to be re-initialized
      chamberInitialized_ = false;
      scaleInitialized_ = false;

      // Change to idle mode, ignore any error messages from PCA9685
      // Note that if we already lost connection to PCA9685, then we won't be able to sleep the pump and solenoids anyway
      startIdleMode(false);
    }
  }

  if (forceSend)
  { // Force send a serial message of the status regardless of what sendSerial is currently set to.
    if (connected_)
    {
      sendConnectionStatus(true, true);
    }
    else
    {
      sendConnectionStatus(false, true);
    }
  }

  return connected_;
}

// Change to idle mode: Do nothing
bool HumidityChamber::startIdleMode(bool sendSerial) {
  // Stop all control devices
  if (sleepPumpAndSolenoids(sendSerial))
  {
    chamberMode_ = 0;
  
    // Set timing checks to false
    cycleStart_         = true;
    humidityTriggered_  = false;
    humidityFetched_    = false;
    fan1SpeedGotten_    = false;
    fan2SpeedGotten_    = false;
    velocityFetched_    = false;
    weightFetched_      = false;
    fanSpeedControlled_ = false;
    humidityControlled_ = false;

    return true;
  }
  else
  {
    return false;
  }
}

// Begin acquisition mode: Acquires data and sends to computer at a given time interval
// This could also be called when stopping control mode
bool HumidityChamber::startAcquisitionMode(unsigned long sendDataInterval, unsigned long curTime, bool sendSerial)  {
  // Stop all control devices
  if (sleepPumpAndSolenoids(sendSerial))
  {
    chamberMode_ = 1;
    sendDataInterval_ = sendDataInterval;

    // Force the next measurement cycle to send a measurement to the computer (i.e. measurement at t=0, even though it's technically not)
    // Note this is done even when returning to DAQ mode from control mode!
    prevDataSentTime_ = curTime - sendDataInterval;

    // Set timing checks to false
    cycleStart_         = true;
    humidityTriggered_  = false;
    humidityFetched_    = false;
    fan1SpeedGotten_    = false;
    fan2SpeedGotten_    = false;
    velocityFetched_    = false;
    weightFetched_      = false;
    fanSpeedControlled_ = false;
    humidityControlled_ = false;

    // Restart measurement cycle.
    cycleStartTime_ = curTime;

    // Reset stored readings in weighing scale
    scale_.resetReadings();

    // Clear the pulse counts for both fans due to beginning of a new cycle
    if (connectCLKDATDevice(CLKDATSWITCH_FAN1, sendSerial))
    {
      fanSpeed_.clearCounts();
      disconnectCLKDATDevice(sendSerial);
    }
    else
    {
      disconnectCLKDATDevice(sendSerial);
    }
    
    if (connectCLKDATDevice(CLKDATSWITCH_FAN2, sendSerial))
    {
      fanSpeed_.clearCounts();
      disconnectCLKDATDevice(sendSerial);
    }
    else
    {
      disconnectCLKDATDevice(sendSerial);
    }

    fan1ReadTimeStart_ = curTime;
    fan2ReadTimeStart_ = curTime;

    // Force the ADC of FS5 to begin a new conversion so that the first velocity reading will be legible.
    if (connectCLKDATDevice(CLKDATSWITCH_FS5, sendSerial))
    {
      vSensor_.triggerMeasurement();
      disconnectCLKDATDevice(sendSerial);
    }

    // Since the first readings have no previous readings to compare against,
    // skip abnormality check for the first readings.
    firstHumidity_ = true;
    firstVelocity_ = true;

    // Reset the readings of HX711
    // TODO: This also happens when we transition from control to acquisition. This
    // shouldn't make a big deal, but could disrupt the stability of readings
    scale_.resetReadings();

    return true;
  }
  else
  {
    return false;
  }
}

// Change to control mode: Acquires data, perform PID control, and sends to computer
bool HumidityChamber::startControlMode(double humiditySetpoint, int velocitySetpoint, unsigned long curTime)  {
  chamberMode_ = 2;
  
  humiditySetpoint_ = humiditySetpoint;
  fanSpeedSetpoint_ = velocitySetpoint;
  
  // Set timing checks to false
  cycleStart_         = true;
  humidityTriggered_  = false;
  humidityFetched_    = false;
  fan1SpeedGotten_    = false;
  fan2SpeedGotten_    = false;
  velocityFetched_    = false;
  weightFetched_      = false;
  fanSpeedControlled_ = false;
  humidityControlled_ = false;
  
  // Restart measurement cycle
  cycleStartTime_ = curTime;
  humidityPID_.setLastTime(curTime);
  fan1SpeedPID_.setLastTime(curTime);
  fan2SpeedPID_.setLastTime(curTime);

  // All the above code doesn't seem to have ways to go wrong..
  return true;
}

// Change the setpoint while the chamber is in control mode
bool HumidityChamber::changeSetpoint(double humiditySetpoint, int velocitySetpoint)  {
  chamberMode_ = 2;
  
  humiditySetpoint_ = humiditySetpoint;
  fanSpeedSetpoint_ = velocitySetpoint;

  // All the above code doesn't seem to have ways to go wrong..
  return true;
}

// Force the next measurement cycle to send readings to the computer, regardless of the interval previously set.
// This method assumes that Acquisition/Control mode has already started
void HumidityChamber::sendReading(unsigned long curTime)  {
  prevDataSentTime_ = curTime - sendDataInterval_; // Force this/next measurement cycle to send a measurement to the computer
}

// Resets the parameters of the PID class. Used when changing setpoint
void HumidityChamber::resetPID(bool affectHumidity)  {
  if (affectHumidity) {
    humidityPID_.Reset();
  } else {
    fan1SpeedPID_.Reset();
    fan2SpeedPID_.Reset();
  }
}

// The main called function; determines what needs to be done based on the given time
void HumidityChamber::checkTiming(unsigned long curTime, bool sendSerial) {
  /* All timing checks should be written in the form of
  *         currentTime - startTime >= interval
  *  as a workaround to time rollover (i.e. exceeding the
  *  max value of unsigned long. Writing the above equation
  *  in other ways may cause unexpected behavior.
  *
  *  Due to the structure of the if-else checks, the timings should
  *  follow the following order:
  *  tHumidityTrigger_ < tHumidityFetch_ < tVelocityGet_ < tWeightGet_ < tControlHumidity_ < tControlVelocity_
  *
  */
  
  // Check if not in idle mode and if chamber is connected
  if (chamberMode_ != 0 && connected_)  {
    // Get reading from HX711 if it is time
    // TODO: Since weight is not needed for environmental control of the chamber, it is actually not necessary
    //       to collect it every cycle; only needed on the cycles where data is being sent to computer. However,
    //       if collecting it every cycle doesn't significantly slow down the program, then leaving it as it is should be OK.
    if (cycleStart_)
    {// Re-identify the time at which data becomes available
      cycleStart_ = false;
      if (connectCLKDATDevice(CLKDATSWITCH_HX711, sendSerial))
      {
        // Reset the time at which data was identified to be available
        scale_.checkTiming(true);

        if (scale_.checkDataAvailability())
        {// Data is available; grab it
          if (!scale_.runDirect())
          {
            sendError(DEVICE_HX711, ERR_HX711_SPI_TIMEOUT, sendSerial);
          }
        }

        disconnectCLKDATDevice(sendSerial);
      }
      else
      {
        disconnectCLKDATDevice(sendSerial);
      }
    }
    else
    {
      if (scale_.checkTiming(false))
      {// Time to get data
        if (connectCLKDATDevice(CLKDATSWITCH_HX711, sendSerial))
        {
          if (scale_.checkDataAvailability())
          {// Data is available; grab it
            if (!scale_.runDirect())
            {
              sendError(DEVICE_HX711, ERR_HX711_SPI_TIMEOUT, sendSerial);
            }
          }

          disconnectCLKDATDevice(sendSerial);
        }
        else
        {
          disconnectCLKDATDevice(sendSerial);
        }
      }
      // Not time to get data, continue on..
    }
    
    // Checks for turning on and off humidifier.
    // This has to be separate from the controlHumidity function because that function relies on the 
    // timing checks, thus the rate at which the humidifier is turned on/off would be limited by the timing
    // check for controlHumidity if the checks for humidifier was merged into that function.
    // Check if currently humidifying
    if (wetSolenoidOn_) {
      if (humidifierOn_ && (curTime - tHumidifierTurnedOn_) >= tHumidifierOnDuration_) {
        // Time to turn off humidifier
        setDeviceOff(DEVICE_HUMID, sendSerial);

        tHumidifierTurnedOff_ = curTime;
        humidifierOn_ = false;
      } else if (!humidifierOn_ && (curTime - tHumidifierTurnedOff_) >= tHumidifierOffDuration_) {
        // Time to turn back on humidifier
        setDeviceOn(DEVICE_HUMID, sendSerial);

        tHumidifierTurnedOn_ = curTime;
        humidifierOn_ = true;
      }
    }
    
	  // Checks for turning on and off air pump
	  // Again, same reasoning as above for why it's separate from the controlHumidity function
	  // Check if humidity control is active.
	  if (pumpOn_	// Only need to switch off if the pump is already on
		    &&
		    tCycle_ - tPumpOn_ > 100	// Only turn off the pump if the off duration is larger than a certain threshold (100 ms), so no dead time between subsequent near-100% dutycycles
		    &&
		    (curTime - tPumpTurnedOn_) >= tPumpOn_)	// Timing check
	  {
      // Time to turn off pump
      setDeviceOff(DEVICE_PUMP, sendSerial);

		  pumpOn_ = false;
    }
	
    // The actual timing checks
    if (humidityTriggered_)   {
      if (humidityFetched_)     {
        if (fan1SpeedGotten_) {
          if (fan2SpeedGotten_) {
            if (weightFetched_) {
              #ifdef MEASURE_AIRVELOCITY
              if (velocityFetched_) {
              #endif // MEASURE_AIRVELOCITY
                // Control mode operations
                if (chamberMode_ == 2) {
                  // In control mode, so perform environmental controls
                  if (!humidityControlled_ && (curTime - cycleStartTime_) >= tControlHumidity_) {
                    controlHumidity(curTime, sendSerial);
                    humidityControlled_ = true;
                  }

                  if (!fanSpeedControlled_ && (curTime - cycleStartTime_) >= tControlFanSpeed_) {
                    controlFanSpeed(curTime, sendSerial);
                    fanSpeedControlled_ = true;
                  }
                }

                // Send DAQ string if the DAQ interval is done (note this is only done
                // once we have acquired all data
                if ((curTime - prevDataSentTime_) >= sendDataInterval_) {
                  prevDataSentTime_ = curTime;
                  printData(sendSerial);
                }

                if ((curTime - cycleStartTime_) >= tCycle_) {
                  // Prevent the interval of next cycle from contracting if there are excessive delays
                  // (happens when adding tCycle_ to cycleStartTime)
                  cycleStartTime_ = millis();

                  // Set timing checks to false to begin next cycle
                  cycleStart_         = true;
                  humidityTriggered_  = false;
                  humidityFetched_    = false;
                  fan1SpeedGotten_    = false;
                  fan2SpeedGotten_    = false;
                  velocityFetched_    = false;
                  weightFetched_      = false;
                  fanSpeedControlled_ = false;
                  humidityControlled_ = false;
                }

              #ifdef MEASURE_AIRVELOCITY
              }
              else if ((curTime - cycleStartTime_) >= tVelocityFetch_) {
                fetchVelocity(sendSerial);
                velocityFetched_ = true;
              }
              #endif // MEASURE_AIRVELOCITY
            }
            else if((curTime - cycleStartTime_) >= tWeightFetch_) {
              fetchWeight(sendSerial);
              weightFetched_ = true;
            }
          }
          else if ((curTime - cycleStartTime_) >= tFan2SpeedGet_) {
            getFanSpeed(FAN2, sendSerial);
            fan2SpeedGotten_ = true;
          }
        } else if ((curTime - cycleStartTime_) >= tFan1SpeedGet_) {
          getFanSpeed(FAN1, sendSerial);
          fan1SpeedGotten_ = true;
        }
      } else if ((curTime - cycleStartTime_) >= tHumidityFetch_) {
        // Attempt to fetch measurement only if it was triggered (the error buffer should be set by the result of triggering)
        if (humidityErrBuffer_ == HIH8000_STATUS_OK) {
          fetchHumidityAndTemperature(sendSerial);
        }
          
        // Regardless of whether we fetched or not, continue the workflow.
        // Error messages will be sent to computer regarding a failed measurement triggering.
        humidityFetched_ = true;
      }
    } else if ((curTime - cycleStartTime_) >= tHumidityTrigger_) {
      triggerHumidity(sendSerial);
      humidityTriggered_ = true;
    }
  }
}

// Used when sending all the collected data
void HumidityChamber::printData(bool sendSerial) {
  if (sendSerial)
  {
#ifndef CALIBRATE_VELOCITY
    Serial.print(SERIAL_SEND_START);
    Serial.print(SERIAL_SEND_ACQUISITION);
    Serial.print(chamberID_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(humidityBuffer_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(temperatureBuffer_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(weightBuffer_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(velocityBuffer_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(fan1SpeedBuffer_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(fan2SpeedBuffer_);
    Serial.print(SERIAL_SEND_END);
    Serial.print(SERIAL_SEND_EOL);
#endif
  }
}

void HumidityChamber::controlHumidity(unsigned long curTime, bool sendSerial) {
  humidityPID_.Compute(curTime);
	
  if (abs(humidityControlOutput_) > PUMP_IDLE_ZONE) {
    if (humidityControlOutput_ > 0) {
      // Humidifying mode
      if (humidityControlIdle_ || !wetSolenoidOn_) {
        // Open solenoid to wet column, close solenoid to dry column
        setDeviceOn(DEVICE_SOL_WET, sendSerial);
        setDeviceOff(DEVICE_SOL_DRY, sendSerial);
        setDeviceOn(DEVICE_HUMID, sendSerial);
        
        tHumidifierTurnedOn_ = curTime;
        
        humidifierOn_ = true;
        wetSolenoidOn_ = true;
        drySolenoidOn_ = false;
        humidityControlIdle_ = false;
      }
    } else {
      // Drying mode
      if (humidityControlIdle_ || !drySolenoidOn_) {
        // Open solenoid to dry column, close solenoid to wet column
        setDeviceOff(DEVICE_SOL_WET, sendSerial);
        setDeviceOn(DEVICE_SOL_DRY, sendSerial);
        setDeviceOff(DEVICE_HUMID, sendSerial);
        
        humidifierOn_ = false;
        wetSolenoidOn_ = false;
        drySolenoidOn_ = true;
        humidityControlIdle_ = false;
      }
    }
    
    // Calculate the appropriate time for turning on/off the pump
	  // The value of humidityControlOutput_ should already be bigger
    // than PUMP_IDLE_ZONE due to if-else checks
		tPumpOn_ = constrain(abs(humidityControlOutput_), PUMP_IDLE_ZONE, 100)/100 * tCycle_;
    setDeviceOn(DEVICE_PUMP, sendSerial);
		tPumpTurnedOn_ = curTime;
		pumpOn_ = true;
    
  } else {
    // Within the idle zone for pump, so turn off pumps and solenoids
    if (!humidityControlIdle_) {
      sleepPumpAndSolenoids(sendSerial);
    }
  }
}

// The duty cycle given to PCA9685 PWM ranges from 0 to 4096 (p 16 of datasheet).
// To be precise, it ranges from 0 to 4095, with 4096 being fully on (a special case,
// which is handled internally by the PCA9685 library).
void HumidityChamber::controlFanSpeed(unsigned long curTime, bool sendSerial) {
  // The PID output ranges from 0 to 4096, as set in the initializer of HumidityChamber
  fan1SpeedPID_.Compute(curTime);
  fan2SpeedPID_.Compute(curTime);

  // Same PWM signal is sent to both fans
  setFanPWM(FAN1, (uint16_t) fan1SpeedControlOutput_, sendSerial);
  setFanPWM(FAN2, (uint16_t) fan2SpeedControlOutput_, sendSerial);
}

// Trigger the humidity sensor to make a measurement. The data is stored in the sensor. Has ~36.65 ms delay until data is available
void HumidityChamber::triggerHumidity(bool sendSerial) {
  humidityErrBuffer_ = humiditySensor_.triggerMeasurement();
  
  if (humidityErrBuffer_ != HIH8000_STATUS_OK) {
    sendError(DEVICE_HIH8000, getHIH8000ErrorCode(humidityErrBuffer_), sendSerial);

	// Empty buffers
	memset(humidityBuffer_, '\0', 5);
	memset(temperatureBuffer_, '\0', 5);

	// Set buffer to error
    humidityBuffer_[0] = 'e';
    temperatureBuffer_[0] = 'e';
  }
}

void HumidityChamber::triggerVelocity(bool sendSerial) {
  if (connectCLKDATDevice(CLKDATSWITCH_FS5, sendSerial))
  {
    vSensor_.triggerMeasurement();
    disconnectCLKDATDevice(sendSerial);
  }
  else
  {
    disconnectCLKDATDevice(sendSerial);
  }
}

// Turn off all solenoids and pump and modify the appropriate state trackers
bool HumidityChamber::sleepPumpAndSolenoids(bool sendSerial)  {
  if (    setDeviceOff(DEVICE_PUMP, sendSerial)
	  &&  setDeviceOff(DEVICE_SOL_WET, sendSerial)
	  &&  setDeviceOff(DEVICE_SOL_DRY, sendSerial)
	  &&  setDeviceOff(DEVICE_HUMID, sendSerial))
  {
    humidifierOn_ = false;
    wetSolenoidOn_ = false;
    drySolenoidOn_ = false;
    pumpOn_ = false;
    humidityControlIdle_ = true;

	return true;
  }
  else
  {
    return false;
  }
}

void HumidityChamber::setFanPWM(FAN fan, uint16_t pwmAmount, bool sendSerial)
{
  // Switch between either fan
  if (fan == FAN1)
  {
    setDevicePWM(DEVICE_FAN1_PWM, pwmAmount, sendSerial);
  }
  else
  {
    setDevicePWM(DEVICE_FAN2_PWM, pwmAmount, sendSerial);
  }
}

// Fetch the newly stored data in the sensor
void HumidityChamber::fetchHumidityAndTemperature(bool sendSerial) {
  humidityErrBuffer_ = humiditySensor_.fetchMeasurement();

  // Empty buffers
  memset(humidityBuffer_, '\0', 5);
  memset(temperatureBuffer_, '\0', 5);
  
  if (humidityErrBuffer_ == HIH8000_STATUS_OK) {  
    // Get humidity reading
    newHumidity_ = humiditySensor_.getHumidity(); // This assignment is necessary because humidity_ is shared with the PID class, and we haven't verified this new reading yet.

    // Check if the humidity reading is abnormal. This is important to prevent a sudden change to the PID algorithm
    if (firstHumidity_ || abs(newHumidity_ - humidity_) < ABNORMAL_THRESHOLD_HUMIDITY)
    {
      // If this is the first reading, we skip the abnormality check
      firstHumidity_ = false;

      // Assign the new reading as a good reading
      humidity_ = newHumidity_;
      abnormalHumidityCount_ = 0;

      dtostrf(humidity_, 1, 1, humidityBuffer_);  // min length of 1, 1 decimal place
      //sprintf(humidityBuffer_, "%01.1f", humidity_);  // sprintf in Arduino doesn't work on float/double
    
      // Temperature
      // Don't have to specially assign temperature to a variable because it's not shared with the PID class
      dtostrf(humiditySensor_.getTemperature(), 1, 1, temperatureBuffer_);          // min length of 1, 1 decimal place
      //sprintf(temperatureBuffer_, "%01.1f", humiditySensor_.getTemperature());    // sprintf in Arduino doesn't work on float/double
    }
    else
    {// This reading is abnormally different from the previous reading.
      abnormalHumidityCount_++;

      if (abnormalHumidityCount_ >= maxAbnormalReadings)
      {// We have multiple consecutive abnormal readings, so assume that the abnormal readings are in fact the actual readings.
        // Assign the new reading as a good reading
        humidity_ = newHumidity_;
        abnormalHumidityCount_ = 0;

        dtostrf(humidity_, 1, 1, humidityBuffer_);  // min length of 1, 1 decimal place
        //sprintf(humidityBuffer_, "%01.1f", humidity_);  // sprintf in Arduino doesn't work on float/double
    
        // Temperature
        // Don't have to specially assign temperature to a variable because it's not shared with the PID class
        dtostrf(humiditySensor_.getTemperature(), 1, 1, temperatureBuffer_);          // min length of 1, 1 decimal place
        //sprintf(temperatureBuffer_, "%01.1f", humiditySensor_.getTemperature());    // sprintf in Arduino doesn't work on float/double
      }
      else
      {// Assume the abnormal reading is, well, abnormal. Consider the temperature reading to be abnormal too (even though sometimes both readings are independent of each other)
        // The PID class will continue to work on the previous good reading.
        sendError(DEVICE_HIH8000, ERR_ABNORMAL_RH, sendSerial);
        humidityBuffer_[0] = 'e';
        temperatureBuffer_[0] = 'e';
      }
    }
  }
  else
  {
    sendError(DEVICE_HIH8000, getHIH8000ErrorCode(humidityErrBuffer_), sendSerial);
    humidityBuffer_[0]    = 'e';
    temperatureBuffer_[0] = 'e';
  }
}

// Get the filtered weight reading and use it for taring/calibrating/nothing special, and prepare for serial output
void HumidityChamber::fetchWeight(bool sendSerial) {
  bool readingExists;

  if (taringScale_) {
    // This round of readings is for taring the weighing scale
    readingExists = scale_.tare(weight_);

    // Send offset value to computer to be recorded
    if (sendSerial)
    {
      if (readingExists)
      {
        Serial.print(SERIAL_SEND_START);
        Serial.print(SERIAL_SEND_SCALEOFFSET);
        Serial.print(chamberID_);
        Serial.print(SERIAL_SEND_SEPARATOR);
        Serial.print(scale_.getScaleOffset());
        Serial.print(SERIAL_SEND_END);
        Serial.print(SERIAL_SEND_EOL);
      }
      else
      {
        sendError(DEVICE_HX711, ERR_ABNORMAL_WEIGHT_TARE, sendSerial);
      }
    }
	
	taringScale_ = false;
  }
  else if (calibratingScale_) {
    // This round of readings is for calculating the scale factor of the weighing scale
    readingExists = scale_.calibrate(weight_, refWeight_);

    // Send scale factor value to computer to be recorded
    if (sendSerial)
    {
      if (readingExists)
      {
        Serial.print(SERIAL_SEND_START);
        Serial.print(SERIAL_SEND_SCALEFACTOR);
        Serial.print(chamberID_);
        Serial.print(SERIAL_SEND_SEPARATOR);
        Serial.print(scale_.getScaleFactor(), scaleFactorDecimalPlaces);
        Serial.print(SERIAL_SEND_END);
        Serial.print(SERIAL_SEND_EOL);
      }
      else
      {
        sendError(DEVICE_HX711, ERR_ABNORMAL_WEIGHT_CAL, sendSerial);
      }
    }
	
	calibratingScale_ = false;
  }
  else {
    // Nothing special; just reading the weight as usual
    readingExists = scale_.calculateWeight(weight_);

    if (!readingExists)
    {
      sendError(DEVICE_HX711, ERR_ABNORMAL_WEIGHT, sendSerial);
    }
  }

  // Prepare the weight reading for serial output
  if (readingExists)
  {
    // Express in grams (no decimals), min length of 1, 0 decimal place
    dtostrf(weight_, 1, 0, weightBuffer_);
  }
  else
  {// No readings were available!
    weightBuffer_[0] = 'e';
  }
}

// Update one of the calibration points for the humidity sensor
void HumidityChamber::applyHumidityCorrection(bool point1, float humidityRaw, float humidityStd)
{
  humiditySensor_.applyHumidityCorrection(point1, humidityRaw, humidityStd);
}

// Next weight reading will be used to calibrate the scaling factor of the weighing scale
void HumidityChamber::calibrateScale(unsigned long curTime, float refWeight) {
  refWeight_ = refWeight;
  calibratingScale_ = true;
  prevDataSentTime_ = curTime - sendDataInterval_; // Force this/next measurement cycle to send a measurement to the computer
}

// Next weight reading will be used to calculate the offset of the weighing scale
void HumidityChamber::tareScale(unsigned long curTime) {
  taringScale_ = true;
  prevDataSentTime_ = curTime - sendDataInterval_; // Force this/next measurement cycle to send a measurement to the computer
}

void HumidityChamber::setScaleOffset(long scaleOffset)
{
  scale_.setScaleOffset(scaleOffset);
}

void HumidityChamber::setScaleFactor(double scaleFactor)
{
  scale_.setScaleFactor(scaleFactor);
}

void HumidityChamber::sleepScale(bool sendSerial)
{
  if (connectCLKDATDevice(CLKDATSWITCH_HX711, sendSerial))
  {
    scale_.powerDown();
    disconnectCLKDATDevice(sendSerial);
  }
  else
  {
    disconnectCLKDATDevice(sendSerial);
  }
}

void HumidityChamber::wakeScale(bool sendSerial)
{
  if (connectCLKDATDevice(CLKDATSWITCH_HX711, sendSerial))
  {
    scale_.powerUp();
    disconnectCLKDATDevice(sendSerial);
  }
  else
  {
    disconnectCLKDATDevice(sendSerial);
  }
}

// Get air velocity reading from the anemometer
void HumidityChamber::fetchVelocity(bool sendSerial) {
	// Disabling air velocity measurement due to unstable air flow pattern inside chamber
  if (connectCLKDATDevice(CLKDATSWITCH_FS5, sendSerial))
  {
	  // Clear buffer
	  memset(velocityBuffer_, '\0', 5);

    if (vSensor_.getMeasurement(newVelocity_))
    {
      disconnectCLKDATDevice(sendSerial);

      // Check if the velocity reading is abnormal. This is important to prevent a sudden change to the PID algorithm
      if (firstVelocity_ || abs(newVelocity_ - velocity_) < ABNORMAL_THRESHOLD_HUMIDITY)
      {
        // If this is the first reading, we skip the abnormality check
        firstVelocity_ = false;

        // Assign the new reading as a good reading
        velocity_ = newVelocity_;
        abnormalVelocityCount_ = 0;

        dtostrf(velocity_, 1, 1, velocityBuffer_);            // min length of 1, 1 decimal place
        //sprintf(velocityBuffer_, "%01.1f", velocity_);      // sprintf in Arduino doesn't work on float/double
      }
      else
      {// This reading is abnormally different from the previous reading.
        abnormalVelocityCount_++;

        if (abnormalVelocityCount_ >= maxAbnormalReadings)
        {// We have multiple consecutive abnormal readings, so assume that the abnormal readings are in fact the actual readings.
          // Assign the new reading as a good reading
          velocity_ = newVelocity_;
          abnormalVelocityCount_ = 0;

          dtostrf(velocity_, 1, 1, velocityBuffer_);            // min length of 1, 1 decimal place
          //sprintf(velocityBuffer_, "%01.1f", velocity_);      // sprintf in Arduino doesn't work on float/double
        }
        else
        {// Assume the abnormal reading is, well, abnormal.
          // The PID class will continue to work on the previous good reading.
          sendError(DEVICE_FS5, ERR_ABNORMAL_VEL, sendSerial);
          velocityBuffer_[0] = 'e';
        }
      }
    }
    else
    {
      disconnectCLKDATDevice(sendSerial);
      sendError(DEVICE_FS5, ERR_FS5_SPI_TIMEOUT, sendSerial);
	    velocityBuffer_[0] = 'e';
    }

#ifdef CALIBRATE_VELOCITY
    if (sendSerial)
    {
      //Serial.print("Raw FS5 reading:");
      Serial.print(velocity_);
      Serial.print('\n');
    }
#endif
  }
  else
  {
    disconnectCLKDATDevice(sendSerial);
  }
  memset(velocityBuffer_, '\0', 5);
  velocityBuffer_[0] = '0';
}

// Get fan speed reading from tachometer of fan
void HumidityChamber::getFanSpeed(FAN fan, bool sendSerial)
{
  // Switch between either fan
  if (fan == FAN1)
  {
    if (connectCLKDATDevice(CLKDATSWITCH_FAN1, sendSerial))
    {
      tachoErrBuffer_ = fanSpeed_.getMeasurement(fan1Speed_, fan1ReadTimeStart_);
      disconnectCLKDATDevice(sendSerial);
	    memset(fan1SpeedBuffer_, '\0', 6); // Empty the array

      if (tachoErrBuffer_ == TACHO_STATUS_OK) {
        dtostrf(fan1Speed_, 1, 1, fan1SpeedBuffer_);  // min length of 1, 1 decimal place
      }
      else
      {
        sendError(DEVICE_FAN1, getTachoErrorCode(tachoErrBuffer_), sendSerial);
        fan1SpeedBuffer_[0] = 'e';
      }
    }
    else
    {
      disconnectCLKDATDevice(sendSerial);
    }
  }
  else
  {
    if (connectCLKDATDevice(CLKDATSWITCH_FAN2, sendSerial))
    {
      tachoErrBuffer_ = fanSpeed_.getMeasurement(fan2Speed_, fan2ReadTimeStart_);
      disconnectCLKDATDevice(sendSerial);
	    memset(fan2SpeedBuffer_, '\0', 6); // Empty the array

      if (tachoErrBuffer_ == TACHO_STATUS_OK) {
        dtostrf(fan2Speed_, 1, 1, fan2SpeedBuffer_);  // min length of 1, 1 decimal place
      }
      else
      {
        sendError(DEVICE_FAN2, getTachoErrorCode(tachoErrBuffer_), sendSerial);
        fan2SpeedBuffer_[0] = 'e';
      }
    }
    else
    {
      disconnectCLKDATDevice(sendSerial);
    }
  }
}

// Turn a device on
bool HumidityChamber::setDeviceOn(Device device, bool sendSerial) {
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    PCA9685ErrBuffer_ = PCA9685Manager_.setChannelOn(device);

    if (PCA9685ErrBuffer_ == PCA9685_STATUS_OK)
    {
      // Add delay to wait for next PCA9685 cycle
      delayMicroseconds(PCA9685CycleTimeDelay_);

      return true;
    }

    tries++;
  }

  // If we reached here, that means communication was not successful
  sendError(device, getPCA9685ErrorCode(device, PCA9685ErrBuffer_), sendSerial);
  return false;
}

// Turn a device off
bool HumidityChamber::setDeviceOff(Device device, bool sendSerial) {
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    PCA9685ErrBuffer_ = PCA9685Manager_.setChannelOff(device);

    if (PCA9685ErrBuffer_ == PCA9685_STATUS_OK)
    {
      // Add delay to wait for next PCA9685 cycle
      delayMicroseconds(PCA9685CycleTimeDelay_);

      return true;
    }

    tries++;
  }

  // If we reached here, that means communication was not successful
  sendError(device, getPCA9685ErrorCode(device, PCA9685ErrBuffer_), sendSerial);
  return false;
}

// Set PWM for a device
bool HumidityChamber::setDevicePWM(Device device, uint16_t pwmAmount, bool sendSerial) {
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    PCA9685ErrBuffer_ = PCA9685Manager_.setChannelPWM(device, pwmAmount);

    if (PCA9685ErrBuffer_ == PCA9685_STATUS_OK)
    {
      // Add delay to wait for next PCA9685 cycle
      delayMicroseconds(PCA9685CycleTimeDelay_);

      return true;
    }

    tries++;
  }

  // If we reached here, that means communication was not successful
  sendError(device, getPCA9685ErrorCode(device, PCA9685ErrBuffer_), sendSerial);
  return false;
}

bool HumidityChamber::connectCLKDATDevice(CLKDATSWITCH device, bool sendSerial) {
  // Connect the CLK, DAT, and connection checking lines to this chamber
  if (setDeviceOn(DEVICE_CLKDAT_EN, sendSerial) && setDeviceOff(DEVICE_CLKDAT_ENX, sendSerial))
  {
    bool switchSuccess = false;

    // Turn on the appropriate select pin
    switch (device) {
      case CLKDATSWITCH_FAN1:
        switchSuccess = setDeviceOff(DEVICE_FAN1_SELX, sendSerial);
        break;
      case CLKDATSWITCH_FS5:
        switchSuccess = setDeviceOff(DEVICE_FS5_SELX, sendSerial);
        break;
      case CLKDATSWITCH_HX711:
        switchSuccess = setDeviceOff(DEVICE_HX711_SELX, sendSerial);
        break;
      case CLKDATSWITCH_FAN2:
        switchSuccess = setDeviceOff(DEVICE_FAN2_SELX, sendSerial);
        break;
      default:
        // Device not listed, fix the code!
        return false;
        break;
    }

    if (switchSuccess)
    {
      // Add delay to wait for next PCA9685 cycle
      delayMicroseconds(PCA9685CycleTimeDelay_);

      // If we reached here, means the PCA9615 connections have been made and we are ready to go
      return true;
    }
    else
    {
      // Failed in switching one of the devices
      return false;
    }
  }
  else
  {
    // Failed in enabling the RS-485 transceivers
    return false;
  }
}

bool HumidityChamber::disconnectCLKDATDevice(bool sendSerial)
{
  // Return clock to idle state (regardless of whether it has been done or not)
  SPI485.writeClockLow();

  // Turn off all the select pins
  setDeviceOn(DEVICE_FAN1_SELX, sendSerial);
  setDeviceOn(DEVICE_FS5_SELX, sendSerial);
  setDeviceOn(DEVICE_HX711_SELX, sendSerial);
  setDeviceOn(DEVICE_FAN2_SELX, sendSerial);

  // Now disconnect the CLK/DAT lines. THIS IS VERY IMPORTANT, otherwise more than 1 chambers
  // will share the same CLK and DAT line, thus fudging up everything. So we must retry at
  // least a few times if we failed to disconnect the line.
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if (setDeviceOff(DEVICE_CLKDAT_EN, sendSerial) && setDeviceOn(DEVICE_CLKDAT_ENX, sendSerial))
    {
      // We don't need to add delay for disconnecting because any subsequent calls to connect
      // the CLKDAT lines would have its own delay.
      //delayMicroseconds(PCA9685CycleTimeDelay_);
      return true;
      break;
    }

    tries++;
  }

  // If all the tries failed, then tell the caller something failed
  return false;
}

uint8_t HumidityChamber::getPCA9685ErrorCode(Device device, PCA9685_STATUS rawErrorCode)
{
  uint8_t errorCode;

  switch (rawErrorCode)
  {
  case PCA9685_STATUS_I2C_START:
    errorCode = ERR_PCA9685_I2C_START;
    break;
  case PCA9685_STATUS_I2C_ACKNACK_MODE:
    errorCode = ERR_PCA9685_I2C_ACKNACK_MODE;
    break;
  case PCA9685_STATUS_I2C_ACKNACK:
    errorCode = ERR_PCA9685_I2C_ACKNACK;
    break;
  case PCA9685_STATUS_I2C_REPSTART:
    errorCode = ERR_PCA9685_I2C_REPSTART;
    break;
  case PCA9685_STATUS_I2C_ACKNACK_RECMODE:
    errorCode = ERR_PCA9685_I2C_ACKNACK_RECMODE;
    break;
  case PCA9685_STATUS_I2C_ACKNACK_REC:
    errorCode = ERR_PCA9685_I2C_ACKNACK_REC;
    break;
  case PCA9685_STATUS_I2C_STOP:
    errorCode = ERR_PCA9685_I2C_STOP;
    break;
  case PCA9685_STATUS_I2C_OTHER:
    errorCode = ERR_PCA9685_I2C_OTHER;
  case PCA9685_STATUS_OTHER:
  default:
    errorCode = ERR_PCA9685_OTHER;
    break;
  }

  return errorCode;
}

uint8_t HumidityChamber::getHIH8000ErrorCode(HIH8000_STATUS rawErrorCode)
{
  switch (rawErrorCode)
  {
  case HIH8000_STATUS_I2C_START:
    return ERR_HIH8000_I2C_START;
    break;
  case HIH8000_STATUS_I2C_ACKNACK_MODE:
    return ERR_HIH8000_I2C_ACKNACK_MODE;
    break;
  case HIH8000_STATUS_I2C_ACKNACK:
    return ERR_HIH8000_I2C_ACKNACK;
    break;
  case HIH8000_STATUS_I2C_REPSTART:
    return ERR_HIH8000_I2C_REPSTART;
    break;
  case HIH8000_STATUS_I2C_ACKNACK_RECMODE:
    return ERR_HIH8000_I2C_ACKNACK_RECMODE;
    break;
  case HIH8000_STATUS_I2C_ACKNACK_REC:
    return ERR_HIH8000_I2C_ACKNACK_REC;
    break;
  case HIH8000_STATUS_I2C_STOP:
    return ERR_HIH8000_I2C_STOP;
    break;
  case HIH8000_STATUS_I2C_OTHER:
    return ERR_HIH8000_I2C_OTHER;
    break;
  case HIH8000_STATUS_ADDRESS:
    return ERR_HIH8000_ADDRESS;
    break;
  case HIH8000_STATUS_OTHER:
  default:
    return ERR_HIH8000_OTHER;
    break;
  }
}

uint8_t HumidityChamber::getTachoErrorCode(TACHO_STATUS rawErrorCode)
{
  switch (rawErrorCode)
  {
  case TACHO_STATUS_I2C_START:
    return ERR_TACHO_I2C_START;
    break;
  case TACHO_STATUS_I2C_ACKNACK_MODE:
    return ERR_TACHO_I2C_ACKNACK_MODE;
    break;
  case TACHO_STATUS_I2C_ACKNACK:
    return ERR_TACHO_I2C_ACKNACK;
    break;
  case TACHO_STATUS_I2C_REPSTART:
    return ERR_TACHO_I2C_REPSTART;
    break;
  case TACHO_STATUS_I2C_ACKNACK_RECMODE:
    return ERR_TACHO_I2C_ACKNACK_RECMODE;
    break;
  case TACHO_STATUS_I2C_ACKNACK_REC:
    return ERR_TACHO_I2C_ACKNACK_REC;
    break;
  case TACHO_STATUS_I2C_STOP:
    return ERR_TACHO_I2C_STOP;
    break;
  case TACHO_STATUS_I2C_OTHER:
    return ERR_TACHO_I2C_OTHER;
    break;
  case TACHO_STATUS_SPI_TIMEOUT:
    return ERR_TACHO_SPI_TIMEOUT;
    break;
  case TACHO_STATUS_OTHER:
  default:
    return ERR_TACHO_OTHER;
    break;
  }
}

void HumidityChamber::sendError(Device device, uint8_t errorCode, bool sendSerial)
{
  if (sendSerial)
  {
    Serial.print(SERIAL_SEND_START);
    Serial.print(SERIAL_SEND_ERROR);
    Serial.print(chamberID_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(device);
    Serial.print(SERIAL_SEND_SEPARATOR);
    Serial.print(errorCode);
    Serial.print(SERIAL_SEND_END);
    Serial.print(SERIAL_SEND_EOL);
  }
}

void HumidityChamber::sendConnectionStatus(bool connected, bool sendSerial)
{
  if (sendSerial)
  {
    Serial.print(SERIAL_SEND_START);
    Serial.print(SERIAL_SEND_CHAMBERCONN);
    Serial.print(chamberID_);
    Serial.print(SERIAL_SEND_SEPARATOR);
    if (connected)
    {
      Serial.print(SERIAL_SEND_CHAMBERCONN_CONN);
    }
    else
    {
      Serial.print(SERIAL_SEND_CHAMBERCONN_DISC);
    }
    Serial.print(SERIAL_SEND_END);
    Serial.print(SERIAL_SEND_EOL);
  }
}
