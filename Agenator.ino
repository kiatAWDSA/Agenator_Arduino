/***************************************************************************
  The Agenator is an open source system capable of:
  a) measuring relative humidity, weight, fan speed, and air velocity
  b) controlling relative humidity and fan speed.

  The program was designed and tested for Arduino Uno. However, due to the large
  size of the compiled program, the Atmega328P chip cannot accomodate the
  compiled program size. It is necessary to upgrade the microcontroller chip to
  Atmega1284P. This could be done, for example, by purchasing the Arduino UU:
  http://www.firebirduino.com/uu/index.html
  
  IMPORTANT: Due to the current design, each chamberShield MUST be plugged
  into a valid chamber, because it relies on PCA9685 to disable the
  RS-422 transceivers. Future iterations should have pull resistors on
  chamberShield to disable the RS-422 transceivers when no chamber is connected.

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
***************************************************************************/

// Uncomment only if calibrating FS5 sensor
// Do the same for the HumidityChamber.h and FS5.h files.
// #define CALIBRATE_VELOCITY 1

#include <RS485SPI.h>
#include <I2C.h>  // use custom library from DSSCircuits that is non-blocking. The built-in Wire library has tendency to freeze up Arduino after a certain time. http://dsscircuits.com/articles/arduino-i2c-master-library
#include <HumidityChamber.h>

// To be removed in final program
#include <PID_modified.h>
#include <HIH8000_customI2C.h>
#include <TachoSensor.h>
#include <FS5.h>
#include <HX711_modified.h>
#include <PCA9685_customI2C.h>

// Communication with the Agenator C# program on computer
const char SERIAL_CMD_START                   = '^';
const char SERIAL_CMD_CONNECTION_ARDUINO      = 'w';
const char SERIAL_CMD_CONNECTION_CHAMBER      = 'g';
const char SERIAL_CMD_READ                    = 'r';
const char SERIAL_CMD_RH_CALIBRATE            = 'h';
const char SERIAL_CMD_SCALE_TARE              = 't';
const char SERIAL_CMD_SCALE_CALIBRATE         = 'k';
const char SERIAL_CMD_STATE_IDLE              = 'i';
const char SERIAL_CMD_STATE_ACQUISITION       = 'a';
const char SERIAL_CMD_STATE_CONTROL           = 'c';
const char SERIAL_CMD_STATE_CONTROL_SETPOINT  = 's';
const char SERIAL_CMD_DISCONNECT              = 'b';
const char SERIAL_CMD_UNKNOWN                 = 'u';  // Command sent from C# program is unknown
const char SERIAL_CMD_SEPARATOR               = '|';
const char SERIAL_CMD_END                     = '@';
const char SERIAL_CMD_EOL                     = '\n';
// const char SERIAL_CMD_RESETPID                = 'd';
// const char SERIAL_CMD_STATE_CONTROL_HUMIDITY  = 'h';
// const char SERIAL_CMD_STATE_CONTROL_VELOCITY  = 'v';
const char SERIAL_SEND_START                  = '^';
const char SERIAL_SEND_CONNECTION             = 'y';  // Used to respond to SERIAL_CMD_CONNECTION_CONTROL
const char SERIAL_SEND_CORRUPTCMD             = 'x';
  const char SERIAL_SEND_CORRUPTCMD_START         = 's';  // The command string does not have a start flag
  const char SERIAL_SEND_CORRUPTCMD_END           = 'e';  // The command string does not have an end flag
  const char SERIAL_SEND_CORRUPTCMD_PARAM_LESS    = 'l';  // There are fewer params in a command line than expected
  const char SERIAL_SEND_CORRUPTCMD_PARAM_MORE    = 'm';  // There are more params in a command line than expected
  const char SERIAL_SEND_CORRUPTCMD_PARAM_NONE    = 'n';  // There are no params in a command line, even though some are expected
const char SERIAL_SEND_CHAMBER_CMDRESPONSE    = 'r';  // Used to indicate execution status of a received command
  const char SERIAL_SEND_CHAMBER_CMDRESPONSE_SUCC = 'y';  // Success
  const char SERIAL_SEND_CHAMBER_CMDRESPONSE_FAIL = 'n';  // Failed
const char SERIAL_SEND_SEPARATOR              = '|';
const char SERIAL_SEND_END                    = '@';
const char SERIAL_SEND_EOL                    = '\n';
const char SERIAL_SEND_CMDERROR               = '|';

// Number of parameters in every command sent by computer
const uint8_t MAXPARAM_RH_CAL         = 5;
const uint8_t MAXPARAM_SCALE_CAL      = 3;
const uint8_t MAXPARAM_SCALE_TARE     = 2;
const uint8_t MAXPARAM_ACQUISITION    = 9;
const uint8_t MAXPARAM_CONTROL        = 4;
const uint8_t MAXPARAM_SETPOINT       = 4;
const uint8_t MAXPARAM_IDLE           = 2;
const uint8_t MAXPARAM_READ           = 2;


// Arduino connection checks
// This will remain false until we receive SERIAL_CMD_CONNECTION_CONTROL from the computer.
// Additionally, the main loop constantly checks if we are receiving SERIAL_CMD_CONNECTION_CONTROL in
// regular intervals (computerConnectionCheckInterval). If we don't, this value will be set to false again.
bool computerConnected                        = false;  
unsigned long lastComputerConnectionCheck;
const uint16_t computerConnectionCheckInterval      = 4000;

// Pins
const uint8_t PIN_PCA9615_EN = A0;
const uint8_t PIN_PCA9685_OE = 4;
const uint8_t PIN_CLK_OUT = 6;
const uint8_t PIN_CLK_IN = SCK;
const uint8_t PIN_DAT_OUT = MISO;
const uint8_t PIN_DAT_IN = MOSI;
const uint8_t PIN_SS = SS;

// Serial communication and buffers
const uint8_t serialBufferLength = 128; // Should be always sufficient for command strings sent by the C# program
const uint8_t paramMaxCount = 9;        // Max possible of params in a command sent by computer.
const uint8_t paramBufferLength = 20;   // Should be always sufficient for params in command strings sent by the C# program
char serialBuffer[serialBufferLength];
char paramBufferElement0[paramBufferLength];
char paramBufferElement1[paramBufferLength];
char paramBufferElement2[paramBufferLength];
char paramBufferElement3[paramBufferLength];
char paramBufferElement4[paramBufferLength];
char paramBufferElement5[paramBufferLength];
char paramBufferElement6[paramBufferLength];
char paramBufferElement7[paramBufferLength];
char paramBufferElement8[paramBufferLength];
// This array of char array pointers will be temp storage for params extracted from incoming command lines
// Only parts of the buffer are used for each command; see the constants defined above for MAXPARAM
char* paramBuffer[paramMaxCount] = {paramBufferElement0 ,
                                    paramBufferElement1, 
                                    paramBufferElement2, 
                                    paramBufferElement3, 
                                    paramBufferElement4,
                                    paramBufferElement5, 
                                    paramBufferElement6, 
                                    paramBufferElement7, 
                                    paramBufferElement8 };
char trimmedSerialBuffer[serialBufferLength]; // For holding the unextracted params portion of the command string

// Chamber vars
const uint8_t chambersCount = 12;
unsigned long curTime;
unsigned long lastChamberConnectionCheck;
const uint16_t chamberConnectionCheckInterval = 1500;

// List of chambers
//                                                      Chamber no.    PCA9685 address    Humidity I2C address   I2C class     scaleOffset   scaleFactor  tHumidityTrigger  tHumidityFetch  tFan1SpeedGet    tFan2SpeedGet    tWeightFetch    tVelocityFetch  tControlHumidity    tControlFanSpeed         tCycle       tHumidifierOnDuration   tHumidifierOffDuration    PUMP_IDLE_ZONE     Vel. mean   Vel. std dev    Vel. x0 coef    Vel. x1 coef    Vel. x2 coef    Vel. x3 coef       humidityKp  humidityKi    humidityKd  fanSpeedKp fanSpeedKi fanSpeedKd
HumidityChamber chambers[chambersCount] = { HumidityChamber(0,                0,                    10,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47320,        1137,           0.063,          0.298,          0.735,          0.733,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(1,                1,                    11,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47140,        1160,           0.007,          0.223,          0.821,          0.789,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(2,                2,                    12,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             46870,        1082,           0.034,          0.250,          0.775,          0.761,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(3,                3,                    13,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47040,        1048,           -0.013,         0.224,          0.951,          0.915,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(4,                4,                    14,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47420,        1107,           0.075,          0.360,          0.832,          0.799,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(5,                5,                    15,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47730,        1088,           0.138,          0.471,          0.784,          0.740,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(6,                6,                    16,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47470,        1144,           0.063,          0.309,          0.802,          0.821,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(7,                7,                    17,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47050,        1060,           0.042,          0.278,          0.849,          0.865,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(8,                8,                    18,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47610,        1111,           0.083,          0.319,          0.806,          0.868,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(9,                9,                    19,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47410,        1207,           -0.05,          0.127,          0.944,          0.98,              5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(10,               10,                   20,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47170,        1084,           0.045,          0.279,          0.853,          0.928,             5,        0.0002,        100000,      0.0001,     0.0002,      1000),
                                            HumidityChamber(11,               11,                   21,             I2c,          1000,       100,              300,              650,          660,             680,              700,             750,            800,                 850,               1000,               1500,                    5000,                 2,             47850,        1139,           0.011,          0.224,          0.899,          0.982,             5,        0.0002,        100000,      0.0001,     0.0002,      1000)
                                            };

void setup() {
  // During initialization, ALL serial sending should be disabled.

  // Set up pins
  //digitalWrite(PIN_PCA9615_EN, LOW);
  digitalWrite(PIN_PCA9685_OE, HIGH); // Disable all PCA9685 output
  digitalWrite(PIN_PCA9615_EN, LOW);  // Disable PCA9615 until start-up sequence is complete (datasheet pg 8)
  pinMode(PIN_PCA9685_OE, OUTPUT);
  pinMode(PIN_PCA9615_EN, OUTPUT);

  // Give ample time for everything to power-up and stabilize
  delay(300);

  digitalWrite(PIN_PCA9615_EN, HIGH); // The ample delay above should have provided enough time for the start-up sequence.
  delayMicroseconds(300);             // Account for PCA9685 t_idle (datasheet pg 8)

  // Initialize RS-485 bus
  SPI485.begin(PIN_CLK_OUT, PIN_CLK_IN, PIN_DAT_OUT, PIN_DAT_IN, PIN_SS, SPI_SLAVE, 10);  

  // Perform power-up and enable sequence for PCA9615; refer to datasheet pg 8.
  //delay(15); // Give time for PCA9615 to power-up and stabilize
  //digitalWrite(PIN_PCA9615_EN, HIGH); // Now enable PCA9615

  // Set up I2C
  // Note: The Arduino UU seems to have problems pulling the I2C lines to GND (will be ~0.6V)
  // if 2k Ohm pull-ups are used. The PCA9615 only reads LOW if the signal is < 0.4 V. Testing
  // showed that a higher value of pull-up (5.1 k) allows the line to be pulled low enough (~0.1V)
  // while giving decently shaped square waves.
  I2c.timeOut(100);      // Timeout after 30 ms
  I2c.begin(false);     // True to use internal pull-up resistors; false for external pull-ups
  I2c.setSpeed(false);  // true for 400 kHz (fast-mode); false for 100 kHz (standard mode)

  // Initialize the chambers
  for (uint8_t l = 0; l < chambersCount; l++)
  {
    chambers[l].init();
    delay(50);
  }

  // Once the chambers have been initialized, then we enable all PCA9685 chips
  digitalWrite(PIN_PCA9685_OE, LOW);
  delayMicroseconds(100);

  // Check the connection of the chambers
  for (uint8_t l = 0; l < chambersCount; l++)
  {
    chambers[l].checkConnection(true, false, false);
  }

  lastChamberConnectionCheck = millis();

  // Initialize the weighing scales for all chambers
  for (uint8_t l = 0; l < chambersCount; l++)
  {
    chambers[l].initScale();
  }

#ifdef CALIBRATE_VELOCITY
  computerConnected = true;
#endif

  // Start serial communication only after everything has been initialized
  Serial.begin(19200);

  // Clear up the Serial buffer
  while (Serial.available())
  {
    Serial.read();
  }
}

void loop() {
  curTime = millis();
  
  // Check chamber connection
  if (curTime - lastChamberConnectionCheck >= chamberConnectionCheckInterval)
  {
    lastChamberConnectionCheck = curTime;

    for (uint8_t l = 0; l < chambersCount; l++)
    {
      chambers[l].checkConnection(false, computerConnected, false);
    }
  }
  
#ifndef CALIBRATE_VELOCITY
  // Haven't received connection check from computer for an extended period.
  // Consider the connection as dropped
  if (computerConnected && curTime - lastComputerConnectionCheck >= computerConnectionCheckInterval)
  {
    lastComputerConnectionCheck = curTime;

    if (computerConnected)
    {
      computerConnected = false;
    }
  }
#endif

  for(uint8_t l = 0; l < chambersCount; l++)
  {
    chambers[l].checkTiming(curTime, computerConnected);
  }
}

void serialEvent() {
  /* The Serial buffer is only checked at the end of each loop() iteration:
   * https://forum.arduino.cc/index.php?topic=166650.0
   * This means more than one command could accumulate in the buffer during a single loop() iteration.
   * It is thus necessary to go thru the entire buffer until we are sure it is empty.
   * This can be done by re-checking the buffer [Serial.available()] after processing one command from the buffer:
   * https://arduino.stackexchange.com/a/26416
   */
  while (Serial.available())
  {
    // All communication must start with SERIAL_CMD_START
    if (Serial.peek() == SERIAL_CMD_START)
    {
      // Extract this command string
      memset(serialBuffer, '\0', serialBufferLength);
      Serial.readBytesUntil(SERIAL_CMD_EOL, serialBuffer, (serialBufferLength - 1));

      // Get the pointer to SERIAL_CMD_END 
      char* endFlag = strchr(serialBuffer, SERIAL_CMD_END);

      // All communication must end with SERIAL_CMD_END 
      // Check if it isn't missing and that there's nothing coming after it
      if (endFlag && serialBuffer[endFlag - serialBuffer + 1] == '\0')
      {
        uint8_t commandPos = 1; // Account for offset from SERIAL_CMD_START
        uint8_t endPos = endFlag - serialBuffer;// Index of SERIAL_CMD_END
        char commandType = serialBuffer[commandPos];

        switch (commandType)
        {
        case SERIAL_CMD_CONNECTION_ARDUINO:
          /*********************************
          *   ARDUINO CONNECTION CHECK     *
          * *******************************/
          /* Computer checking for connection to Arduino. This doesn't have command IDs.
          * Format:
          * ^c@
          * where    ^    is SERIAL_CMD_START
          *          c    is SERIAL_CMD_CONNECTION_CONTROL
          *          @    is SERIAL_CMD_END
          */
          lastComputerConnectionCheck = millis();
          computerConnected = true;

          Serial.print(SERIAL_SEND_START);
          Serial.print(SERIAL_SEND_CONNECTION);
          Serial.print(SERIAL_SEND_END);
          Serial.print(SERIAL_SEND_EOL);
          break;
        case SERIAL_CMD_CONNECTION_CHAMBER:
          /*********************************
          *       Chamber Connection       *
          * *******************************/
          /* Check the connection of the chambers
          * Format:
          * ^c[commandID]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_CONNECTION_CHAMBER
          *          [commandID]     is the ID for this command
          *          @               is SERIAL_CMD_END
          */
          lastChamberConnectionCheck = millis();
          computerConnected = true;

          // The HumidityChamber instances will send their own messages
          for (uint8_t l = 0; l < chambersCount; l++)
          {
            chambers[l].checkConnection(false, computerConnected, true);
          }
          break;
        case SERIAL_CMD_RH_CALIBRATE:
          /*********************************
          *    Calibrate humidity sensor    *
          * *******************************/
          /* Calibrate the relative humidity sensor against a humidity standard
          * Format:
          * ^c[commandID]|[chamberID]|[pointNo.]|[humidityRaw]|[humidityStd]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_RH_CALIBRATE
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          [pointNo.]      is 1 or 2, depending on which point we are doing for the 2-point calibration
          *          [humidityRaw]   is the humidity reading of the sensor (%)
          *          [humidityStd]   is the theoretical humidity of the standard (%)
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          // Index 2 is the ID of the calibration point
          // Index 3 is the raw humidity (%)
          // Index 4 is the standard humidity (%)
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_RH_CAL))
          {
            uint8_t commandID = atoi(paramBuffer[0]);
            uint8_t chamberID = atoi(paramBuffer[1]);

            if (atoi(paramBuffer[2]) == 1)
            {// Point 1
              chambers[chamberID].applyHumidityCorrection(true, atof(paramBuffer[3]), atof(paramBuffer[4]));
            }
            else
            {// Point 2
              chambers[chamberID].applyHumidityCorrection(false, atof(paramBuffer[3]), atof(paramBuffer[4]));
            }

            sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_RH_CALIBRATE, true);
          }
          // If there were problems while extracting params, they were handled by extractCommandParams().
          break;
        case SERIAL_CMD_SCALE_CALIBRATE:
          /*********************************
          *    Calibrate weighing scale    *
          * *******************************/
          /* Calibrate the weighing scale against a given reference weight
          * Format:
          * ^c[commandID]|[chamberID]|[scaleFactor]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_SCALE_CALIBRATE
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          [scaleFactor]   is the reference weight (g)
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          // Index 2 is reference weight (g)
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_SCALE_CAL))
          {
            uint8_t commandID = atoi(paramBuffer[0]);
            uint8_t chamberID = atoi(paramBuffer[1]);

            chambers[chamberID].calibrateScale(millis(), atof(paramBuffer[2]));

            sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_SCALE_CALIBRATE, true);
          }
          // If there were problems while extracting params, they were handled by extractCommandParams().
          break;
        case SERIAL_CMD_SCALE_TARE:
          /*********************************
          *       Tare weighing scale      *
          * *******************************/
          /* Tare the weighing scale
          * Format:
          * ^c[commandID]|[chamberID]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_SCALE_TARE
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_SCALE_TARE))
          {
            uint8_t commandID = atoi(paramBuffer[0]);
            uint8_t chamberID = atoi(paramBuffer[1]);

            chambers[chamberID].tareScale(millis());

            sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_SCALE_TARE, true);
          }
          // If there were problems while extracting params, they were handled by extractCommandParams().
          break;
        case SERIAL_CMD_STATE_ACQUISITION:
        {
          /*********************************
          *            Begin DAQ           *
          * *******************************/
          /* Begin data acquisition for a specific chamber
          * Format:
          * ^c[commandID]|[chamberID]|[DAQInterval]|[scaleOffset]|[scaleFactor]|[humidityRaw1]|[humidityStd1]|[humidityRaw2]|[humidityStd2]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_STATE_ACQUISITION
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          [DAQInterval]   is the time interval (s) between each attempt to send data to computer
          *          [scaleOffset]   is the offset for the weighing scale
          *          [scaleFactor]   is the calibration factor for the weighing scale
          *          [humidityRaw1]  is the raw humidity reading for 1st RH calibration point
          *          [humidityStd1]  is the humidity reading for a standard for 1st RH calibration point
          *          [humidityRaw2]  is the raw humidity reading for 2nd RH calibration point
          *          [humidityStd2]  is the humidity reading for a standard for 2nd RH calibration point
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          // Index 2 is DAQInterval
          // Index 3 is scaleOffset
          // Index 4 is scaleFactor
          // Index 5 is humidityRaw1
          // Index 6 is humidityStd1
          // Index 7 is humidityRaw2
          // Index 8 is humidityStd2
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_ACQUISITION))
          {
            uint8_t commandID = atoi(paramBuffer[0]);
            uint8_t chamberID = atoi(paramBuffer[1]);

            // Computer sends DAQ interval in seconds (integers), convert to ms
            unsigned long DAQInterval = atol(paramBuffer[2]) * 1000;

            // Set weighing scale offset
            chambers[chamberID].setScaleOffset(atol(paramBuffer[3]));

            // Set weighing scale factor
            chambers[chamberID].setScaleFactor(atof(paramBuffer[4]));

            // Apply corrections to humidity sensor
            chambers[chamberID].applyHumidityCorrection(true, atof(paramBuffer[5]), atof(paramBuffer[6]));
            chambers[chamberID].applyHumidityCorrection(false, atof(paramBuffer[7]), atof(paramBuffer[8]));

            // Begin DAQ mode
            if (chambers[chamberID].startAcquisitionMode(DAQInterval, millis(), computerConnected))
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_ACQUISITION, true);
            }
            else
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_ACQUISITION, false);
            }
          }
          // If there were problems while extracting params, they were handled by extractCommandParams().
        }
          break;
        case SERIAL_CMD_STATE_CONTROL:
        {
          /*********************************
          *       Begin control mode       *
          * *******************************/
          /* Begin control mode for a specific chamber
          * Format:
          * ^c[commandID]|[chamberID]|[RHSetpoint]|[fSetpoint]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_STATE_CONTROL
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          [RHSetpoint]    is the relative humidity setpoint
          *          [fSetpoint]     is the fan speed setpoint
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          // Index 2 is RHSetpoint
          // Index 3 is fSetpoint
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_CONTROL))
          {
            uint8_t commandID = atoi(paramBuffer[0]);
            uint8_t chamberID = atoi(paramBuffer[1]);

            // Begin control mode
            if (chambers[chamberID].startControlMode(atof(paramBuffer[2]), atof(paramBuffer[3]), millis()))
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_CONTROL, true);
            }
            else
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_CONTROL, false);
            }
          }
          // If there were problems while extracting params, they were handled by extractCommandParams().
        }
          break;
        case SERIAL_CMD_STATE_CONTROL_SETPOINT:
        {
          /*********************************
          * Change setpoint of control mode*
          * *******************************/
          /* Change the setpoints of an ongoing control program in a chamber
          * Format:
          * ^c[commandID]|[chamberID]|[RHSetpoint]|[fSetpoint]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_STATE_CONTROL_SETPOINT
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          [RHSetpoint]    is the relative humidity setpoint
          *          [fSetpoint]     is the fan speed setpoint
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          // Index 1 is RHSetpoint
          // Index 2 is fSetpoint
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_SETPOINT))
          {
            uint8_t commandID = atoi(paramBuffer[0]);
            uint8_t chamberID = atoi(paramBuffer[1]);

            // Begin control mode
            if (chambers[chamberID].changeSetpoint(atof(paramBuffer[2]), atof(paramBuffer[3])))
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_CONTROL_SETPOINT, true);
            }
            else
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_CONTROL_SETPOINT, false);
            }
          }
          // If there were problems while extracting params, they were handled by extractCommandParams().
        }
          break;
        case SERIAL_CMD_STATE_IDLE:
        {
          /*********************************
          *    Set chamber to idle mode    *
          * *******************************/
          /* Stops all operations in a chamber
          * Format:
          * ^c[commandID]|[chamberID]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_STATE_IDLE
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_IDLE))
          {
            uint8_t commandID = atoi(paramBuffer[0]);
            uint8_t chamberID = atoi(paramBuffer[1]);

            if (chambers[chamberID].startIdleMode(computerConnected))
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_IDLE, true);
            }
            else
            {
              sendChamberCommandResponse(commandID, chamberID, SERIAL_CMD_STATE_IDLE, false);
            }
          }
        }
          break;
        case SERIAL_CMD_READ:
          /*********************************
          *         Send a reading         *
          * *******************************/
          /* Forces the chamber to send a single reading of all its measurements
          *  on the end of the current measurement cycle (This means its not
          *  immediate).
          * Format:
          * ^c[commandID]|[chamberID]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_READ
          *          [commandID]     is the ID for this command
          *          |               is SERIAL_CMD_SEPARATOR
          *          [chamberID]     is the ID for the chamber
          *          @               is SERIAL_CMD_END
          */

          // Index 0 is commandID
          // Index 1 is chamberID
          if (extractCommandParams(serialBuffer, paramBuffer, commandPos, endPos, MAXPARAM_READ))
          {
            chambers[atoi(paramBuffer[1])].sendReading(millis());
          }
          break;
        case SERIAL_CMD_DISCONNECT:
          /*********************************
          *    End serial communication    *
          * *******************************/
          /* Computer no longer wishes to receive serial messages from
          *  Arduino. Note that this doesn't end the Serial class itself.
          * Format:
          * ^c[commandID]@
          * where    ^               is SERIAL_CMD_START
          *          c               is SERIAL_CMD_DISCONNECT
          *          [commandID]     is the ID for this command
          *          @               is SERIAL_CMD_END
          */
          computerConnected = false;

          // Force all chambers to idle mode
          for (uint8_t i = 0; i < chambersCount; i++)
          {
            chambers[i].startIdleMode(false);
          }
          break;
        /* This is left here in case we wish to expand this in the future. This resets the PID settings.
        case SERIAL_CMD_RESETPID:
          // Move to the identifier for humidity or velocity
          i++;

          if (serialBuffer[i] == SERIAL_CMD_STATE_CONTROL_HUMIDITY) {
          // Reset the humidity PID
          chambers[targetChamber].resetPID(true);
          } else {
          // Reset the velocity PID
          chambers[targetChamber].resetPID(false);
          }

          break;
        */
        default:
          // Unknown command received; could be due to noise
          sendCommandError(SERIAL_CMD_UNKNOWN);
          break;
        }
      }
      else
      {// SERIAL_CMD_END wasn't seen or in wrong location, so assume this is garbage
        // Let C# program know
        sendCommandError(SERIAL_SEND_CORRUPTCMD_END);
      }
    }
    else
    {// SERIAL_CMD_START wasn't seen, so assume this is garbage
      // Grab the buffer, but don't store it anywhere. This helps to clear the buffer
      Serial.read();
      // Let C# program know
      sendCommandError(SERIAL_SEND_CORRUPTCMD_START);
    }
  }
}

// Inform C# program that a given command was corrupt
// We could also send the actual command for better debugging, but that would
// take more communication time. Leave this feature out for now.
void sendCommandError(char corruptionType)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_CORRUPTCMD);
  Serial.print(corruptionType);
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

// Inform C# program on the status of a command for a specific chamber
void sendChamberCommandResponse(uint8_t commandID, uint8_t chamberID, char commandType, bool success)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_CHAMBER_CMDRESPONSE);
  Serial.print(commandID);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(chamberID);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(commandType);
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (success)
  {
    Serial.print(SERIAL_SEND_CHAMBER_CMDRESPONSE_SUCC);
  }
  else
  {
    Serial.print(SERIAL_SEND_CHAMBER_CMDRESPONSE_FAIL);
  }
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}


bool extractCommandParams(char *rawBuffer, char *params[paramBufferLength], uint8_t commandPos, uint8_t endPos, uint8_t paramsCount)
{
  // First, extract out the params portion of the command string
  trimBuffer(rawBuffer, trimmedSerialBuffer, serialBufferLength, commandPos, endPos);

  // Extract first param
  uint8_t paramsExtracted = 0;
  char *tokenPtr = strtok(trimmedSerialBuffer, &SERIAL_CMD_SEPARATOR);

  if (tokenPtr)
  {
    do
    {
      paramsExtracted++;
      
      // Clear the output buffer
      memset(params[paramsExtracted - 1], '\0', paramBufferLength);
      strcpy(params[paramsExtracted - 1], tokenPtr); // IMPORTANT: Make sure that params[paramsExtracted - 1] is a char array with at least as many elements as tokenPtr
      tokenPtr = strtok(NULL, &SERIAL_CMD_SEPARATOR);
    } while (paramsExtracted < paramsCount && tokenPtr);

    // Check if the number of extracted params are as expected
    // (because we might exit the while loop above due to tokenPtr being null)
    if (paramsExtracted < paramsCount)
    {
      // Not enough params were extracted
      sendCommandError(SERIAL_SEND_CORRUPTCMD_PARAM_LESS);
      return false;
    }

    // Check if there are still params remaining
    // (because we might exit the while loop above due to paramsExtracted >= paramsCount)
    if (tokenPtr)
    {
      // Too many params given in command, though not all were extracted
      sendCommandError(SERIAL_SEND_CORRUPTCMD_PARAM_MORE);
      return false;
    }

    return true;
  }
  else
  {
    // Not a single param is available
    sendCommandError(SERIAL_SEND_CORRUPTCMD_PARAM_NONE);
    return false;
  }
}

// Extract the params portion of the command string
void trimBuffer(char* inputBuffer, char* outputBuffer, uint8_t outputBufferSize, uint8_t commandPos, uint8_t endPos)
{
  // Clear the output buffer
  memset(outputBuffer, '\0', outputBufferSize);

  uint8_t paramsLength = endPos - commandPos - 1;
  strncpy(outputBuffer, inputBuffer + 2, paramsLength);	// +2 to account for SERIAL_CMD_START and the command type
}
