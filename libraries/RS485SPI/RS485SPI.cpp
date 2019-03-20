#include "RS485SPI.h"

RS485SPI::RS485SPI()
{
}

// Initialize the Arduino for SPI communications.
// clkPin: pin for bit-banging OUT clock pulses
// bitOrder: Choose from MSBFIRST or LSBFIRST
// spiRole: Choose from SPI_MASTER or SPI_SLAVE
// spiMode: Choose from SPIMODE_0 thru SPIMODE_3. See http://avrbeginners.net/architecture/spi/spi.html for explanation of each mode.
// timeOut: Maximum time to wait (ms) for receiving a byte
void RS485SPI::begin(uint8_t clkOutPin, uint8_t clkInPin, uint8_t misoPin, uint8_t mosiPin, uint8_t ssPin, uint8_t spiRole, unsigned long timeOut = 1000)
{
  clkOutPin_ = clkOutPin;
  mosiPin_ = mosiPin;

  // Reset the SPI control register
  SPCR = 0;

  // Set this Arduino to be master or slave (spiRole)
  SPCR |= (spiRole << MSTR);

  // Configure the pins
  digitalWrite(clkOutPin, LOW);
  pinMode(clkOutPin, OUTPUT);
  pinMode(clkInPin, INPUT);
  pinMode(misoPin, OUTPUT);
  pinMode(mosiPin, INPUT);
  pinMode(ssPin, INPUT);

  setTimeOut(timeOut);

  enableSPI();
}

void RS485SPI::end()
{
  // Reset the SPI control register
  SPCR = 0;
}

void RS485SPI::enableSPI()
{
  SPDR = 0;
  SPCR |= (1 << SPE); // 0b01000000
}

void RS485SPI::disableSPI()
{
  SPDR = 0;
  SPCR &= ~(1 << SPE); // 0b10111111
}

// Maximum time to wait (ms) for receiving a byte
void RS485SPI::setTimeOut(unsigned long timeOut)
{
  timeOut_ = timeOut;
}

// Receives a single byte and stores it in the buffer provided in function argument.
// Returns 1 if successful, 0 if timeout.
/*
bool RS485SPI::receiveByte(uint8_t& buffer)
{
  // 8 clock pulses for one byte
  for (uint8_t i = 0; i < 8; i++)
  {
    digitalWrite(clkOutPin_, HIGH);
    digitalWrite(clkOutPin_, LOW);
  }

  // Create a timer for timing out the receiving if it takes too long
  startTime_ = millis();

  // Wait for the end of the transmission
  while (!(SPSR & (1 << SPIF)))
  {
    if (millis() - timeOut_ > startTime_)
    {
      // read the buffer anyway to clear SPIF flag
      uint8_t throwaway = SPDR;
      // Timeout
      return false;
    }
  };

  // Reading SPDR and SPIF (done in loop above) clears the SPIF flag
  buffer = SPDR;
  return true;
}
*/

// Sets the clock polarity (CPOL) and clock phase (CPHA).
// idleHigh == false => CPOL == 0         i.e. clock idles LOW
// idleHigh == true => CPOL == 1          i.e. clock idles HIGH
// sampleTrailing == false => CPHA == 0   i.e. data is sampled at the leading edge of clock pulse
// sampleTrailing == true => CPHA == 1    i.e. data is sampled at the trailing edge of clock pulse
void RS485SPI::setSPIMode(bool idleHigh, bool sampleTrailing)
{
  if (idleHigh)
  {
    // Clock idles high, set CPOL to 1
    SPCR |= (1 << CPOL);
  }
  else
  {
    // Clock idles low, set CPOL to 0
    SPCR &= ~(1 << CPOL);
  }

  if (sampleTrailing)
  {
    // Data is sampled at the trailing edge of clock pulse, set CPHA to 1
    SPCR |= (1 << CPHA);
  }
  else
  {
    // Data is sampled at the leading edge of clock pulse, set CPHA to 0
    SPCR &= ~(1 << CPHA);
  }
}

// Temporarily change the bitOrder for one receiving cycle
// bitOrder defaults to MSBFIRST if not specified
bool RS485SPI::receiveByte(uint8_t& buffer, uint8_t bitOrder)
{
  setBitOrder(bitOrder);
    
  // Clear data register (also "sends" an empty packet thru MISO pin, which is not connected)
  SPDR = 0;

  // 8 clock pulses for one byte
  for (uint8_t i = 0; i < 8; i++)
  {
    pulseClock();
  }

  // Create a timer for timing out the receiving if it takes too long
  startTime_ = millis();

  // Wait for the end of the transmission
  while (!(SPSR & (1 << SPIF)))
  {
    if (millis() - timeOut_ > startTime_)
    {
      // Timeout
      return false;
    }
  };

  // Reading SPDR and SPIF (done in loop above) clears the SPIF flag
  buffer = SPDR;
  return true;
}

int RS485SPI::readMOSI()
{
  return digitalRead(mosiPin_);
}

void RS485SPI::writeClockHigh()
{
  digitalWrite(clkOutPin_, HIGH);
}

void RS485SPI::writeClockLow()
{
  digitalWrite(clkOutPin_, LOW);
}

void RS485SPI::pulseClock()
{
  writeClockHigh();
  writeClockLow();
}

// Choose the bit order; watch out that MSBFIRST is 1, but the DORD bit on SPCR should be set to 0 for most significant byte first
void RS485SPI::setBitOrder(uint8_t bitOrder)
{
  if (bitOrder == MSBFIRST)
  {
    // Change bit order to MSB
    SPCR &= ~(1 << DORD); // 0b11011111
  }
  else
  {
    // Change bit order to LSB
    SPCR |= (1 << DORD);  // 0b00100000
  }
}


RS485SPI SPI485 = RS485SPI();