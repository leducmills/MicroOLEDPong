#include "SFE_MicroOLED.h"
#include "application.h"
//#include "SPI.h"
//#include "Wire.h"

#define I2C_FREQ 400000L

void MicroOLED::spiSetup()
{
	/*
	ssport		= portOutputRegister(digitalPinToPort(csPin));
	sspinmask	= digitalPinToBitMask(csPin);
	ssreg		= portModeRegister(digitalPinToPort(csPin));
	*/
	pinMode(MOSI, OUTPUT);
	pinMode(SCK, OUTPUT);

	pinMode(csPin, OUTPUT);
	digitalWrite(csPin, HIGH);

	SPI.setClockDivider(SPI_CLOCK_DIV2);
	//SPI.setDataMode(SPI_MODE0);
	pinMode(csPin, OUTPUT);
	//pinMode(10, OUTPUT); // Required for setting into Master mode
	digitalWrite(csPin, HIGH);
	SPI.begin();
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
}

void MicroOLED::spiTransfer(uint8_t data)
{
	SPI.transfer(data);
}

void MicroOLED::i2cSetup()
{
	/*
	Wire.begin();

	// SCL frequency = (F_CPU) / (16 + 2(TWBR) * (prescalar))
	TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
	*/
}

void MicroOLED::i2cWrite(uint8_t address, uint8_t dc, uint8_t data)
{
	/*
	Wire.beginTransmission(address);
	Wire.write(dc); // If data = 0, if command = 0x40
	Wire.write(data);
	Wire.endTransmission();
	*/
}

void MicroOLED::parallelSetup()
{
}

void MicroOLED::parallelWrite(uint8_t data, uint8_t dc)
{
	/*
	// chip select high->low
	*ssport &= ~sspinmask;		// SS LOW

	// dc high or low
	//digitalWrite(dcPin, dc);
	if (dc)
		*dcport |= dcpinmask;	// DC HIGH
	else
		*dcport &= ~dcpinmask;		// DC pin LOW

	// wr high->low
	//digitalWrite(wrPin, LOW);
	*wrport &= ~wrpinmask;		// SS LOW

	// set data pins
	PORTD = data;

	// wr low->high
	*wrport |= wrpinmask;	// SS HIGH
	//digitalWrite(wrPin, HIGH);

	// cs high
	//digitalWrite(csPin, HIGH);
	*ssport |= sspinmask;	// SS HIGH
	*/
}
