/******************************************************************************
SFE_LSM9DS1.cpp
SFE_LSM9DS1 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file implements all functions of the LSM9DS1 class. Functions here range
from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.6
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SFE_LSM9DS1.h"
#include "application.h"

LSM9DS1::LSM9DS1()
{
}

LSM9DS1::LSM9DS1(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	// interfaceMode will keep track of whether we're using SPI or I2C:
	interfaceMode = interface;

	// mAddress and xgAddress will store the 7-bit I2C address, if using I2C.
	// If we're using SPI, these variables store the chip-select pins.
	mAddress = mAddr;
	xgAddress = xgAddr;
}


void LSM9DS1::init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	// interfaceMode will keep track of whether we're using SPI or I2C:
	interfaceMode = interface;

	// mAddress and xgAddress will store the 7-bit I2C address, if using I2C.
	// If we're using SPI, these variables store the chip-select pins.
	mAddress = mAddr;
	xgAddress = xgAddr;
}

uint16_t LSM9DS1::begin(gyro_scale gScl, accel_scale aScl, mag_scale mScl,
						gyro_odr gODR, accel_odr aODR, mag_odr mODR)
{
	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	gScale = gScl;
	aScale = aScl;
	mScale = mScl;

	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	// Now, initialize our hardware interface.
	if (interfaceMode == DOF_MODE_I2C)	// If we're using I2C
		initI2C();					// Initialize I2C
	else if (interfaceMode == DOF_MODE_SPI) 	// else, if we're using SPI
		initSPI();							// Initialize SPI

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xlTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I

	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
	setGyroScale(gScale); // Set the gyro range

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(aODR); // Set the accel data rate.
	setAccelScale(aScale); // Set the accel range.

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(mODR); // Set the magnetometer output data rate.
	setMagScale(mScale); // Set the magnetometer's range.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (xlTest << 8) | mTest;
}

void LSM9DS1::initGyro()
{
	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection
	xgWriteByte(CTRL_REG1_G, 0xC0); // 952 Hz OD, 33 Hz cutoff

	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	xgWriteByte(CTRL_REG2_G, 0x00);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	xgWriteByte(CTRL_REG3_G, 0x00);

	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	xgWriteByte(CTRL_REG4, 0x38);
}

void LSM9DS1::initAccel()
{
	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	xgWriteByte(CTRL_REG5_XL, 0x38);

	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	xgWriteByte(CTRL_REG6_XL, 0x00);

	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	xgWriteByte(CTRL_REG7_XL, 0x00);
}

void LSM9DS1::initMag()
{
	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	mWriteByte(CTRL_REG1_M, 0x1C); // 80 Hz ODR

	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	mWriteByte(CTRL_REG2_M, 0x00); // +/-4Gauss

	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	mWriteByte(CTRL_REG3_M, 0x00); // Continuous conversion mode

	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	mWriteByte(CTRL_REG4_M, 0x00);

	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	mWriteByte(CTRL_REG5_M, 0x00);
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS1, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
/*
void LSM9DS1::calLSM9DS1(float * gbias, float * abias)
{
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	int samples, ii;

	// First get gyro bias
	byte c = gReadByte(CTRL_REG5_G);
	gWriteByte(CTRL_REG5_G, c | 0x40);         // Enable gyro FIFO
	delay(20);                                 // Wait for change to take effect
	gWriteByte(FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (gReadByte(FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

	for(ii = 0; ii < samples ; ii++)
	{	// Read the gyro data stored in the FIFO
		gReadBytes(OUT_X_L_G,  &data[0], 6);
		gyro_bias[0] += (((int16_t)data[1] << 8) | data[0]);
		gyro_bias[1] += (((int16_t)data[3] << 8) | data[2]);
		gyro_bias[2] += (((int16_t)data[5] << 8) | data[4]);
	}

	gyro_bias[0] /= samples; // average the data
	gyro_bias[1] /= samples;
	gyro_bias[2] /= samples;

	gbias[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
	gbias[1] = (float)gyro_bias[1]*gRes;
	gbias[2] = (float)gyro_bias[2]*gRes;

	c = gReadByte(CTRL_REG5_G);
	gWriteByte(CTRL_REG5_G, c & ~0x40);  // Disable gyro FIFO
	delay(20);
	gWriteByte(FIFO_CTRL_REG_G, 0x00);   // Enable gyro bypass mode


	//  Now get the accelerometer biases
	c = xmReadByte(CTRL_REG0_XM);
	xmWriteByte(CTRL_REG0_XM, c | 0x40);      // Enable accelerometer FIFO
	delay(20);                                // Wait for change to take effect
	xmWriteByte(FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable accelerometer FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (xmReadByte(FIFO_SRC_REG) & 0x1F); // Read number of stored accelerometer samples

	for(ii = 0; ii < samples ; ii++)
	{	// Read the accelerometer data stored in the FIFO
		xmReadBytes(OUT_X_L_A, &data[0], 6);
		accel_bias[0] += (((int16_t)data[1] << 8) | data[0]);
		accel_bias[1] += (((int16_t)data[3] << 8) | data[2]);
		accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) - (int16_t)(1./aRes); // Assumes sensor facing up!
	}

	accel_bias[0] /= samples; // average the data
	accel_bias[1] /= samples;
	accel_bias[2] /= samples;

	abias[0] = (float)accel_bias[0]*aRes; // Properly scale data to get gs
	abias[1] = (float)accel_bias[1]*aRes;
	abias[2] = (float)accel_bias[2]*aRes;

	c = xmReadByte(CTRL_REG0_XM);
	xmWriteByte(CTRL_REG0_XM, c & ~0x40);    // Disable accelerometer FIFO
	delay(20);
	xmWriteByte(FIFO_CTRL_REG, 0x00);       // Enable accelerometer bypass mode
}
*/

void LSM9DS1::readAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
	xgReadBytes(OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void LSM9DS1::readMag()
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp
	mReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void LSM9DS1::readTemp()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp
	xgReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L
	temperature = (((int16_t) temp[1] << 12) | temp[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
}

void LSM9DS1::readGyro()
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	xgReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

float LSM9DS1::calcGyro(int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro;
}

float LSM9DS1::calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float LSM9DS1::calcMag(int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void LSM9DS1::setGyroScale(gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG1_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= gScl << 3;
	// And write the new register value back into CTRL_REG4_G:
	xgWriteByte(CTRL_REG1_G, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes();
}

void LSM9DS1::setAccelScale(accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG6_XL);
	// Then mask out the accel scale bits:
	temp &= 0xC7;	//0xFF^(0x2 << 3);
	// Then shift in our new scale bits:
	temp |= (aScl << 3);
	// And write the new register value back into CTRL_REG2_XM:
	xgWriteByte(CTRL_REG6_XL, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void LSM9DS1::setMagScale(mag_scale mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void LSM9DS1::setGyroODR(gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0x7 << 5);
	// Then shift in our new ODR bits:
	temp |= (gRate << 5);
	// And write the new register value back into CTRL_REG1_G:
	xgWriteByte(CTRL_REG1_G, temp);
}

void LSM9DS1::setAccelODR(accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG6_XL);
	// Then mask out the accel ODR bits:
	temp &= 0x1F;
	// Then shift in our new ODR bits:
	temp |= (aRate << 5);
	// And write the new register value back into CTRL_REG1_XM:
	xgWriteByte(CTRL_REG6_XL, temp);
}

/*
void LSM9DS1::setAccelABW(accel_abw abwRate)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG2_XM);
	// Then mask out the accel ABW bits:
	temp &= 0xFF^(0x3 << 7);
	// Then shift in our new ODR bits:
	temp |= (abwRate << 7);
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp);
}
*/

void LSM9DS1::setMagODR(mag_odr mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}

void LSM9DS1::configAccelIntTHS(interrupt_select intSelect, uint8_t ths_x_xl,
							 uint8_t ths_y_xl, uint8_t ths_z_xl)
{
	/*
	uint8_t temp;
	if (intSelect == INT1)
	{
		temp = xgReadByte(INT1_CTRL);
		temp |= (1<<6);
		xgWriteByte(INT1_CTRL
	}*/
}


void LSM9DS1::configAccelIntDRDY(interrupt_select intSelect, bool enable)
{
	uint8_t temp;
	uint8_t intRegister = intSelect;
	temp = xgReadByte(intRegister);
	if (enable)
		temp |= (1<<0);
	else
		temp &= ~(1<<0);
	xgWriteByte(intRegister, temp);
}

void LSM9DS1::configGyroIntDRDY(interrupt_select intSelect, bool enable)
{
	uint8_t temp;
	uint8_t intRegister = intSelect;
	temp = xgReadByte(intRegister);
	if (enable)
		temp |= (1<<1);
	else
		temp &= ~(1<<1);
	xgWriteByte(intRegister, temp);
}

void LSM9DS1::configAGInterrupt(h_lactive activeLow, pp_od pushPull)
{
	uint8_t temp;
	temp = xgReadByte(CTRL_REG8);

	if (activeLow)
		temp |= (1<<5);
	else
		temp &= ~(1<<5);

	if (pushPull)
		temp &= ~(1<<4);
	else
		temp |= (1<<4);

	xgWriteByte(CTRL_REG8, temp);
}

void LSM9DS1::configMInterrupt(h_lactive activeLow)
{
	uint8_t temp;
	temp = mReadByte(INT_CFG_M);

	if (activeLow)
		temp &= ~(1<<2);
	else
		temp |= (1<<2);

	mWriteByte(INT_CFG_M, temp);
}

void LSM9DS1::configMagIntTHS(int threshold, bool xien, bool yien, bool zien)
{
	uint8_t msb, lsb, temp;
	msb = (uint8_t)((threshold & 0x7F00) >> 8);
	lsb = (uint8_t)(threshold & 0x00FF);
	mWriteByte(INT_THS_L_M, lsb);
	mWriteByte(INT_THS_H_M, msb);

	temp = mReadByte(INT_CFG_M);
	if (xien)
		temp |= (1<<7);
	else
		temp &= ~(1<<7);
	if (yien)
		temp |= (1<<6);
	else
		temp &= ~(1<<6);
	if (zien)
		temp |= (1<<5);
	else
		temp &= ~(1<<5);
	temp |= 1; // Set IEN
	mWriteByte(INT_CFG_M, temp);
}

uint8_t LSM9DS1::getMagIntSrc()
{
	return mReadByte(INT_SRC_M);
}

/*
void LSM9DS1::configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX, uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration)
{
	gWriteByte(INT1_CFG_G, int1Cfg);
	gWriteByte(INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(INT1_THS_XL_G, (int1ThsX & 0xFF));
	gWriteByte(INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(INT1_THS_YL_G, (int1ThsY & 0xFF));
	gWriteByte(INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		gWriteByte(INT1_DURATION_G, 0x80 | duration);
	else
		gWriteByte(INT1_DURATION_G, 0x00);
}
*/
void LSM9DS1::calcgRes()
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	uint8_t scale;
	switch (gScale)
	{
	case G_SCALE_245DPS:
		scale = 245;
		break;
	case G_SCALE_500DPS:
		scale = 500;
		break;
	case G_SCALE_2000DPS:
		scale = 2000;
		break;
	}
	gRes = (float)(scale) / 32768.0;
}

void LSM9DS1::calcaRes()
{
	uint8_t scale;
	switch (aScale)
	{
	case A_SCALE_2G:
		scale = 2;
		break;
	case A_SCALE_4G:
		scale = 4;
		break;
	case A_SCALE_8G:
		scale = 8;
		break;
	case A_SCALE_16G:
		scale = 16;
		break;
	}
	aRes = (float)(scale) / 32768.0;
}

void LSM9DS1::calcmRes()
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 4 Gs (00), 8 Gs (01), 12 Gs (10) 16 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	mRes = (float)(4 * (mScale + 1)) / 32768.0;
}


void LSM9DS1::xgWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (interfaceMode == DOF_MODE_I2C)
		I2CwriteByte(xgAddress, subAddress, data);
	else if (interfaceMode == DOF_MODE_SPI)
		SPIwriteByte(xgAddress, subAddress, data);
}

void LSM9DS1::mWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (interfaceMode == DOF_MODE_I2C)
		return I2CwriteByte(mAddress, subAddress, data);
	else if (interfaceMode == DOF_MODE_SPI)
		return SPIwriteByte(mAddress, subAddress, data);
}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (interfaceMode == DOF_MODE_I2C)
		return I2CreadByte(xgAddress, subAddress);
	else if (interfaceMode == DOF_MODE_SPI)
		return SPIreadByte(xgAddress, subAddress);
}

void LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	if (interfaceMode == DOF_MODE_I2C)
		I2CreadBytes(xgAddress, subAddress, dest, count);
	else if (interfaceMode == DOF_MODE_SPI)
		SPIreadBytes(xgAddress, subAddress, dest, count);
}

uint8_t LSM9DS1::mReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (interfaceMode == DOF_MODE_I2C)
		return I2CreadByte(mAddress, subAddress);
	else if (interfaceMode == DOF_MODE_SPI)
		return SPIreadByte(mAddress, subAddress);
}

void LSM9DS1::mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (interfaceMode == DOF_MODE_I2C)
		I2CreadBytes(mAddress, subAddress, dest, count);
	else if (interfaceMode == DOF_MODE_SPI)
		SPIreadBytes(mAddress, subAddress, dest, count);
}

void LSM9DS1::initSPI()
{
	pinMode(xgAddress, OUTPUT);
	digitalWrite(xgAddress, HIGH);
	pinMode(mAddress, OUTPUT);
	digitalWrite(mAddress, HIGH);

	SPI.begin();
	// Maximum SPI frequency is 10MHz, could divide by 2 here:
	SPI.setClockDivider(SPI_CLOCK_DIV4);
	// Data is read and written MSb first.
	SPI.setBitOrder(MSBFIRST);
	// Data is captured on rising edge of clock (CPHA = 0)
	// Base value of the clock is HIGH (CPOL = 1)
	SPI.setDataMode(SPI_MODE1);
}

void LSM9DS1::SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data)
{
	digitalWrite(csPin, LOW); // Initiate communication

	// If write, bit 0 (MSB) should be 0
	// If single write, bit 1 should be 0
	SPI.transfer(subAddress & 0x3F); // Send Address
	SPI.transfer(data); // Send data

	digitalWrite(csPin, HIGH); // Close communication
}

uint8_t LSM9DS1::SPIreadByte(uint8_t csPin, uint8_t subAddress)
{
	uint8_t temp;
	// Use the multiple read function to read 1 byte.
	// Value is returned to `temp`.
	SPIreadBytes(csPin, subAddress, &temp, 1);
	return temp;
}

void LSM9DS1::SPIreadBytes(uint8_t csPin, uint8_t subAddress,
							uint8_t * dest, uint8_t count)
{
	digitalWrite(csPin, LOW); // Initiate communication
	// To indicate a read, set bit 0 (msb) to 1
	// If we're reading multiple bytes, set bit 1 to 1
	// The remaining six bytes are the address to be read
	if (count > 1)
		SPI.transfer(0xC0 | (subAddress & 0x3F));
	else
		SPI.transfer(0x80 | (subAddress & 0x3F));
	for (int i=0; i<count; i++)
	{
		dest[i] = SPI.transfer(0x00); // Read into destination array
	}
	digitalWrite(csPin, HIGH); // Close communication
}

void LSM9DS1::initI2C()
{
	Wire.begin();	// Initialize I2C library
}

// Wire.h read and write protocols
void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	Wire.write(subAddress | 0x80);     // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, count);  // Read bytes from slave register address
	for (int i=0; i<count;)
	{
		if (Wire.available())
		{
			dest[i++] = Wire.read();
		}
	}
	/*while (Wire.available())
	{
		dest[i++] = Wire.read(); // Put read results in the Rx buffer
	}*/
}
