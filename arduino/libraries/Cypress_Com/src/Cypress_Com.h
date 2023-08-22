// ######################################

//========== Cypress_Com.h ============

// ######################################

/// @file Used for the Cypress_Com class

#ifndef _CYPRESS_COMM_h
#define _CYPRESS_COMM_h

//============= INCLUDE ================
#include "Arduino.h"
#include <Wire.h>
#include "Cypress_Com_Base.h"
#include "Maze_Debug.h"

/// @brief This class handles all of the Cypress chip I2C comms.
///
/// @remarks This class uses an instance of the Maze_Debug class.
///
/// @remarks Status codes from @ref Wire::beginTransmission()
/// 	0: Success. Indicates that the transmission was successful.
/// 	1: Data too long to fit in transmit buffer. This error indicates that the data you tried
///			to send is larger than the library's buffer.
/// 	2: Received NACK on transmit of address. This means the slave device did not acknowledge its address.
/// 		This can happen if the device is not connected or if there is an issue with the I2C address.
/// 	3: Received NACK on transmit of data. This means the slave device acknowledged its address but
///			did not acknowledge the receipt of data.
/// 	4: Other error. A miscellaneous error occurred.
class Cypress_Com
{

	// ---------VARIABLES-----------------
public:
	// // 1x3 testing addresses
	uint8_t ADDR_LIST[9]{
		0x12, 0x10, 0x6, 0x8, 0xA,
		0xC, 0xE, 0x14, 0x16};

	// // 3x3 testing addresses
	// uint8_t ADDR_LIST[1]{0x2};

	// // 3x3 maze addresses
	// uint8_t ADDR_LIST[9]
	// {
	// 	0x2, 0x4, 0x6, 0x8, 0xA,
	//  0xC, 0xE, 0x10, 0x12
	// };

	// // 7x7 maze addresses
	//  uint8_t ADDR_LIST[49]{ ///list of all possible I2C addresses
	//  0x2, 0x4, 0x6, 0x8, 0xA,
	//  0xC, 0xE, 0x10, 0x12, 0x14,
	//  0x16, 0x18, 0x1A, 0x1C, 0x1E,
	//  0x20, 0x22, 0x24, 0x26, 0x28,
	//  0x2A, 0x2C, 0x2E, 0x30, 0x32,
	//  0x34, 0x36, 0x38, 0x3A, 0x3C,
	//  0x3E, 0x40, 0x42, 0x44, 0x46,
	//  0x48, 0x4A, 0x4C, 0x4E, 0x50,
	//  0x52, 0x54, 0x56, 0x58, 0x5A,
	//  0x5C, 0x5E, 0x60, 0x62
	//  };

	// PWM config
	const uint8_t pwmClockVal = 0;	 /// PWM clock config [0: 32 kHz(default), 1: 24 MHz, 2: 1.5 MHz, 3: 93.75 kHz, 4: 367.6 Hz(programmable), 5: previous PWM]
	const uint8_t pwmPeriodVal = 32; /// PWM period of the PWM counter(1 - 255).Devisor for hardward clock

private:
	Maze_Debug _Dbg; /// unique instance of Maze_Debug class

	// -----------METHODS-----------------
public:
	Cypress_Com();

public:
	uint8_t wireEndTransmissionWrapper(bool = true, bool = true);

public:
	uint8_t i2cRead(uint8_t, uint8_t, uint8_t[], uint8_t = 1);

public:
	uint8_t i2cWrite(uint8_t, uint8_t, uint8_t);
	uint8_t i2cWrite(uint8_t, uint8_t, uint8_t[], uint8_t);

private:
	void _updateRegByte(uint8_t &, uint8_t, uint8_t);

public:
	uint8_t ioReadPin(uint8_t, uint8_t, uint8_t, uint8_t &);

public:
	uint8_t ioWritePin(uint8_t, uint8_t, uint8_t, uint8_t);

public:
	uint8_t ioReadPort(uint8_t, uint8_t, uint8_t, uint8_t &);

public:
	uint8_t ioWritePort(uint8_t, uint8_t, uint8_t, uint8_t);

public:
	uint8_t ioReadReg(uint8_t, uint8_t, uint8_t[], uint8_t);

public:
	uint8_t ioWriteReg(uint8_t, uint8_t[], uint8_t, uint8_t, uint8_t[] = nullptr);

public:
	uint8_t setupCypress(uint8_t);

public:
	uint8_t setupSourcePWM(uint8_t, uint8_t, uint8_t);

public:
	uint8_t setSourceDutyPWM(uint8_t, uint8_t, uint8_t);

public:
	uint8_t setPortRegister(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

public:
	uint8_t i2cScan();
};

#endif
