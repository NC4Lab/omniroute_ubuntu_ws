// // ######################################

//========= CypressCom.cpp ===========

// ######################################

/// @file Used for the CypressCom class

//============= INCLUDE ================
#include "CypressCom.h"
#include "CypressComBase.h"

//========CLASS: CypressCom==========

/// <summary>
/// Constructor
/// </summary>
CypressCom::CypressCom() {}

//------------------------ LOW-LEVEL METHODS ------------------------

/// @brief Initialize wire coms and setup I2C.
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::i2cInit()
{
	// Join I2C bus
	Wire.begin();

	// Set I2C timeout to 5 second
	Wire.setWireTimeout(5000000); // (us) for Wire librarary (default: 25000)
	Wire.setTimeout(5000);		   // (ms) for Stream librarary

	// Scan I2C bus for Cypress chips and print found addresses
	i2cScan();
}

/// @brief Lowest level function to read from a given Cypress register.
///
/// @param address I2C address for a given Cypress chip.
/// @param reg Register to read from.
/// @param p_byte_out_arr Byte array from the register (used as output).
/// @param s Length of p_byte_out_arr [1-16].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::i2cRead(uint8_t address, uint8_t reg, uint8_t p_byte_out_arr[], uint8_t s)
{
	if (s > 16)
		return -1;
	_beginTransmissionWrapper(address);
	Wire.write(reg);
	uint8_t resp = _endTransmissionWrapper(false); // master stops sending but keeps the transmission line open
	if (resp == 0)
	{
		Wire.requestFrom(address, s);

		int k = 0;
		while (Wire.available() && k < s)
		{
			p_byte_out_arr[k] = Wire.read();
			k++;
		}

		if (k < s)
			return 1;
		return 0;
	}
	else
		return resp;
}

/// @brief Lowest level function to write to a given Cypress register.
///
/// @param address I2C address for a given Cypress chip.
/// @param reg Register to read from.
/// @param byte_val_in Value to set the registry pin/bit to [0,1].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::i2cWrite(uint8_t address, uint8_t reg, uint8_t byte_val_in)
{
	_beginTransmissionWrapper(address);
	Wire.write(reg);
	Wire.write(byte_val_in);
	return _endTransmissionWrapper();
}
/// @brief OVERLOAD: Option to pass an array of bytes "array "p_byte_val_in_arr" to set multiple
/// registers beginning at register specified by "reg".
///
/// @param p_byte_val_in_arr Byte pointer array with the registry values.
/// @param s Length of the "p_byte_val_in_arr" array [1-16]
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::i2cWrite(uint8_t address, uint8_t reg, uint8_t p_byte_val_in_arr[], uint8_t s)
{
	if (s > 16)
		return -1;
	_beginTransmissionWrapper(address);
	Wire.write(reg);
	for (size_t i = 0; i < s; i++)
	{
		Wire.write(p_byte_val_in_arr[i]);
	}
	return _endTransmissionWrapper();
}

/// @brief Updates a given byte value based on a given mask.
///
/// @note This is used by several of the write methods to update the byte values read in from a register based on the provided byte mask before they are updated in the register.
///
/// @param r_byte_val_out Reference to the byte value to change (used as output).
/// @param byte_mask Byte value in which bits set to one denote the bit to set in "r_byte_val_out".
/// @param bit_val_set Value to set the bits to [0,1].
void CypressCom::_updateRegByte(uint8_t &r_byte_val_out, uint8_t byte_mask, uint8_t bit_val_set)
{
	if (bit_val_set == 1)
	{ // bitwise comparison, set bit in byte1 to 1 if it or the corresponding bit in byte2 is equal to 1
		r_byte_val_out = r_byte_val_out | byte_mask;
	}
	else
	{ // bitwise comparison, set bit in byte1 to 0 if the corresponding bit is equal to 1 in byte2
		r_byte_val_out = r_byte_val_out & ~byte_mask;
	}
}

//------------------------ MID-LEVEL METHODS ------------------------

/// @brief Read from a given IO pin associated with a given limit switch.
///
/// @note This is only setup for input, not output, registers.
/// @note See, for example, @ref WallOperation::wms.ioUp for wall to port mapping.
///
/// @param address I2C address for a given Cypress chip.
/// @param port Number of port to set [0-5].
/// @param pin Pin number to read [0,7].
/// @param r_bit_val_out Reference with the registry bit value (used as output).
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::ioReadPin(uint8_t address, uint8_t port, uint8_t pin, uint8_t &r_bit_val_out)
{
	if (port > 5 || pin > 7)
		return -1;
	else
	{
		uint8_t byte_val_out_arr[1];
		uint8_t resp = i2cRead(address, REG_GI0 + port, byte_val_out_arr, 1);
		delay(1); // hack to deal with strange resp variable behavior
		if (resp == 0)
			r_bit_val_out = bitRead(byte_val_out_arr[0], pin);
		return resp;
	}
}

/// @brief Read from a given IO pin associated with a given limit switch.
///
/// @note See, for example, @ref WallOperation::wms.pwmUp for wall to port mapping.
///
/// @param address I2C address for a given Cypress chip.
/// @param port Number of port to set [0-5].
/// @param pin Pin number to read [0,7].
/// @param bit_val_set Value to set the bits to [0,1].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::ioWritePin(uint8_t address, uint8_t port, uint8_t pin, uint8_t bit_val_set)
{
	if (port > 5 || pin > 7)
		return -1;
	else
	{
		uint8_t byte_val_out_arr[1];
		uint8_t resp = i2cRead(address, REG_GO0 + port, byte_val_out_arr, 1); // get current port registry value
		if (resp == 0)
		{
			uint8_t byte_val_in = byte_val_out_arr[0];
			bitWrite(byte_val_in, pin, bit_val_set);			   // set pin specific bit of uint8_t
			resp = i2cWrite(address, REG_GO0 + port, byte_val_in); // update register
		}
		return resp;
	}
}

/// @brief Read from a given IO port.
/// This can be an input or output port depending on what value you pass for "reg" (e.g., REG_GI0, REG_GO0).
///
/// @param address I2C address for a given Cypress chip.
/// @param reg Register to read from.
/// @param port Number of port to set [0-5].
/// @param r_byte_val_out Byte reference with the registry value (used as output).
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::ioReadPort(uint8_t address, uint8_t reg, uint8_t port, uint8_t &r_byte_val_out)
{
	if (port > 5)
		return -1;
	uint8_t resp = reg > REG_GO5 ? i2cWrite(address, REG_PORT_SEL, port) : 0; // specify port to set for reading non io registers
	if (resp == 0)
		resp = i2cRead(address, reg + port, &r_byte_val_out, 1);
	return resp;
}

/// @brief Writes to a given IO port.
/// Only applies to output ports. Only changes bit values specified by the "byte_mask" argument.
///
/// @param address I2C address for a given Cypress chip.
/// @param port Number of port to set [0-5].
/// @param byte_mask Byte value in which bits set to one denote the pin/bit to set in the register.
/// @param bit_val_set Value to set the bits to [0,1].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::ioWritePort(uint8_t address, uint8_t port, uint8_t byte_mask, uint8_t bit_val_set)
{
	if (port > 5)
		return -1;
	else
	{
		uint8_t port_byte;
		ioReadPort(address, REG_GO0, port, port_byte); // get current port values
		_updateRegByte(port_byte, byte_mask, bit_val_set);
		uint8_t resp = i2cWrite(address, REG_GO0 + port, port_byte); // update port
		return resp;
	}
}

/// @brief Reads from one or multiple sequential registers.
///
/// @param address I2C address for a given Cypress chip.
/// @param reg Register to read from.
/// @param p_byte_out_arr Byte pointer array with the registry values (used as output).
/// @param s Length of the "p_byte_out_arr" array [1-16].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::ioReadReg(uint8_t address, uint8_t reg, uint8_t p_byte_out_arr[], uint8_t s)
{
	if (s > 16)
		return -1;
	uint8_t resp = i2cRead(address, reg, p_byte_out_arr, s);
	return resp;
}

/// @brief Write to one or multiple sequential registers.
/// An option is included to provide the previous registry value in order to bypass the additional ioReadReg() step.
///
/// @param address I2C address for a given Cypress chip.
/// @param p_byte_mask_arr Byte value pointer array in which bits set to one denote the pin/bit to set in the registers.
/// @param s Length of the "p_byte_mask_arr" array [1-16].
/// @param bit_val_set Value to set the bits to [0,1].
/// @param p_reg_last_byte_arr OPTIONAL: pointer byte array of the previous registry. Should be same length as "s".
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::ioWriteReg(uint8_t address, uint8_t p_byte_mask_arr[], uint8_t s, uint8_t bit_val_set, uint8_t p_reg_last_byte_arr[])
{
	uint8_t resp = 0;
	if (s > 16)
		return -1;

	// Handle missing old registry values
	uint8_t p_byte_val[s]; // initialize array to handle null array argument
	if (p_reg_last_byte_arr == nullptr)
	{
		resp = ioReadReg(address, REG_GO0, p_byte_val, s); // get current registry values
		/// @bug: Just discovered a likely big bug here! Had this returning with successful reads!
		if (resp != 0)
			return resp;
	}
	else
	{ // copy over inputed old registry values
		for (size_t i = 0; i < s; i++)
			p_byte_val[i] = p_reg_last_byte_arr[i];
	}

	// Update registry values with new values
	for (size_t i = 0; i < s; i++)
	{
		_updateRegByte(p_byte_val[i], p_byte_mask_arr[i], bit_val_set);
	}
	resp = i2cWrite(address, REG_GO0, p_byte_val, s); // update register
	return resp;
}

//------------------------ HIGH-LEVEL METHODS ------------------------

/// @brief Checks the I2C status and sets up the Cypress chip for a new session by reinitializing the settings.
///
/// @param address I2C address for a given Cypress chip.
///
/// @return CStatus codes [-1: critical error or from @ref Wire::endTransmission()].
uint8_t CypressCom::setupCypress(uint8_t address)
{

	// Check I2C lines
	bool is_err = false;
#ifdef ARDUINO_SAM_DUE
	is_err = (digitalRead(20) == LOW) || (digitalRead(21) == LOW);
#endif
#ifdef __AVR_ATmega2560__
	is_err = (digitalRead(20) == LOW) || (digitalRead(21) == LOW);
#endif
#ifdef ARDUINO_AVR_UNO
	is_err = (digitalRead(PC4) == LOW) || (digitalRead(PC5) == LOW);
#endif
	if (is_err)
		_Dbg.printMsg(_Dbg.MT::ERROR, "I2C LINES LOW: CHECK POWER");

	// Test I2C connection
	_beginTransmissionWrapper(address);
	Wire.write((uint8_t)0);
	uint8_t resp = _endTransmissionWrapper();

	// Check for timeout
	/// @todo Get this working
	if (resp != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "FAILED SETUP I2C CHECK: WIRE STATUS[%d]", resp);
	}

	// Setup Cypress chip
	if (resp == 0)
	{
		// Restore chip
		resp = i2cWrite(address, REG_CMD, REG_CMD_RESTORE);
		if (resp)
			_Dbg.printMsg(_Dbg.MT::ERROR, "FAILED: CYPRESS CHIP RESTORE: WIRE STATUS[%d]", resp);

		// Reset chip
		resp = i2cWrite(address, REG_CMD, REG_CMD_RECONF);
		if (resp)
			_Dbg.printMsg(_Dbg.MT::ERROR, "FAILED: CYPRESS CHIP RECONFIGURE: WIRE STATUS[%d]", resp);
	}

	return resp;
}

/// @brief Sets up the different properties of the PWM source.
///
/// @param address I2C address for a given Cypress chip.
/// @param source Specifies one of 8 sources to set. See @ref WallOperation::wms.pwmSrc.
/// @param duty PWM duty cycle [0-255].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::setupSourcePWM(uint8_t address, uint8_t source, uint8_t duty)
{
	if (source > 7 || duty > 255)
		return -1;
	else
	{
		uint8_t resp = i2cWrite(address, REG_SEL_PWM, source); // specify pwm source to set
		if (resp == 0)
		{
			resp = i2cWrite(address, REG_CONF_PWM, pwmClockVal); // set hardware clock to 0 (32kHz)
			if (resp == 0)
			{
				resp = i2cWrite(address, REG_PERI_PWM, pwmPeriodVal); // set period to 32 (clock = 32kHz/32 = 1kHz)
				if (resp == 0)
				{
					resp = setSourceDutyPWM(address, source, duty); // set duty cycle to duty
				}
			}
		}
		return resp;
	}
}

/// @brief Option to change the PWM duty cycle for a given source.
/// Sets up the different properties of the PWM source.
///
/// @param address I2C address for a given Cypress chip.
/// @param source Specifies one of 8 sources to set. See @ref WallOperation::wms.pwmSrc.
/// @param duty PWM duty cycle [0-255].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::setSourceDutyPWM(uint8_t address, uint8_t source, uint8_t duty)
{
	if (source > 7 || duty > 255)
		return -1;
	uint8_t pulse_wd = (float(duty) / 255) * (float)pwmPeriodVal; // compute pulse width
	uint8_t resp = i2cWrite(address, REG_PW_PWM, pulse_wd);		  // set duty cycle to duty
	return resp;
}

/// @brief Set a given port register.
///
/// @param address I2C address for a given Cypress chip.
/// @param reg Register to read from.
/// @param port Number of port to set [0-5].
/// @param byte_mask Byte value in which bits set to one denote the pin/bit to set in the register.
/// @param bit_val_set Value to set the bits to [0,1].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::setPortRegister(uint8_t address, uint8_t reg, uint8_t port, uint8_t byte_mask, uint8_t bit_val_set)
{
	if (port > 5)
		return -1;
	uint8_t resp = i2cWrite(address, REG_PORT_SEL, port); // specify port to set
	if (resp == 0)
	{
		uint8_t port_byte;
		resp = i2cRead(address, reg, &port_byte, 1); // get port registry byte
		if (resp == 0)
		{
			_updateRegByte(port_byte, byte_mask, bit_val_set);
			resp = i2cWrite(address, reg, port_byte); // update register
		}
	}
	return resp;
}

/// @brief Wrapper for Wire::beginTransmission() to catch address value for debugging.
///
/// @param address I2C address for a given Cypress chip.
void CypressCom::_beginTransmissionWrapper(uint8_t address)
{
	ADDR = address;
	Wire.beginTransmission(address);
}

/// @brief Wrapper for Wire::endTransmission() to catch and print errors debugging.
///
/// @param send_stop Indicates whether or not a STOP should be performed on the bus [default=true].
/// @param print_err Indicates whether or not to print error if received [default=true].
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t CypressCom::_endTransmissionWrapper(bool send_stop, bool do_print_err)
{
	uint8_t resp = Wire.endTransmission(send_stop);
	if (resp != 0 && do_print_err)
		_Dbg.printMsg(_Dbg.MT::ERROR, "I2C Error[%d] Address[%s] from Wire::endTransmission()", resp, _Dbg.hexStr(ADDR));
	return resp;
}

//------------------------ TESTING AND DEBUGGING METHODS ------------------------

/// @brief Scans for I2C addresses and prints to Serial Output Window along with expected address.
///
/// @note This library was designed to support the Cypress CY8C9540A and should work for the CY8C9520A
///	but will not support the additional registries of the CY8C9560A.
///
/// @return Last address found
uint8_t CypressCom::i2cScan()
{
	uint8_t address;
	uint8_t resp;
	uint8_t cnt_addr = 0;
	uint8_t cnt_err = 0;
	uint8_t list_addr[128] = {0};
	uint8_t list_addr_with_err[128] = {0};
	uint8_t list__err_code[128] = {0};

	// Loop and test all 128 possible addresses
	for (address = 1; address < 127; address++)
	{
		// Track dt for errors
		_Dbg.dtTrack(1);

		// Test address
		_beginTransmissionWrapper(address);
		resp = _endTransmissionWrapper(true, false);

		// Handle response
		if (resp == 0)
		{ // check for repsonse
			list_addr[cnt_addr] = address;
			cnt_addr++;
		}
		else if (resp > 4)
		{ // catch unknown error
			list_addr_with_err[cnt_addr] = address;
			cnt_err++;

			// Print unknown error code immediately
			_Dbg.printMsg(_Dbg.MT::WARNING, "I2C Error[%d] Address[%s] DT[%s]", resp, _Dbg.hexStr(address), _Dbg.dtTrack());
		}
	}

	// Print results
	if (cnt_addr > 0)
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "I2C Devices Found:");
		for (size_t i = 0; i < cnt_addr; i++)
		{ // print devices
			_Dbg.printMsg(_Dbg.MT::INFO, "\t%d) %s", i, _Dbg.hexStr(list_addr[i]));
		}
	}
	else
	{
		_Dbg.printMsg(_Dbg.WARNING, "No I2C Devices Found");
	}
	if (cnt_err > 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "I2C Errors Found");
		for (size_t i = 0; i < cnt_addr; i++)
		{ // print errors
			_Dbg.printMsg(_Dbg.MT::INFO, "\t%d) %s", i, _Dbg.hexStr(list_addr_with_err[i]));
		}
	}

	// Return last address
	return list_addr[cnt_addr];
}

/// @brief Print a single registry byte in binary format.
///
/// @param byte_mask_in Byte value used as a mask with Cypress methods.
void CypressCom::printRegByte(uint8_t byte_mask_in)
{
	if (DB_VERBOSE == 0)
		return;

	uint8_t p_byte_mask_in[1] = {byte_mask_in};
	printRegByte(p_byte_mask_in, 1);
}
/// @overload: Option to print an array of registry bytes in binary format.
///
/// @param p_byte_mask_in Pointer to an array of byte values used as masks.
/// @param s Size of the byte array.
void CypressCom::printRegByte(uint8_t p_byte_mask_in[], uint8_t s)
{
	if (DB_VERBOSE == 0)
		return;

	_Dbg.printMsg(_Dbg.MT::INFO, "\tRegistry Bytes: ");
	for (size_t i = 0; i < s; i++)
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "\tport[%d]\n\t\t 76543210\n\t\t%s", i, _Dbg.binStr(p_byte_mask_in[i]));
	}
}