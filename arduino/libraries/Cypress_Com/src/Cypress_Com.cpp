// ######################################

//========= Cypress_Com.cpp ===========

// ######################################

/// <file>
/// Used for the Cypress_Com class
/// <file>

//============= INCLUDE ================
#include "Cypress_Com.h"
#include "Cypress_Com_Base.h"

//========CLASS: Cypress_Com==========

/// <summary>
/// Constructor
/// </summary>
Cypress_Com::Cypress_Com() {}

//+++++++++ Low-level Methods +++++++++++

/// <summary>
/// Lowest level funtion to read from a given Cypress register.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="reg">Register to read from.</param>
/// <param name="p_byte_out_arr">Byte array from the register (used as output).</param>
/// <param name="s">Length of p_byte_out_arr [1-16].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::i2cRead(uint8_t address, uint8_t reg, uint8_t p_byte_out_arr[], uint8_t s)
{
	if (s > 16)
		return -1;
	Wire.beginTransmission(address);
	Wire.write(reg);
	uint8_t resp = wireEndTransmissionWrapper(false); // master stops sending but keeps the transmission line open
	if (!resp)
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
	{
		return resp;
	}
}

/// <summary>
/// Lowest level funtion to write to a given Cypress register.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="reg">Register to read from.</param>
/// <param name="byte_val_in">Value to set the registry pin/bit to [0,1].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::i2cWrite(uint8_t address, uint8_t reg, uint8_t byte_val_in)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(byte_val_in);
	return wireEndTransmissionWrapper();
}
/// <summary>
/// Overload option to pass an array of bytes "array "p_byte_val_in_arr" to set multiple
/// registers begining at register specified by "reg".
/// </summary>
/// <param name="p_byte_val_in_arr">Byte pointer array with the registry values.</param>
/// <param name="s">Length of the "p_byte_val_in_arr" array [1-16]</param>
uint8_t Cypress_Com::i2cWrite(uint8_t address, uint8_t reg, uint8_t p_byte_val_in_arr[], uint8_t s)
{
	if (s > 16)
		return -1;
	Wire.beginTransmission(address);
	Wire.write(reg);
	for (size_t i = 0; i < s; i++)
	{
		Wire.write(p_byte_val_in_arr[i]);
	}
	return wireEndTransmissionWrapper();
}

/// <summary>
/// Updates a given byte value based on a given mask.
/// Note, this is used by several of the write methods to update the byte values
/// read in from a register based on the provided byte mask before they are updated
/// in the register.
/// </summary>
/// <param name="r_byte_val_out">Reference to the byte value to change (used as output)</param>
/// <param name="byte_mask">Byte value in which bits set to one denote the bit to set in "r_byte_val_out".</param>
/// <param name="bit_val_set">Value to set the bits to [0,1].</param>
void Cypress_Com::_updateRegByte(uint8_t &r_byte_val_out, uint8_t byte_mask, uint8_t bit_val_set)
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

//+++++++++ Mid-level Methods +++++++++++

/// <summary>
/// Read from a given IO pin associated with a given limit switch.
/// Note, this is only setup for input, not output, registers.
/// Note, see, for example, @ref Wall_Operation::wms.ioUp for wall to port mapping.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="port">Number of port to set [0-5].</param>
/// <param name="pin">Pin number to read [0,7] </param>
/// <param name="r_bit_val_out">Reference with the registry bit value (used as output).</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::ioReadPin(uint8_t address, uint8_t port, uint8_t pin, uint8_t &r_bit_val_out)
{
	if (port > 5 || pin > 7)
		return -1;
	else
	{
		uint8_t byte_val_out_arr[1];
		uint8_t resp = i2cRead(address, REG_GI0 + port, byte_val_out_arr, 1);
		delay(1); // hack to deal with strange resp variable behavior
		if (!resp)
		{
			r_bit_val_out = bitRead(byte_val_out_arr[0], pin);
		}
		else
			return resp;
	}
}

/// <summary>
/// Read from a given IO pin associated with a given limit switch.
/// Note, see, for example, @ref Wall_Operation::wms.pwmUp for wall to port mapping.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="port">Number of port to set [0-5]</param>
/// <param name="pin">Pin number to read [0,7] </param>
/// <param name="bit_val_set">Value to set the bits to [0,1].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::ioWritePin(uint8_t address, uint8_t port, uint8_t pin, uint8_t bit_val_set)
{
	if (port > 5 || pin > 7)
		return -1;
	else
	{
		uint8_t byte_val_out_arr[1];
		uint8_t resp1 = i2cRead(address, REG_GO0 + port, byte_val_out_arr, 1); // get current port registry value
		if (!resp1)
		{
			uint8_t byte_val_in = byte_val_out_arr[0];
			bitWrite(byte_val_in, pin, bit_val_set);						// set pin specific bit of uint8_t
			uint8_t resp2 = i2cWrite(address, REG_GO0 + port, byte_val_in); // update register
			return resp2;
		}
		else
			return resp1;
	}
}

/// <summary>
/// Read from a given IO port.
/// Note, this can be an input or output port depending on what value you pass for "reg" (e.g., REG_GI0, REG_GO0).
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="reg">Register to read from.</param>
/// <param name="port">Number of port to set [0-5].</param>
/// <param name="r_byte_val_out">Byte reference with the registry value (used as output).</param></param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::ioReadPort(uint8_t address, uint8_t reg, uint8_t port, uint8_t &r_byte_val_out)
{
	if (port > 5)
		return -1;
	uint8_t resp1 = reg > REG_GO5 ? i2cWrite(address, REG_PORT_SEL, port) : 0; // specify port to set for reading non io registers
	if (!resp1)
	{
		uint8_t resp2 = i2cRead(address, reg + port, &r_byte_val_out, 1);
		if (resp2)
			return resp2;
	}
	return resp1;
}

/// <summary>
/// Writes to a given IO port.
/// Note, only applies to ouptut ports.
/// Note, only changes bit values specied by the "byte_mask" argument.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="port">Number of port to set [0-5].</param>
/// <param name="byte_mask">Byte value in which bits set to one denote the pin/bit to set in the register.</param>
/// <param name="bit_val_set">Value to set the bits to [0,1].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::ioWritePort(uint8_t address, uint8_t port, uint8_t byte_mask, uint8_t bit_val_set)
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

/// <summary>
/// Reads from one or multiple sequential registers.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="reg">Register to read from.</param>
/// <param name="p_byte_out_arr">Byte pointer array with the registry values (used as output).</param>
/// <param name="s">Length of the "p_byte_out_arr" array [1-16]</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::ioReadReg(uint8_t address, uint8_t reg, uint8_t p_byte_out_arr[], uint8_t s)
{
	if (s > 16)
		return -1;
	uint8_t resp = i2cRead(address, reg, p_byte_out_arr, s);
	return resp;
}

/// <summary>
/// Write to one or multiple sequential registers.
/// Note, an option is included to provide the previous registry value in order to bypass the additional ioReadReg() step
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="p_byte_mask_arr">Byte value pointer array in which bits set to one denote the pin/bit to set in the registers.</param>
/// <param name="s">Length of the "p_byte_mask_arr" array [1-16]</param>
/// <param name="bit_val_set">Value to set the bits to [0,1].</param>
/// <param name="p_reg_last_byte_arr">OPTIONAL: pointer byte array of the previous registry. Should be same s as "s".</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::ioWriteReg(uint8_t address, uint8_t p_byte_mask_arr[], uint8_t s, uint8_t bit_val_set, uint8_t p_reg_last_byte_arr[])
{
	if (s > 16)
		return -1;

	// Handle missing old registry values
	uint8_t p_byte_val[s]; // initialize array to handle null array argument
	if (p_reg_last_byte_arr == nullptr)
	{
		uint8_t resp = ioReadReg(address, REG_GO0, p_byte_val, s); // get current registry values
		if (resp)
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
	uint8_t resp = i2cWrite(address, REG_GO0, p_byte_val, s); // update register
	return resp;
}

//+++++++++ High-level Methods +++++++++++

/// <summary>
/// Scans for I2C addresses and prints to Serial Output Window along with
/// espected address (TO BE ADDED).
/// Note, this library was designed to support the Cypress CY8C9540A
/// and should work for the CY8C9520A but will not support the additional
/// registries of the CY8C9560A
/// </summary>
/// /// <returns>Last address found.</returns>
uint8_t Cypress_Com::i2cScan()
{
	uint8_t addr;
	uint8_t err;
	uint8_t cnt_addr = 0;
	uint8_t cnt_err = 0;
	uint8_t list_addr[128] = {0};
	uint8_t list_err[128] = {0};

	// Loop and test all 128 possible addresses
	for (addr = 1; addr < 127; addr++)
	{
		Wire.beginTransmission(addr);
		err = wireEndTransmissionWrapper(true, false);
		if (err == 0)
		{ // check for repsonse
			list_addr[cnt_addr] = addr;
			cnt_addr++;
		}
		else if (err == 4)
		{ // catch unknown error
			list_err[cnt_addr] = addr;
			cnt_err++;
		}
	}

	// Print results
	if (cnt_addr > 0)
	{
		_DB.printMsg("I2C devices found");
		for (size_t i = 0; i < cnt_addr; i++)
		{ // print devices
			_DB.printMsg("%d)\t%s", i + 1, _DB.hexStr(list_addr[i]));
		}
	}
	else
	{
		_DB.printMsg("No I2C devices found");
	}
	if (cnt_err > 0)
	{
		_DB.printMsg("!!I2C errors found!!");
		for (size_t i = 0; i < cnt_addr; i++)
		{ // print errors
			_DB.printMsg("%d)\t%s", i + 1, _DB.hexStr(list_err[i]));
		}
	}

	// Return last address
	return list_addr[cnt_addr];
}

/// <summary>
/// Checks the I2C status and sets up the Cypress chip for a new session
/// by reinitializing the settings.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <returns>Wire::method output.</returns>
uint8_t Cypress_Com::setupCypress(uint8_t address)
{
	uint8_t resp;

	// Check I2C lines
#ifdef ARDUINO_SAM_DUE
	if ((digitalRead(20) == LOW) || (digitalRead(21) == LOW))
	{
		_DB.printMsg("!!i2c lines LOW!!");
	}
#endif
#ifdef __AVR_ATmega2560__
	if ((digitalRead(20) == LOW) || (digitalRead(21) == LOW))
	{
		_DB.printMsg("!!i2c lines LOW!!");
	}
#endif
#ifdef ARDUINO_AVR_UNO
	if ((digitalRead(PC4) == LOW) || (digitalRead(PC5) == LOW))
	{
		_DB.printMsg("!!i2c lines LOW!!");
	}
#endif

	// Test i2c
	Wire.beginTransmission(address);
	Wire.write((uint8_t)0);
	resp = wireEndTransmissionWrapper();
	if (resp != 0)
	{
		_DB.printMsg("!!i2c failed!!");
		return resp;
	}

	// Restore chip
	resp = i2cWrite(address, REG_CMD, REG_CMD_RESTORE);
	if (resp != 0)
	{
		_DB.printMsg("!!chip restore failed!!");
		return resp;
	}

	// Reset chip
	resp = i2cWrite(address, REG_CMD, REG_CMD_RECONF);
	if (resp != 0)
	{
		_DB.printMsg("!!chip reconfigure failed!!");
		return resp;
	}

	return resp;
}

/// <summary>
/// Sets up the different properties of the PWM source.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="source">Specifies one of 8 sources to set. See @ref Wall_Operation::wms.pwmSrc.</param>
/// <param name="duty">PWM duty cycle [0-255].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::setupSourcePWM(uint8_t address, uint8_t source, uint8_t duty)
{
	if (source > 7 || duty > 255)
		return -1;
	else
	{
		uint8_t resp1 = i2cWrite(address, REG_SEL_PWM, source); // specify pwm source to set
		if (!resp1)
		{
			uint8_t resp2 = i2cWrite(address, REG_CONF_PWM, pwmClockVal); // set hardware clock to 0 (32kHz)
			if (!resp2)
			{
				uint8_t resp3 = i2cWrite(address, REG_PERI_PWM, pwmPeriodVal); // set period to 32 (clock = 32kHz/32 = 1kHz)
				if (!resp3)
				{
					uint8_t resp4 = setSourceDutyPWM(address, source, duty); // set duty cycle to duty
					return resp4;
				}
				else
					return resp3;
			}
			else
				return resp2;
		}
		else
			return resp1;
	}
}

/// <summary>
/// Option to change the PWM duty cycle for a given source Sets up the different properties of the PWM source.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="source">Specifies one of 8 sources to set. See @ref Wall_Operation::wms.pwmSrc.</param>
/// <param name="duty">PWM duty cycle [0-255].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::setSourceDutyPWM(uint8_t address, uint8_t source, uint8_t duty)
{
	if (source > 7 || duty > 255)
		return -1;
	uint8_t pulse_wd = (float(duty) / 255) * (float)pwmPeriodVal; // compute pulse width
	uint8_t resp = i2cWrite(address, REG_PW_PWM, pulse_wd);		  // set duty cycle to duty
	return resp;
}

/// <summary>
/// Set a given port register.
/// </summary>
/// <param name="address">I2C address for a given Cypress chip.</param>
/// <param name="reg">Register to read from.</param>
/// <param name="port">Number of port to set [0-5].</param>
/// <param name="byte_mask">Byte value in which bits set to one denote the pin/bit to set in the register.</param>
/// <param name="bit_val_set">Value to set the bits to [0,1].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Cypress_Com::setPortRegister(uint8_t address, uint8_t reg, uint8_t port, uint8_t byte_mask, uint8_t bit_val_set)
{
	if (port > 5)
		return -1;
	uint8_t resp1 = i2cWrite(address, REG_PORT_SEL, port); // specify port to set
	if (!resp1)
	{
		uint8_t port_byte;
		uint8_t resp2 = i2cRead(address, reg, &port_byte, 1); // get port registry byte
		if (!resp2)
		{
			_updateRegByte(port_byte, byte_mask, bit_val_set);
			uint8_t resp3 = i2cWrite(address, reg, port_byte); // update register
			return resp3;
		}
		else
			return resp2;
	}
	else
		return resp1;
}

//+++++++ Testing and Debugging Methods ++++++++

/// <summary>
/// Wrapper for Wire::endTransmission() to catch and print errors.
/// </summary>
/// <param name="send_stop">Parameter indicating whether or not a STOP should be performed on the bus [default=true].</param>
/// /// <param name="print_err">Parameter indicating whether or not to print error if recieved [default=true].</param>
/// <returns>Output from Wire::method call.</returns>
uint8_t Cypress_Com::wireEndTransmissionWrapper(bool send_stop, bool print_err)
{
	uint8_t resp = Wire.endTransmission(send_stop);
	if (resp != 0 && print_err)
	{
		_DB.printMsgTime("!!I2C Error: %d!!", resp);
	}
	return resp;
}
