// ######################################

//========== Maze_Debug.cpp ============

// ######################################

/// <file>
/// Used for the Maze_Debug class
/// <file>

//============= INCLUDE ================
#include "Maze_Debug.h"

//===========CLASS: Maze_Debug============

/// @brief Constructor
Maze_Debug::Maze_Debug() {}

/// @brief Print a message using printf formatting with elapsed time.
///
/// @param p_fmt Message string with formatting comparable to sprintf().
/// @param ... Variable arguments related to the formatting string.
void Maze_Debug::printMsg(const char *p_fmt, ...)
{
	if (DB_VERBOSE == 0)
		return;

	// Handle input args
	va_list args;
	va_start(args, p_fmt);			  // Start retrieving additional arguments
	_printMsg(MT::INFO, p_fmt, args); // Pass arguments to _printMsg with default message type
	va_end(args);
}
/// @overload: Option for including message type
///
/// @param msg_type_enum Enum specifying message type.
void Maze_Debug::printMsg(MT msg_type_enum, const char *p_fmt, ...)
{
	if (DB_VERBOSE == 0)
		return;

	// Handle input args
	va_list args;
	va_start(args, p_fmt);				   // Start retrieving additional arguments
	_printMsg(msg_type_enum, p_fmt, args); // Pass arguments to _printMsg
	va_end(args);
}

/// @brief Print a message with elapsed time.
///
/// @param msg_type_enum Enum specifying message type.
/// @param p_fmt Message string with formatting passed from Maze_Debug::printMsg().
/// @param args Variable arguments list.
void Maze_Debug::_printMsg(MT msg_type_enum, const char *p_fmt, va_list args)
{
	const uint8_t buff_s = 125;
	static char buff[buff_s];
	buff[0] = '\0';
	const uint8_t buff_sym_s = 31;
	static char buff_sym[buff_sym_s];
	buff_sym[0] = '\0';

	// Check if message type is an attention grabbing header message
	bool is_head1_msg = (msg_type_enum == MT::HEAD1 || msg_type_enum == MT::HEAD1A || msg_type_enum == MT::HEAD1B);
	bool is_head2_msg = (msg_type_enum == MT::HEAD2);

	// Format message
	vsnprintf(buff, buff_s, p_fmt, args);

	// Make header of '=' characters based on message length
	size_t n = buff_sym_s - 1 - strlen(buff) / 2;
	n = n < sizeof(buff_sym) && n < buff_sym_s ? n : 3; // ensure n is not negative and doesn't exceed buff_sym size

	// Fill buffer with '=' characters
	if (is_head1_msg)
		memset(buff_sym, '=', n);
	else if (is_head2_msg)
		memset(buff_sym, '_', n);
	else if (msg_type_enum == MT::ERROR)
		memset(buff_sym, '!', n);
	buff_sym[n] = '\0';

	// Add additional new line for attention messages
	if (msg_type_enum == MT::HEAD1 || msg_type_enum == MT::HEAD1A)
		Serial.print("\n");

	// Print message type
	Serial.print(_message_type_str[msg_type_enum]);

	// Print time string
	Serial.print(" [");
	Serial.print(_timeStr(0));
	Serial.print("]: ");

	// Print preceding header symbols
	if (is_head1_msg || is_head2_msg || msg_type_enum == MT::ERROR)
		Serial.print(buff_sym);

	// Print message
	Serial.print(buff);

	// Print proceding header symbols
	if (is_head1_msg || is_head2_msg || msg_type_enum == MT::ERROR)
		Serial.print(buff_sym);

	// Add new line
	Serial.print("\n");

	// Add additional new line for attention messages
	if (msg_type_enum == MT::HEAD1 || msg_type_enum == MT::HEAD1B)
		Serial.print("\n");
}

/// @brief Generate a time string based on current run time.
///
/// @param ts_0 Reference time (ms).
/// @return Formatted time string in the form [MM:SS:MS].
const char *Maze_Debug::_timeStr(uint32_t ts_0)
{
	static char buff[2][15]; // track up to 2 instances
	static uint8_t i;
	buff[i][0] = '\0';

	// get minutes, seconds and milliseconds
	uint32_t dt = millis() - ts_0;
	uint32_t dt_m = (dt - (dt % (60UL * 1000UL))) / (60UL * 1000UL);		  // minutes
	uint32_t dt_s = (dt - (dt_m * (60UL * 1000UL)) - (dt % 1000UL)) / 1000UL; // seconds
	uint32_t dt_ms = dt - (dt_m * (60UL * 1000UL)) - (dt_s * 1000UL);		  // milliseconds

	// Format string and print
	snprintf(buff[i], sizeof(buff[i]), "%02lu:%02lu:%03lu", dt_m, dt_s, dt_ms);
	uint8_t ii = i;
	i = i == 1 ? 0 : i + 1;
	return buff[ii];
}

/// @brief Convert an array of values to a string representation.
///
/// @param p_arr Array to convert.
/// @param s Size of the array.
///
/// @return Formatted string representing the array.
const char *Maze_Debug::arrayStr(uint8_t p_arr[], size_t s)
{
	if (DB_VERBOSE == 0)
		return "";
	if (s > 10)
		return "";

	static char buff1[20];
	buff1[0] = '\0';
	static char buff2[5];
	buff2[0] = '\0';

	strncat(buff1, "[", sizeof(buff1) - strlen(buff1) - 1);
	for (size_t i = 0; i < s; i++)
	{
		snprintf(buff2, sizeof(buff2), "%d", p_arr[i]);
		strncat(buff1, buff2, sizeof(buff1) - strlen(buff1) - 1);
		if (i < s - 1)
			strncat(buff1, ",", sizeof(buff1) - strlen(buff1) - 1);
		else
			strncat(buff1, "]", sizeof(buff1) - strlen(buff1) - 1);
	}
	return buff1;
}

/// @brief Generate a binary representation of a byte.
///
/// @param b Byte to be converted to its binary representation.
///
/// @return Formatted byte string in the form [B00000000].
const char *Maze_Debug::binStr(uint8_t b)
{
	if (DB_VERBOSE == 0)
		return "";

	static char buff[12];
	buff[0] = '\0';

	snprintf(buff, sizeof(buff), "B%d%d%d%d%d%d%d%d", bitRead(b, 7), bitRead(b, 6), bitRead(b, 5), bitRead(b, 4), bitRead(b, 3), bitRead(b, 2), bitRead(b, 1), bitRead(b, 0));

	return buff;
}

/// @brief Generate a hexadecimal representation of a value.
///
/// @param h Value to be converted to its hexadecimal representation.
///
/// @return Formatted hex string, e.g., [0xFF].
const char *Maze_Debug::hexStr(uint8_t h)
{
	static char buff[10];
	buff[0] = '\0';

	snprintf(buff, sizeof(buff), "0x%02X", h);
	return buff;
}

/// @brief Track and return the elapsed time between calls.
/// Initially call with an input argument of 1 to set the clock. Subsequent calls provide elapsed time.
///
/// @param do_reset If set to 1, resets the clock.
///
/// @return Formatted time string in the form [s:ms:us].
const char *Maze_Debug::dtTrack(uint8_t do_reset)
{
	static unsigned long ts_0 = 0;
	if (do_reset == 1)
	{
		ts_0 = millis(); // store current time on first call
		return "";
	}
	else
	{ // get dt on subsiquent call
		return (_timeStr(ts_0));
	}
}

/// @brief Generate a string that lists the indices of bits set to one in a byte.
///
/// @param byte_mask_in Byte value used as a mask.
///
/// @return Formatted array string, e.g., "[0,1,2]".
const char *Maze_Debug::bitIndStr(uint8_t byte_mask_in)
{
	if (DB_VERBOSE == 0)
		return "";

	static char buff1[20];
	buff1[0] = '\0';
	static char buff2[5];
	buff2[0] = '\0';

	if (byte_mask_in == 0)
	{
		snprintf(buff1, sizeof(buff1), "[]"); // no ones in byte
	}
	else
	{
		strncat(buff1, "[", sizeof(buff1) - strlen(buff1) - 1);
		for (size_t i = 0; i < 8; i++)
		{
			if (bitRead(byte_mask_in, i) == 1)
			{
				snprintf(buff2, sizeof(buff2), "%d,", i); // add comma right here
				strncat(buff1, buff2, sizeof(buff1) - strlen(buff1) - 1);
			}
		}
		// Remove the last comma and append closing bracket
		size_t len = strlen(buff1);
		if (len > 1 && buff1[len - 1] == ',')
		{
			buff1[len - 1] = ']';
			buff1[len] = '\0';
		}
		else
		{
			strncat(buff1, "]", sizeof(buff1) - strlen(buff1) - 1);
		}
	}
	return buff1;
}

/// @brief do print test
void Maze_Debug::printTest()
{
	// Print each message type
	printMsg(MT::INFO, "INFO");
	printMsg(MT::HEAD1, "HEAD1");
	printMsg(MT::DEBUG, "DEBUG");
	printMsg(MT::ERROR, "ERROR");
	printMsg(MT::WARNING, "WARNING");
	printMsg(MT::HEAD1A, "HEAD1A");
	printMsg(MT::INFO, "INFO");
	printMsg(MT::HEAD1B, "HEAD1B");
	printMsg(MT::INFO, "INFO");
	printMsg(MT::HEAD2, "HEAD2");
	printMsg(MT::INFO, "INFO");
}