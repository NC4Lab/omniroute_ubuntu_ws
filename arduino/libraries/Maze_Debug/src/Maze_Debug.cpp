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
	static const uint16_t buff_s = 200;
	static char buff[buff_s];
	buff[0] = '\0';

	// Format message
	if (msg_type_enum == MT::ATTN || msg_type_enum == MT::ATTN_START || msg_type_enum == MT::ATTN_END)
	{
		char buff_msg[buff_s];
		char buff_sym[50];

		// Format temp message
		vsnprintf(buff_msg, buff_s, p_fmt, args); // format string from argument list

		// Make header of '=' characters based on message length
        int n = 30 - strlen(buff_msg) / 2;
        n = n <= 0 ? 3 : n;

        // Fill buffer with '=' characters
        memset(buff_sym, '=', n);
        buff_sym[n] = '\0';

		// Format complete message
		sprintf(buff, "%s %s %s", buff_sym, buff_msg, buff_sym);
	}
	else
		vsnprintf(buff, buff_s, p_fmt, args); 

	// Add additional new line for attention messages
	if (msg_type_enum == MT::ATTN || msg_type_enum == MT::ATTN_START)
		Serial.print("\n");

	// Print message type
	Serial.print(_message_type_str[msg_type_enum]);

	// Print time string
	Serial.print(_timeStr(0));

	// Print message
	Serial.print(buff);

	// Add new line
	Serial.print("\n");

	// Add additional new line for attention messages
	if (msg_type_enum == MT::ATTN || msg_type_enum == MT::ATTN_END)
		Serial.print("\n");
}

/// @brief Generate a time string based on current run time.
///
/// @param ts_0 Reference time (ms).
/// @return Formatted time string in the form [MM:SS:MS].
const char *Maze_Debug::_timeStr(uint32_t ts_0)
{
	static char buff[3][25]; // track up to 3 instances
	static uint8_t i;
	buff[i][0] = '\0';

	// get minutes, seconds and milliseconds
	uint32_t dt = millis() - ts_0;
	uint32_t dt_m = (dt - (dt % (60UL * 1000UL))) / (60UL * 1000UL);		  // minutes
	uint32_t dt_s = (dt - (dt_m * (60UL * 1000UL)) - (dt % 1000UL)) / 1000UL; // seconds
	uint32_t dt_ms = dt - (dt_m * (60UL * 1000UL)) - (dt_s * 1000UL);		  // milliseconds

	// Format string and print
	sprintf(buff[i], "%02lu:%02lu:%03lu", dt_m, dt_s, dt_ms);
	uint8_t ii = i;
	i = i == 2 ? 0 : i + 1;
	return buff[ii];
}

/// @brief Store or retrieve a string for later use.
///
/// @param p_str (OPTIONAL) String to store.
/// @return Previously stored string or the newly stored string.
const char *Maze_Debug::setGetStr(const char *p_str)
{
	static char buff[100];

	// Store string
	if (p_str != nullptr)
	{
		buff[0] = '\0';
		strncpy(buff, p_str, sizeof(buff) - 1);
		buff[sizeof(buff) - 1] = '\0';
	}
	return buff;
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
	static char buff1[50];
	buff1[0] = '\0';
	char buff2[10];
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
		return "B00000000";
	static char buff[10][10]; // track up to 10 instances
	static uint8_t i;
	buff[i][0] = '\0';
	sprintf(buff[i], "B%d%d%d%d%d%d%d%d", bitRead(b, 7), bitRead(b, 6), bitRead(b, 5), bitRead(b, 4), bitRead(b, 3), bitRead(b, 2), bitRead(b, 1), bitRead(b, 0));
	uint8_t ii = i;
	i = i == 9 ? 0 : i + 1;
	return buff[ii];
}

/// @brief Generate a hexadecimal representation of a value.
///
/// @param h Value to be converted to its hexadecimal representation.
///
/// @return Formatted hex string, e.g., [0xFF].
const char *Maze_Debug::hexStr(uint8_t h)
{
	static char buff[20];
	buff[0] = '\0';
	sprintf(buff, "0x%02X", h);
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
	char buff2[10];
	if (byte_mask_in == 0)
	{
		sprintf(buff1, "[]"); // no ones in byte
	}
	else
	{
		strcat(buff1, "[");
		for (size_t i = 0; i < 8; i++)
		{
			if (bitRead(byte_mask_in, i) == 1)
			{
				sprintf(buff2, "%d,", i); // add comma right here
				strcat(buff1, buff2);
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
			strcat(buff1, "]");
		}
	}
	return buff1;
}

/// @brief Print a single registry byte in binary format.
///
/// @param byte_mask_in Byte value used as a mask with Cypress methods.
void Maze_Debug::printRegByte(uint8_t byte_mask_in)
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
void Maze_Debug::printRegByte(uint8_t p_byte_mask_in[], uint8_t s)
{
	if (DB_VERBOSE == 0)
		return;
	char buff[250];
	Serial.println("\tRegistry Bytes: ");
	for (size_t i = 0; i < s; i++)
	{
		sprintf(buff, "\tport[%d]\n\t\t 76543210\n\t\t%s", i, binStr(p_byte_mask_in[i]));
		Serial.println(buff);
	}
}
/// @overload: Option to print and compare two arrays of registry bytes in binary format.
///
/// @param p_byte_mask_in_1 Pointer to the first array of byte values used as masks.
/// @param p_byte_mask_in_2 Pointer to the second array of byte values used as masks.
/// @param s Size of the byte arrays.
void Maze_Debug::printRegByte(uint8_t p_byte_mask_in_1[], uint8_t p_byte_mask_in_2[], uint8_t s)
{
	if (DB_VERBOSE == 0)
		return;
	char buff[250];
	Serial.println("\tRegistry Bytes: ");
	for (size_t i = 0; i < s; i++)
	{
		sprintf(buff, "\tport[%d]\n\t\t 76543210\n1)\t\t%s\n2)\t\t%s", i, binStr(p_byte_mask_in_1[i]), binStr(p_byte_mask_in_2[i]));
		Serial.println(buff);
	}
}
