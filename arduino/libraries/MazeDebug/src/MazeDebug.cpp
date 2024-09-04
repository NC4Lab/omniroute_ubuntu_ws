// ######################################

//========== MazeDebug.cpp ============

// ######################################

/// <file>
/// Used for the MazeDebug class
/// <file>

//============= INCLUDE ================
#include "MazeDebug.h"

//===========CLASS: MazeDebug============

/// @brief Constructor
MazeDebug::MazeDebug() {}

/// @brief Print a message with elapsed time.
///
/// @param msg_type_enum Enum specifying message type.
/// @param p_fmt Message string with formatting comparable to sprintf().
/// @param ... Variable arguments related to the formatting string.
void MazeDebug::printMsg(MT msg_type_enum, const char *p_fmt, ...)
{
	if (DB_VERBOSE == 0)
		return;

	// Local variables
	const uint8_t buff_s = 125;
	static char buff[buff_s];
	buff[0] = '\0';

	// Format message
	va_list args;
	va_start(args, p_fmt); // Start retrieving additional arguments
	vsnprintf(buff, buff_s, p_fmt, args);
	va_end(args);

	// Add additional new line for attention messages
	if (msg_type_enum == MT::ATTN)
		Serial.print("\n");

	// Print message type based on the enum value
	switch (msg_type_enum)
	{
	case MT::ATTN:
		Serial.print("[ATTN]");
		break;
	case MT::INFO:
		Serial.print("[INFO]");
		break;
	case MT::ERROR:
		Serial.print("[!!ERROR!!]");
		break;
	case MT::WARNING:
		Serial.print("[!WARNING!]");
		break;
	case MT::DEBUG:
		Serial.print("[DEBUG]");
		break;
	default:
		Serial.print("[UNKNOWN]");
		break;
	}

	// Print time string
	Serial.print(" [");
	Serial.print(_timeStr(0));
	Serial.print("]: ");

	// Print message
	Serial.print(buff);

	// Add new line
	Serial.print("\r\n");

	// Add additional new line for attention messages
	if (msg_type_enum == MT::ATTN)
		Serial.print("\r\n");
}

/// @brief Generate a time string based on current run time.
///
/// @param ts_0 Reference time (ms).
/// @return Formatted time string in the form [MM:SS:MS].
const char *MazeDebug::_timeStr(uint32_t ts_0)
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
const char *MazeDebug::arrayStr(uint8_t p_arr[], size_t s)
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
const char *MazeDebug::binStr(uint8_t b)
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
const char *MazeDebug::hexStr(uint8_t h)
{
	static char buff[10];
	buff[0] = '\0';

	snprintf(buff, sizeof(buff), "0x%02X", h);
	return buff;
}

/// @brief Track and return the elapsed time between calls.
/// Initially call with an input argument of 1 to set the clock. Subsequent calls provide elapsed time.
///
/// @param do_reset If true, resets the clock. DEFAULT: false.
///
/// @return Formatted time string in the form [s:ms:us].
const char *MazeDebug::dtTrack(bool do_reset)
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
const char *MazeDebug::bitIndStr(uint8_t byte_mask_in)
{
	if (DB_VERBOSE == 0)
		return "";

	static char buff1[2][20]; // track up to 2 instances
	static uint8_t i;
	buff1[i][0] = '\0';
	static char buff2[5];
	buff2[0] = '\0';

	if (byte_mask_in == 0)
	{
		snprintf(buff1[i], sizeof(buff1[i]), "[]"); // no ones in byte
	}
	else
	{
		strncat(buff1[i], "[", sizeof(buff1[i]) - strlen(buff1[i]) - 1);
		for (size_t j = 0; j < 8; j++)
		{
			if (bitRead(byte_mask_in, j) == 1)
			{
				snprintf(buff2, sizeof(buff2), "%d,", j); // add comma right here
				strncat(buff1[i], buff2, sizeof(buff1[i]) - strlen(buff1[i]) - 1);
			}
		}
		// Remove the last comma and append closing bracket
		size_t len = strlen(buff1[i]);
		if (len > 1 && buff1[i][len - 1] == ',')
		{
			buff1[i][len - 1] = ']';
			buff1[i][len] = '\0';
		}
		else
		{
			strncat(buff1[i], "]", sizeof(buff1[i]) - strlen(buff1[i]) - 1);
		}
	}
	uint8_t ii = i;
	i = i == 1 ? 0 : i + 1;
	return buff1[ii];
}

/// @brief do print test
void MazeDebug::printTest()
{
	/// @note: Keep commmented out unless using because const strings take up memory

	printMsg(MT::INFO, "INFO");
	printMsg(MT::ATTN, "ATTN");
	printMsg(MT::DEBUG, "DEBUG");
	printMsg(MT::ERROR, "ERROR");
	printMsg(MT::WARNING, "WARNING");
}