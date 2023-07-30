// ######################################

//========== Maze_Debug.cpp ============

// ######################################

/// <file>
/// Used for the Maze_Debug class
/// <file>

//============= INCLUDE ================
#include "Maze_Debug.h"

//===========CLASS: Maze_Debug============

/// <summary>
/// Constructor
/// </summary>
Maze_Debug::Maze_Debug() {}

/// <summary>
/// Print message using printf formatting without ellapsed time.
/// </summary>
/// <param name="p_fmt">Formating string comperable to sprintf().</param>
/// <param name="">a list of variables related to the formatting string.</param>
void Maze_Debug::printMsg(const char *p_fmt, ...)
{
	if (DB_VERBOSE == 0)
		return;
	// Handle input args
	va_list args;
	va_start(args, p_fmt);	// Start retrieving additional arguments
	_printMsg(p_fmt, args); // Pass arguments to _printMsg
	va_end(args);			// End retrieval
}

/// <summary>
/// Print message using printf formatting with ellapsed time.
/// </summary>
/// <param name="p_fmt">Formating string comperable to sprintf().</param>
/// <param name="">a list of variables related to the formatting string.</param>
void Maze_Debug::printMsgTime(const char *p_fmt, ...)
{
	if (DB_VERBOSE == 0)
		return;

	// Print ellapsed time
	Serial.print(_timeStr(0)); // print time string
	Serial.print(": ");		   // print time string

	// Handle input args
	va_list args;
	va_start(args, p_fmt);	// Start retrieving additional arguments
	_printMsg(p_fmt, args); // Pass arguments to _printMsg
	va_end(args);			// End retrieval
}

/// <summary>
/// Print message from @ref Maze_Debug::printMsg() with ellapsed time.
/// </summary>
/// <param name="p_fmt">auto passed from @ref Maze_Debug::printMsg().</param>
/// <param name="args">auto passed from @ref Maze_Debug::printMsg().</param>
void Maze_Debug::_printMsg(const char *p_fmt, va_list args)
{
	static const uint16_t buff_s = 250;
	static char buff[buff_s];
	buff[0] = '\0';

	vsnprintf(buff, buff_s, p_fmt, args); // format string from argument list
	Serial.println(buff);				  // print message
}

/// <summary>
/// Print message from printMsgTime() with ellapsed time.
/// </summary>
/// <param name="ts_ms">Current time (ms).</param>
/// <param name="t0_ms">Zero time if computing dt (ms).</param>
/// <returns>String in the form [MM:SS:MS]</returns>
char *Maze_Debug::_timeStr(uint32_t ts_0)
{
	static char buff[3][25]; // track up to 3 instances
	static uint8_t i;
	buff[i][0] = '\0';

	// get minutes, seconds and milliseconds
	uint32_t dt = millis() - ts_0;
	int dt_m = (dt - (dt % (60 * 1000))) / (60 * 1000);			 // minutes
	int dt_s = (dt - (dt_m * (60 * 1000)) - (dt % 1000)) / 1000; // seconds
	int dt_ms = dt - (dt_m * (60 * 1000)) - (dt_s * 1000);		 // milliseconds

	// Format string and print
	sprintf(buff[i], "%02u:%02u:%03u", dt_m, dt_s, dt_ms);
	uint8_t ii = i;
	i = i == 2 ? 0 : i + 1;
	return buff[ii];
}

/// <summary>
/// Store or retrieve a string for later use. Call without argument to retrieve string.
/// </summary>
/// <param name="p_str">OPTIONAL: string to store</param>
char *Maze_Debug::setGetStr(const char *p_str)
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

/// <summary>
/// Print array of values.
/// </summary>
/// <param name="p_arr">Array to print (max 10 elements).</param>
/// <param name="s">Size of array.</param>
/// <returns>Formatted array string. E.g., "[0,1,2]"</returns>
char* Maze_Debug::arrayStr(uint8_t p_arr[], size_t s)
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

/// <summary>
/// Prints byte in binary.
/// </summary>
/// <param name="b">Byte to print.</param>
/// <returns>Formatted byte string [B00000000].</returns>
char *Maze_Debug::binStr(uint8_t b)
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

/// <summary>
/// Prints value in hex.
/// </summary>
/// <param name="h">Hex value to print</param>
/// <returns>formatted hex string [e.g., 0xFF]</returns>
char *Maze_Debug::hexStr(uint8_t h)
{
	static char buff[20];
	buff[0] = '\0';
	sprintf(buff, "%p", h);
	return buff;
}

/// <summary>
/// Track elapsed time between calls. Call firts with
/// an input arg of 1 to set the clock. Subsiquent calls
/// will provide ellapsed time.
/// </summary>
/// <param name="do_reset">Reset clock [1].</param>
/// <returns>Formatted string [s:ms:us].</returns>
char *Maze_Debug::dtTrack(uint8_t do_reset)
{
	static unsigned long ts_0 = 0;
	if (do_reset == 1)
	{
		ts_0 = millis(); // store current time on first call
		return '\0';
	}
	else
	{ // get dt on subsiquent call
		return (_timeStr(ts_0));
	}
}

/// <summary>
/// Print indeces of ones in byte.
/// </summary>
/// <param name="byte_mask_in">Byte value used as a mask with Cypress methods.</param>
/// <returns>Formatted array string. E.g., "[0,1,2]"</returns>
char *Maze_Debug::bitIndStr(uint8_t byte_mask_in)
{
	if (DB_VERBOSE == 0)
		return '\0';
	static char buff1[20];
	buff1[0] = '\0';
	char buff2[10];
	if (byte_mask_in == 0)
		sprintf(buff1, "[]"); // no ones in byte
	else
	{
		strcat(buff1, "[");
		uint8_t c_cnt = 0;
		for (size_t i = 0; i < 8; i++)
		{
			if (bitRead(byte_mask_in, i) == 1)
			{
				sprintf(buff2, "%d", i);
				strcat(buff1, buff2);
				strcat(buff1, ",");
				c_cnt += 2;
			}
		}
		buff1[c_cnt] = ']';
	}
	return buff1;
}

/// <summary>
/// Print single registry byte in binary.
/// </summary>
/// <param name="byte_mask_in">Byte value used as a mask with Cypress methods.</param>
void Maze_Debug::printRegByte(uint8_t byte_mask_in)
{
	if (DB_VERBOSE == 0)
		return;
	uint8_t p_byte_mask_in[1] = {byte_mask_in};
	printRegByte(p_byte_mask_in, 1);
}
/// <summary>
/// Overloads to pass a byte arrays.
/// </summary>
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
/// <summary>
/// Overloads to pass a second byte array for comparisons.
/// </summary>
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
