// ######################################

//=========== Maze_Debug.h =============

// ######################################

/// @file Used for the Maze_Debug class

//============= INCLUDE ================
#include "Arduino.h"
#include "Esmacat_Com.h"

#ifndef _MAZE_DEBUG_h
#define _MAZE_DEBUG_h

extern bool DB_VERBOSE; ///< set this variable in your INO file to control debugging [0:silent, 1:verbose]

/// @brief Used for printing different types of information to the Serial Output Window.
///
/// @remarks This class is used in both the Cypress_Comm and Wall_Operation classes.
class Maze_Debug
{

	// ---------------VARIABLES---------------
public:
	const char _message_type_str[8][15] = {
		"[INFO]",
		"[INFO]"
		"[INFO]",
		"[INFO]",
		"[INFO]",
		"[ERROR]",
		"[WARNING]",
		"[DEBUG]"};

	enum MT
	{
		HEAD1 = 0,
		HEAD1A = 1,
		HEAD1B = 2,
		HEAD2 = 3,
		INFO = 4,
		ERROR = 5,
		WARNING = 6,
		DEBUG = 7
	};

	// ---------------METHODS---------------
public:
	Maze_Debug();

public:
	void printMsg(const char *, ...);
	void printMsg(MT, const char *, ...);

private:
	void _printMsg(MT, const char *, va_list);

private:
	const char *_timeStr(uint32_t);

public:
	const char *arrayStr(uint8_t[], size_t);

public:
	const char *binStr(uint8_t);

public:
	const char *hexStr(uint8_t);

public:
	const char *dtTrack(uint8_t = 0);

public:
	const char *bitIndStr(uint8_t);

public:
	void printTest();
};

#endif
