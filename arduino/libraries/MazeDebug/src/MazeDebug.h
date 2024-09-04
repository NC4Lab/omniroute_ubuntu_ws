// ######################################

//=========== MazeDebug.h =============

// ######################################

/// @file Used for the MazeDebug class

//============= INCLUDE ================
#include "Arduino.h"
#include "EsmacatCom.h"

#ifndef _MAZE_DEBUG_h
#define _MAZE_DEBUG_h

extern bool DB_VERBOSE; ///< set this variable in your INO file to control debugging [0:silent, 1:verbose]

/// @brief Used for printing different types of information to the Serial Output Window.
///
/// @remarks This class is used in both the CypressComm and WallOperation classes.
class MazeDebug
{

	// ---------------VARIABLES---------------
public:

	enum MT
	{
		ATTN = 0,
		INFO = 1,
		ERROR = 2,
		WARNING = 3,
		DEBUG = 4
	};

	// ---------------METHODS---------------
public:
	MazeDebug();

public:
	void printMsg(MT, const char *, ...);

private:
	const char *_timeStr(uint32_t);

public:
	const char *arrayStr(uint8_t[], size_t);

public:
	const char *binStr(uint8_t);

public:
	const char *hexStr(uint8_t);

public:
	const char *dtTrack(bool = false);

public:
	const char *bitIndStr(uint8_t);

public:
	void printTest();
};

#endif
