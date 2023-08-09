// ######################################

//=========== Maze_Debug.h =============

// ######################################

/// <file>
/// Used for the Maze_Debug class
/// <file>

//============= INCLUDE ================
#include "Arduino.h"
#include "Esmacat_Com.h"

#ifndef _MAZE_DEBUG_h
#define _MAZE_DEBUG_h

extern bool DB_VERBOSE; ///< set this variable in your INO file to control debugging [0:silent, 1:verbose]

/// <summary>
/// Used for printing different types of information to the Serial Output Window.
/// </summary>
/// <remarks>
/// This class is used in both the Cypress_Comm and Wall_Operation classes.
/// </remarks>
class Maze_Debug
{
	// -----------METHODS-----------------
public:
	Maze_Debug();

public:
	void printMsg(const char *, ...);

public:
	void printMsgTime(const char *, ...);

private:
	void _printMsg(const char *, va_list);

private:
	char *_timeStr(uint32_t);

public:
	char *setGetStr(const char* = nullptr);

public:
	char *arrayStr(uint8_t[], size_t);

public:
	char *binStr(uint8_t);

public:
	char *hexStr(uint8_t);

public:
	char *dtTrack(uint8_t = 0);

public:
	char *bitIndStr(uint8_t);

public:
	void printRegByte(uint8_t);

public:
	void printRegByte(uint8_t[], uint8_t);

public:
	void printRegByte(uint8_t[], uint8_t[], uint8_t);

};

#endif
