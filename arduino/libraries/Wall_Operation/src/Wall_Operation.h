// ######################################

//========= Wall_Operation.h ===========

// ######################################

/// <file>
/// Used for the Wall_Operation class
/// <file>

#ifndef _WALL_OPERATION_h
#define _WALL_OPERATION_h

//============= INCLUDE ================
#include "Arduino.h"
#include "Maze_Debug.h"
#include "Cypress_Com.h"
#include "Esmacatshield.h"

// TEMP COMMIT TEST

/// <summary>
/// This class handles the actual opperation of the maze walls.
/// </summary>
/// <remarks>
/// This class uses an instance of the Maze_Debug and Cypress_Com classes.
/// This class also deals with the mapping of walls to associated Cypress pins.
/// </remarks>
class Wall_Operation
{

	// ---------VARIABLES-----------------
public:
	uint8_t nCham;		 ///< number of chambers
	struct WallMapStruct ///< pin mapping organized by wall with entries corresponding to the associated port or pin
	{
		uint8_t pwmSrc[8] =
			{4, 6, 7, 5, 3, 1, 0, 2};
		uint8_t ioDown[2][8] = {
			{4, 1, 0, 0, 3, 3, 5, 4}, // port
			{3, 3, 1, 7, 2, 4, 0, 7}  // pin/bit
		};
		uint8_t ioUp[2][8] = {
			{4, 1, 0, 3, 3, 3, 4, 4}, // port
			{0, 2, 2, 0, 3, 5, 5, 4}  // pin/bit
		};
		uint8_t pwmDown[2][8] = {
			{1, 1, 0, 3, 5, 5, 5, 4}, // port
			{1, 0, 0, 1, 2, 3, 1, 2}  // pin/bit
		};
		uint8_t pwmUp[2][8] = {
			{4, 1, 0, 0, 3, 3, 2, 4}, // port
			{1, 4, 4, 5, 6, 7, 2, 6}  // pin/bit
		};
		uint8_t funMap[6][8] = {0}; ///< map of pin function [0:none, 1:io_down, 2:io_up, 3:pwm_down, 4:pwm_up]
	};
	WallMapStruct wms;	///< only one instance used
	struct PinMapStruct ///< pin mapping orgnanized by port / pin number
	{
		uint8_t port[6];		///< stores port numbers
		uint8_t pin[6][8];		///< stores pin numbers
		uint8_t wall[6][8];		///< stores wall numbers
		uint8_t bitMask[6];		///< stores registry mask byte for each used port
		uint8_t bitMaskLong[6]; ///< stores registry mask byte for all ports in registry
		uint8_t nPorts;			///< stores number of ports in list
		uint8_t nPins[6];		///< stores number of pins in list
	};
	PinMapStruct pmsAllIO;	  ///< all io pins
	PinMapStruct pmsAllPWM;	  ///< all pwm pins
	PinMapStruct pmsUpIO;	  ///< io up pins
	PinMapStruct pmsDownIO;	  ///< io down pins
	PinMapStruct pmsUpPWM;	  ///< pwm up pins
	PinMapStruct pmsDownPWM;  ///< pwm down pins
	struct ChamberTrackStruct ///< struct for tracking each chamber
	{
		uint8_t num = 0;			   ///< chamber number
		uint8_t addr = 0;			   ///< chamber I2C address
		uint8_t bitWallMoveFlag = 0;   ///< bitwise variable, current wall active flag [0:inactive, 1:active]
		uint8_t bitWallPosition = 0;   ///< bitwise variable, current wall position [0:down, 1:up]
		uint8_t bitWallErrorFlag = 0;  ///< bitwise variable, flag move errors for a given wall
		uint8_t bitWallUpdateFlag = 0; ///< bitwise variable, flag that wall position should be updated
		uint8_t bitOutRegLast[6];	   ///< stores output registry values
		PinMapStruct pmsDynPWM;		   ///< reusable dynamic instance for active PWM
		PinMapStruct pmsDynIO;		   ///< reusable dynamic instance for active IO
	};
	ChamberTrackStruct C[9]; ///< initialize with max number of chambers for 3x3
	union Union
	{					 ///< union for storing ethercat data shareable accross data types
		byte b[2];		 ///<(byte) 1 byte
		uint16_t i16[1]; ///<(uint16_t) 2 byte
	};
	Union U;
	uint8_t isEthercatInitialized = false; ///< flag to track setup/initialization of ethercat coms
	uint8_t isMazeReset = false;		   ///< flag to track if maze system has been reset
	int etherMsgNumID = 0;				   ///< tracks the message ethercat message number
	enum Py2ArdMsgTypeID
	{
		PY2ARD_NONE = 0,
		INITIALIZE_COMMS = 128,
		MOVE_WALLS = 1,
		RESET_SYSTEM = 2,
		ERROR = 254,
		JUNK = 255
	};
	Py2ArdMsgTypeID py2ardMsgTypeID = Py2ArdMsgTypeID::PY2ARD_NONE;
	enum Ard2PyMsgTypeID
	{
		ARD2PY_NONE = 0
	};
	Ard2PyMsgTypeID ard2PyMsgTypeID = Ard2PyMsgTypeID::ARD2PY_NONE;
	enum RunErrorType
	{
		ERROR_NONE = 0,
		MESSAGE_ID_DISORDERED = 1,
		MISSING_FOOTER = 2
	};
	RunErrorType runErrorTypeEnum = RunErrorType::ERROR_NONE;

private:
	Maze_Debug _DB;		  ///< local instance of Maze_Debug class
	Cypress_Com _C_COM;	  ///< local instance of Cypress_Com class
	Esmacatshield ESlave; //<instance of Esmacatshield class

	// -----------METHODS-----------------
public:
	void resetWallFlags();

public:
	Wall_Operation(uint8_t, uint8_t = 1);

private:
	void _makePMS(PinMapStruct &, uint8_t[], uint8_t[]);

private:
	void _makePMS(PinMapStruct &, uint8_t[], uint8_t[], uint8_t[], uint8_t[]);

private:
	void _addPortPMS(PinMapStruct &, uint8_t[], uint8_t[]);

private:
	void _addPinPMS(PinMapStruct &, uint8_t[], uint8_t[]);

private:
	void _sortArr(uint8_t[], size_t s, uint8_t[] = nullptr);

public:
	uint8_t changeWallDutyPWM(uint8_t, uint8_t, uint8_t);

public:
	uint8_t setupWallIO();

public:
	uint8_t setupWallPWM(uint8_t);

public:
	uint8_t initializeWalls();

private:
	void _resetPMS(PinMapStruct &);

private:
	void _updateDynamicPMS(PinMapStruct, PinMapStruct &, uint8_t);

public:
	uint8_t checkEthercatComms();

public:
	uint8_t setWallCmdManual(uint8_t, uint8_t, uint8_t[] = nullptr, uint8_t = 8);

public:
	uint8_t runWalls(uint32_t = 1000);

public:
	uint8_t forceStopWalls();

public:
	uint8_t testWallIO(uint8_t, uint8_t[] = nullptr, uint8_t = 8);

public:
	uint8_t testWallPWM(uint8_t, uint8_t[] = nullptr, uint8_t = 8, uint32_t = 500);

public:
	uint8_t testWallOperation(uint8_t, uint8_t[] = nullptr, uint8_t = 8);

private:
	void _printPMS(PinMapStruct);
};

#endif