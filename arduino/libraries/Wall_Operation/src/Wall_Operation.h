// ######################################

//========= Wall_Operation.h ===========

// ######################################
#ifndef _WALL_OPERATION_h
#define _WALL_OPERATION_h

//============= INCLUDE ================
#include "Arduino.h"
#include "Maze_Debug.h"
#include "Cypress_Com.h"
#include "Esmacat_Com.h"

/// @brief This class handles the actual opperation of the maze walls and Ethercat coms.
///
/// @remarks
/// This class uses an instance of the Maze_Debug and Cypress_Com classes.
/// This class also deals with the mapping of walls to associated Cypress pins.
/// This class also deals with incoming and outgoing Ethercat communication.
class Wall_Operation
{

	// --------------VARIABLES--------------
public:

    uint8_t nCham;			// number of chambers to initialize in maze
	uint8_t nChambMove;	// max number of chambers to move at once
	uint8_t nMoveAttempt;	// max number of attempts to move a walls
	uint8_t pwmDuty;		// pwm duty cycle 
	uint16_t dtMoveTimeout; // timeout for wall movement (ms)

	struct WallMapStruct // pin mapping organized by wall with entries corresponding to the associated port or pin
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
		uint8_t funMap[6][8] = {0}; // map of pin function [0:none, 1:io_down, 2:io_up, 3:pwm_down, 4:pwm_up]
	};
	WallMapStruct wms; // only one instance used

	struct PinMapStruct // pin mapping orgnanized by port / pin number
	{
		uint8_t nPortsInc;		// stores number of included ports in the arrays
		uint8_t nPinsInc[6];	// stores number of included pins in the arrays for each included port
		uint8_t portInc[6];		// stores included port numbers
		uint8_t pinInc[6][8];	// stores included pin numbers for each included port
		uint8_t wallInc[6][8];	// stores included wall numbers
		uint8_t byteMaskInc[6]; // stores registry mask byte for each included port
		uint8_t byteMaskAll[6]; // stores registry mask byte for all ports in registry
	};
	PinMapStruct pmsAllIO;	 // all io pins
	PinMapStruct pmsAllPWM;	 // all pwm pins
	PinMapStruct pmsUpIO;	 // io up pins
	PinMapStruct pmsDownIO;	 // io down pins
	PinMapStruct pmsUpPWM;	 // pwm up pins
	PinMapStruct pmsDownPWM; // pwm down pins

	struct ChamberStruct // struct for tracking each chamber
	{
		uint8_t addr = 0;			  // chamber I2C address
		uint8_t i2cStatus = 0;		  // track I2C status errors for chamber
		uint8_t runStatus = 0;		  // track run status for chamber [0:no move, 1:move success, 2:i2c error, 3:timeout, 4:unspecified]
		uint8_t bitWallPosition = 0;  // bitwise variable, current wall position [0:down, 1:up]
		uint8_t bitWallRaiseFlag = 0; // bitwise variable, flag current walls that should be raised/active [0:inactive, 1:active]
		uint8_t bitWallErrorFlag = 0; // bitwise variable, flag wall move errors [0:no error, 1:error]
		PinMapStruct pmsActvPWM;	  // reusable dynamic instance for active PWM
		PinMapStruct pmsActvIO;		  // reusable dynamic instance for active IO
	};
	ChamberStruct C[9]; // initialize with max number of chambers for 3x3

	Esmacat_Com EsmaCom; //< instance of Esmacatshield class

private:
	Maze_Debug _Dbg;	 // local instance of Maze_Debug class
	Cypress_Com _CypCom; // local instance of Cypress_Com class

	// ---------------METHODS---------------

public:
	Wall_Operation(uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);

public:
	void procEcatMessage();

private:
	void _makePMS(PinMapStruct &, uint8_t[], uint8_t[]);
	void _makePMS(PinMapStruct &, uint8_t[], uint8_t[], uint8_t[], uint8_t[]);

private:
	void _addPortPMS(PinMapStruct &, uint8_t[], uint8_t[]);

private:
	void _addPinPMS(PinMapStruct &, uint8_t[], uint8_t[]);

private:
	void _sortArr(uint8_t[], size_t s, uint8_t[] = nullptr);

private:
	void _resetPMS(PinMapStruct &);

private:
	void _updateDynamicPMS(PinMapStruct, PinMapStruct &, uint8_t);

public:
	void initSoftware(uint8_t, uint8_t[] = nullptr);

public:
	uint8_t initCypress();

public:
	uint8_t initWalls(uint8_t);

private:
	uint8_t _setupCypressIO(uint8_t);

private:
	uint8_t _setupCypressPWM(uint8_t);

public:
	uint8_t setWallMove(uint8_t, uint8_t);
	uint8_t setWallMove(uint8_t, uint8_t, uint8_t[], uint8_t);
	uint8_t setWallMove(uint8_t, uint8_t, uint8_t);

private:
	uint8_t _setWallMove(uint8_t, uint8_t, uint8_t);

public:
	uint8_t moveWallsStaged();

public:
	uint8_t moveWalls();
	uint8_t moveWalls(uint8_t[], uint8_t);

private:
	uint8_t _moveWalls(uint8_t[], uint8_t);

private:
	uint8_t _moveConductor(uint8_t[], uint8_t, uint8_t);

private:
	uint8_t _moveStart(uint8_t);

private:
	uint8_t _moveCheck(uint8_t);

public:
	uint8_t changeWallDutyPWM(uint8_t, uint8_t, uint8_t);

public:
	uint8_t getWallState(uint8_t, uint8_t, uint8_t &);

public:
	uint8_t testWallIO(uint8_t, uint8_t[] = nullptr, uint8_t = 8);

public:
	uint8_t testWallPWM(uint8_t, uint8_t[] = nullptr, uint8_t = 8, uint16_t = 500);

public:
	uint8_t testWallOperation(uint8_t, uint8_t[] = nullptr, uint8_t = 8);

private:
	void _printPMS(PinMapStruct);
};

#endif