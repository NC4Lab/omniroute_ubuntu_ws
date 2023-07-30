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

/// <summary>
/// This class handles the actual opperation of the maze walls and Ethercat coms.
/// </summary>
/// <remarks>
/// This class uses an instance of the Maze_Debug and Cypress_Com classes.
/// This class also deals with the mapping of walls to associated Cypress pins.
/// This class also deals with incoming and outgoing Ethercat communication.
/// </remarks>
class Wall_Operation
{

	// ---------VARIABLES-----------------
public:
	// -----------MAZE VARS-----------------
	uint8_t nCham;	 ///< number of chambers [1-9]
	uint8_t pwmDuty; ///< pwm duty cycle [0-255]

	// -----------CYPRESS VARS-----------------
	struct Wall_Map_Str ///< pin mapping organized by wall with entries corresponding to the associated port or pin
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
	Wall_Map_Str wms; ///< only one instance used

	struct PinMapStr ///< pin mapping orgnanized by port / pin number
	{
		uint8_t port[6];		///< stores port numbers
		uint8_t pin[6][8];		///< stores pin numbers
		uint8_t wall[6][8];		///< stores wall numbers
		uint8_t bitMask[6];		///< stores registry mask byte for each used port
		uint8_t bitMaskLong[6]; ///< stores registry mask byte for all ports in registry
		uint8_t nPorts;			///< stores number of ports in list
		uint8_t nPins[6];		///< stores number of pins in list
	};
	PinMapStr pmsAllIO;	  ///< all io pins
	PinMapStr pmsAllPWM;  ///< all pwm pins
	PinMapStr pmsUpIO;	  ///< io up pins
	PinMapStr pmsDownIO;  ///< io down pins
	PinMapStr pmsUpPWM;	  ///< pwm up pins
	PinMapStr pmsDownPWM; ///< pwm down pins

	struct ChamberStruct ///< struct for tracking each chamber
	{
		uint8_t num = 0;			   ///< chamber number
		uint8_t addr = 0;			   ///< chamber I2C address
		uint8_t bitWallMoveFlag = 0;   ///< bitwise variable, current wall active flag [0:inactive, 1:active]
		uint8_t bitWallPosition = 0;   ///< bitwise variable, current wall position [0:down, 1:up]
		uint8_t bitWallErrorFlag = 0;  ///< bitwise variable, flag move errors for a given wall
		uint8_t bitWallUpdateFlag = 0; ///< bitwise variable, flag that wall position should be updated
		uint8_t bitOutRegLast[6];	   ///< stores output registry values
		PinMapStr pmsDynPWM;		   ///< reusable dynamic instance for active PWM
		PinMapStr pmsDynIO;			   ///< reusable dynamic instance for active IO
	};
	ChamberStruct C[9]; ///< initialize with max number of chambers for 3x3

	// ---------ETHERCAT COMMS-----------------
	uint8_t isHandshakeDone = false; ///< flag to track setup handshake of ethercat coms

	union ComUnion
	{					  ///< union for storing ethercat 8 16-bit reg entries, shareable accross 16 and 16 8 bit data types
		byte ui8[16];	  ///< (byte) 1 byte
		uint16_t ui16[8]; ///< (uint16_t) 2 byte
		uint64_t ui64[2]; ///< (uint64_t) 8 byte
	};
	ComUnion U;

	const char msgTypeStr[7][20] = {
		"MSG_NONE",
		"CONFIRM_RECIEVED",
		"HANDSHAKE",
		"MOVE_WALLS",
		"START_SESSION",
		"END_SESSION",
		"ERROR"};
	enum MsgType
	{
		MSG_NONE = 0,
		CONFIRM_RECIEVED = 32,
		HANDSHAKE = 64,
		MOVE_WALLS = 2,
		START_SESSION = 128,
		END_SESSION = 129,
		ERROR = 254
	};
	MsgType p2aMsgType = MsgType::MSG_NONE;
	MsgType a2pMsgType = MsgType::MSG_NONE;
	int p2aMsgID = 0; ///< tracks the received ethercat message number
	int a2pMsgID = 0; ///< tracks the sent ethercat message number

	enum ErrorType
	{
		ERROR_NONE = 0,
		MESSAGE_ID_DISORDERED = 1,
		NO_MESSAGE_TYPE_MATCH = 2,
		REGISTER_LEFTOVERS = 3,
		MISSING_FOOTER = 4
	};
	ErrorType p2aErrType = ErrorType::ERROR_NONE;

	struct EtherBuffStruct ///< struct for storing ethercat messages
	{
		uint8_t id = 0;						///< Ethercat message ID
		uint16_t reg16[8] = {0};			///< Ethercat 16 bit register values
		MsgType msgTyp = MsgType::MSG_NONE; ///< Ethercat message type
		char msg_typ_str[50];				///< Ethercat message type string

		// Constructor
		EtherBuffStruct(uint8_t id, MsgType msgTyp, const char *msg)
			: id(id), msgTyp(msgTyp)
		{
			strncpy(msg_typ_str, msg, sizeof(msg_typ_str) - 1); // create message type string
			msg_typ_str[sizeof(msg_typ_str) - 1] = '\0';		// ensure null-termination
		}

		// Default Constructor
		EtherBuffStruct()
			: id(0), msgTyp(MsgType::MSG_NONE)
		{
			msg_typ_str[0] = '\0';
		}

		// Member function
		void setVars(uint8_t newId)
		{
			id = newId;
		}
	};
	const static uint8_t etherBufLen = 10;	   ///< size of ethercat message buffer
	EtherBuffStruct a2pEtherBuff[etherBufLen]; ///<  initialize max 10 messages

private:
	Maze_Debug _DB;		  ///< local instance of Maze_Debug class
	Cypress_Com _C_COM;	  ///< local instance of Cypress_Com class
	Esmacatshield ESlave; //< instance of Esmacatshield class

	// -----------METHODS-----------------

public:
	Wall_Operation(uint8_t, uint8_t, uint8_t = 1);

private:
	void _makePMS(PinMapStr &, uint8_t[], uint8_t[]);
	void _makePMS(PinMapStr &, uint8_t[], uint8_t[], uint8_t[], uint8_t[]);

private:
	void _addPortPMS(PinMapStr &, uint8_t[], uint8_t[]);

private:
	void _addPinPMS(PinMapStr &, uint8_t[], uint8_t[]);

private:
	void _sortArr(uint8_t[], size_t s, uint8_t[] = nullptr);

private:
	void _resetPMS(PinMapStr &);

private:
	void _updateDynamicPMS(PinMapStr, PinMapStr &, uint8_t);

public:
	void resetMaze(uint8_t);

private:
	uint8_t _setupCypressIO();

private:
	uint8_t _setupCypressPWM(uint8_t);

private:
	uint8_t _setupWalls();

public:
	void sendEthercatMessage(MsgType, uint8_t[] = nullptr, uint8_t = 0);

public:
	uint8_t getEthercatMessage();

public:
	void executeEthercatCommand();

public:
	void handleHandshake();

public:
	uint8_t setWallCmdManual(uint8_t, uint8_t, uint8_t[] = nullptr, uint8_t = 8);

public:
	uint8_t changeWallDutyPWM(uint8_t, uint8_t, uint8_t);

public:
	uint8_t moveWalls(uint32_t = 1000);

public:
	uint8_t forceStopWalls();

public:
	uint8_t testWallIO(uint8_t, uint8_t[] = nullptr, uint8_t = 8);

public:
	uint8_t testWallPWM(uint8_t, uint8_t[] = nullptr, uint8_t = 8, uint32_t = 500);

public:
	uint8_t testWallOperation(uint8_t, uint8_t[] = nullptr, uint8_t = 8);

public:
	void printPMS(PinMapStr);

public:
	void printEtherReg(uint8_t, int[] = nullptr);
	void printEtherReg(uint8_t, ComUnion);
};

#endif