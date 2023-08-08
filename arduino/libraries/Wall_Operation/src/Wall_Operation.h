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
	// -----------GENERAL-----------------
	uint8_t nCham;	 ///< number of chambers [1-9]
	uint8_t pwmDuty; ///< pwm duty cycle [0-255]

	// -----------CYPRESS-----------------
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
	WallMapStruct wms; ///< only one instance used

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
	PinMapStruct pmsAllIO;	 ///< all io pins
	PinMapStruct pmsAllPWM;	 ///< all pwm pins
	PinMapStruct pmsUpIO;	 ///< io up pins
	PinMapStruct pmsDownIO;	 ///< io down pins
	PinMapStruct pmsUpPWM;	 ///< pwm up pins
	PinMapStruct pmsDownPWM; ///< pwm down pins

	struct ChamberStruct ///< struct for tracking each chamber
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
	ChamberStruct C[9]; ///< initialize with max number of chambers for 3x3

	// ---------ETHERCAT COMMS-----------------
	uint8_t isHandshakeDone = false; ///< flag to track setup handshake of ethercat coms

	const char message_type_str[8][30] = {
		"MSG_NONE",
		"CONFIRM_DONE",
		"HANDSHAKE",
		"MOVE_WALLS",
		"START_SESSION",
		"END_SESSION",
		"ERROR"};
	enum MessageType
	{
		MSG_NONE = 0,
		CONFIRM_DONE = 1,
		HANDSHAKE = 2,
		MOVE_WALLS = 3,
		START_SESSION = 4,
		END_SESSION = 5,
		ERROR = 6,
		N_MessageType
	};
	const char error_type_str[5][30] = {
		"ERROR_NONE",
		"MESSAGE_ID_DISORDERED",
		"NO_MESSAGE_TYPE_MATCH",
		"REGISTER_LEFTOVERS",
		"MISSING_FOOTER"};
	enum ErrorType
	{
		ERROR_NONE = 0,
		MESSAGE_ID_DISORDERED = 1,
		NO_MESSAGE_TYPE_MATCH = 2,
		REGISTER_LEFTOVERS = 3,
		MISSING_FOOTER = 4
	};

	union RegUnion
	{					  ///< union for storing ethercat 8 16-bit reg entries, shareable accross 16 and 16 8 bit data types
		byte ui8[16];	  ///< (byte) 1 byte
		uint16_t ui16[8]; ///< (uint16_t) 2 byte
		uint64_t ui64[2]; ///< (uint64_t) 8 byte
	};
	RegUnion U; ///< union for storing ethercat 8 16-bit reg entries

	struct EcatMessageStruct ///< class for handeling ethercat messages
	{
		int msgDt = 10;							   ///< delay between message send/write (ms)
		uint16_t msgID = 0;							   ///< Ethercat message ID
		MessageType msgTp = MessageType::MSG_NONE; ///< Ethercat message error
		ErrorType errTp = ErrorType::ERROR_NONE;   ///< Ethercat message error
		char msg_tp_str[50] = {0};				   ///< Ethercat message type string
		char err_tp_str[50] = {0};				   ///< Ethercat error type string
		uint8_t msg_tp_val = 0;					   ///< Ethercat message type value
		uint8_t msg_arg_lng = 0;				   ///< Ethercat message argument length in bytes
		uint8_t Arg8[10] = {0};					   ///< Ethercat message arguments (8 bit)
		RegUnion RegU;							   ///< Union for storing ethercat 8 16-bit reg entries
		uint8_t u8i = 0;						   ///< index for RegUnion.ui8[16]
		uint8_t u16i = 0;						   ///< index for RegUnion.ui16[8]
		bool isDone = false;					   ///< flag for message exicution completion
	};
	EcatMessageStruct sndEM; ///<  initialize message handler instance for sending messages
	EcatMessageStruct rcvEM; ///<  initialize message handler instance for receiving messages
	EcatMessageStruct tmpEM; ///<  initialize message handler instance for temporary receiving messages

private:
	Maze_Debug _DB;		///< local instance of Maze_Debug class
	Cypress_Com _C_COM; ///< local instance of Cypress_Com class
	Esmacatshield ESMA; //< instance of Esmacatshield class

	// -----------METHODS-----------------

public:
	Wall_Operation(uint8_t, uint8_t, uint8_t = 1);

private:
	void _resetU(EcatMessageStruct &);

private:
	void _seti8(EcatMessageStruct &, uint8_t);

private:
	void _seti16(EcatMessageStruct &, uint16_t);

private:
	uint8_t _geti8(EcatMessageStruct &);

private:
	uint16_t _geti16(EcatMessageStruct &);

private:
	void _printEcatReg(uint8_t, int[] = nullptr);
	void _printEcatReg(uint8_t, RegUnion);

private:
	bool _setupMsgStruct(EcatMessageStruct &, uint16_t, MessageType);
	bool _setupMsgStruct(EcatMessageStruct &, uint16_t, uint8_t);

private:
	void _checkErr(EcatMessageStruct &, ErrorType, bool);

public:
	void sendEthercatMessage(MessageType, uint8_t[] = nullptr, uint8_t = 0);

public:
	uint8_t getEthercatMessage();

public:
	void executeEthercatMessage();

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
	void resetMaze(uint8_t);

private:
	uint8_t _setupCypressIO();

private:
	uint8_t _setupCypressPWM(uint8_t);

private:
	uint8_t _setupWalls();

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
	void printPMS(PinMapStruct);
};

#endif