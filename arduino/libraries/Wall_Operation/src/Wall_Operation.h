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

	/// @todo: need to have a way to track the argument length associated with each message type
	const char msg_type_str[7][20] = {
		"MSG_NONE",
		"CONFIRM_RECIEVED",
		"HANDSHAKE",
		"MOVE_WALLS",
		"START_SESSION",
		"END_SESSION",
		"ERROR"};
	enum MessageType
	{
		MSG_NONE = 0,
		CONFIRM_RECIEVED = 32,
		HANDSHAKE = 64,
		MOVE_WALLS = 2,
		START_SESSION = 128,
		END_SESSION = 129,
		ERROR = 254
	};
	MessageType rcvMsgTyp = MessageType::MSG_NONE; ///< tracks the received ethercat message type
	MessageType sndMsgTyp = MessageType::MSG_NONE; ///< tracks the sent ethercat message type
	MessageType cnfMsgTyp = MessageType::MSG_NONE; ///< tracks the conf ethercat message type
	int rcvMsgID = 0;							  ///< tracks the received ethercat message number
	int sndMsgID = 0;							  ///< tracks the sent ethercat message number
	enum ErrorType
	{
		ERROR_NONE = 0,
		MESSAGE_ID_DISORDERED = 1,
		NO_MESSAGE_TYPE_MATCH = 2,
		REGISTER_LEFTOVERS = 3,
		MISSING_FOOTER = 4
	};
	ErrorType rcvErrTyp = ErrorType::ERROR_NONE; ///< tracks the received ethercat error type

	union RegUnion
	{					  ///< union for storing ethercat 8 16-bit reg entries, shareable accross 16 and 16 8 bit data types
		byte ui8[16];	  ///< (byte) 1 byte
		uint16_t ui16[8]; ///< (uint16_t) 2 byte
		uint64_t ui64[2]; ///< (uint64_t) 8 byte
	};
	RegUnion U; ///< union for storing ethercat 8 16-bit reg entries

	/*
		const static uint8_t msg_queue_len = 10; ///< size of ethercat message buffer
		struct MessageHandler					 ///< struct for storing ethercat messages
		{
		public:
			uint8_t msgid = 0;							 ///< Ethercat message ID
			char msgtypstr[50];							 ///< Ethercat message type string
			MessageType msgtype = MessageType::MSG_NONE; ///< Ethercat message type
			ErrorType errtype = ErrorType::ERROR_NONE;	 ///< Ethercat message error
			uint16_t Reg16[8] = {0};					 ///< Ethercat 16 bit register values

			/// @todo: add way of storing message type of CONFIRM_RECEIVED

			// Indexes for RegUnion
			uint8_t U8i = 0;  // index for RegUnion.ui8[16]
			uint8_t U16i = 0; // index for RegUnion.ui16[8]

			// Shared by all instances of EcatRegStruct (static)
			static uint8_t LenQ;	  ///< set to msg_queue_len @todo: define this by @ref msg_queue_len in constructor
			static uint8_t IndQ;	  ///< index for ethercat receive message buffer
			static const int MsgDt;	  ///< delay between message send/write (ms)
			static uint32_t MsgTS;	  ///< last sent/recieve ts (ms)
			static MessageType MsgTp; ///< Ethercat message error @todo: set to MessageType::MSG_NONE in constructor
			static ErrorType ErrTp;	  ///< Ethercat message error @todo: set to ErrorType::ERROR_NONE in constructor
		private:
			static RegUnion _U; ///< union for storing ethercat 8 16-bit reg entries

			// Constructor
			MessageHandler(uint8_t _msgid, MessageType _msgtype, const char *_msgtypstr)
				: msgid(_msgid), msgtype(_msgtype)
			{
				strncpy(msgtypstr, _msgtypstr, sizeof(msgtypstr) - 1); // create message type string
				msgtypstr[sizeof(msgtypstr) - 1] = '\0';			   // ensure null-termination
			}

			// Default Constructor
			MessageHandler()
				: msgid(0), msgtype(MessageType::MSG_NONE)
			{
				msgtypstr[0] = '\0';
			}

			// Struct Methods: Setters

			void set_all(uint8_t _msgid, MessageType _msgtype, const char *_msgtypstr, uint16_t *_Reg16)
			{
				msgid = _msgid;
				msgtype = _msgtype;
				strncpy(msgtypstr, _msgtypstr, sizeof(msgtypstr) - 1); // create message type string
				msgtypstr[sizeof(msgtypstr) - 1] = '\0';			   // ensure null-termination
				for (int i = 0; i < 8; i++)							   // copy Reg16 array for 8 registers
					Reg16[i] = _Reg16[i];
			}
			void set_msgid(uint8_t _msgid)
			{
				msgid = _msgid;
			}
			void set_msg_type(MessageType _msgtype)
			{
				msgtype = _msgtype;
			}
			void set_typ_str(const char *_msgtypstr)
			{
				strncpy(msgtypstr, _msgtypstr, sizeof(msgtypstr) - 1); // create message type string
				msgtypstr[sizeof(msgtypstr) - 1] = '\0';			   // ensure null-termination
			}
			void set_int16(uint16_t *_Reg16)
			{
				for (int i = 0; i < 8; i++) // copy Reg16 array for 8 registers
					Reg16[i] = _Reg16[i];
			}

			// Struct Methods: Getters
			uint8_t get_id()
			{
				return msgid;
			}
			MessageType get_msg_mt()
			{
				return msgtype;
			}
			const char *get_msg_type_str()
			{
				return msgtypstr;
			}

			uint16_t *get_int16()
			{
				return Reg16;
			}

			// Struct Methods: Setters RegUnion

			// Create a method that will work with the U union to set the 8 16-bit registers, tracking both 16 and 8 bit index
			void set_ui16(int p_reg[])
			{
				// set full register
				for (int U16i = 0; U16i < 8; U16i++)
					_U.ui16[U16i] = p_reg[U16i];
			}
			// Store reg data in ui16[8] and and incriment ui16 and ui8 index
			void set_ui16(int reg_i16)
			{
				return;
			}
			// Store reg data in ui8[16] and and incriment ui16 and ui8 index
			void set_ui8(int reg_i18)
			{
				return;
			}
			// Store reg data in ui16[16] at index set_u16_i and update ui16 and ui8 index
			void set_ui16(int reg_i16, uint8_t set_u16_i)
			{
				return;
			}
			// Store reg data in ui8[16] at index set_u8_i and incriment ui16 and ui8 index
			void set_ui8(int reg_i8, uint8_t set_u8_i)
			{
				return;
			}

			/// @@todo: create a method that will work with the U union to set the 8 16-bit registers, tracking both 16 and 8 bit index

			/// @todo: @methods: Getter RegUnion

			/// @todo: @methods: DATA STORAGE

			/// 	@todo: methods for higher level storing of message data within instances of the EcatRegStruct arrau for sending and receiving from the ethercat master
			///			@methods: Store indevidual message parameters and data and track the index of the of
			///			@methods: Get data to be sent cycling through the
			void write_message_to_ecat()
			{

				// Exit if no message to send

				// Exit if < dt has not passed

				// Exit if < dt has not passed
			}

			/// 	@todo: Store reg data in message data

			/// 	QUESTION: would it make sense to just move the Esmacatshield instance into here and send data directly to the master from here?

			///		@todo: methods for storing data within the instances of EcatRegStruct for sending and receiving from the ethercat master

			/// @todo: @methods: Deubuggig

			/// 	@todo: Move to Maze_Debug::plotReg() method into here
		};
		RegUnion Wall_Operation::MessageHandler::U;
		uint8_t MessageHandler::LenQ = msg_queue_len;
		uint8_t MessageHandler::IndQ = 0;
		const int MessageHandler::MsgDt = 0; // You need to provide a value here, 0 is just an example
		uint32_t MessageHandler::MsgTS = 0;	 // Initialize as per your requirement
		MessageType MessageHandler::MsgTp = MessageType::MSG_NONE;
		ErrorType MessageHandler::ErrTp = ErrorType::ERROR_NONE;

		MessageHandler SndMH[msg_queue_len]; ///<  initialize array of MessageHandler structs for sending messages
		MessageHandler RcvMH[msg_queue_len]; ///<  initialize array of MessageHandler structs for receiving messages
		*/

private:
	Maze_Debug _DB;		///< local instance of Maze_Debug class
	Cypress_Com _C_COM; ///< local instance of Cypress_Com class
	Esmacatshield ESMA; //< instance of Esmacatshield class

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
	void sendEthercatMessage(MessageType, uint8_t[] = nullptr, uint8_t = 255);

public:
	uint8_t getEthercatMessage();

public:
	void executeEthercatCommand();

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
	void printEcat(uint8_t, int[] = nullptr);
	void printEcat(uint8_t, RegUnion);
};

#endif