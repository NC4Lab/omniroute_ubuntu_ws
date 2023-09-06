// ######################################

//=========== Esmacat_Com.h ============

// ######################################
#ifndef ESMACAT_COM_H
#define ESMACAT_COM_H

//============= INCLUDE ================
#include "Arduino.h"
#include "Maze_Debug.h"
#include "Cypress_Com.h"
#include "SPI.h"

//============= GLOBALS ================
#define READ_REG 0B00000000
#define WRITE_REG 0B10000000
#define SINGLE_SHOT 0B10111111
#define LED_ON 0B00000100
#define LED_OFF 0B11111011

extern bool DO_ECAT_SPI; // set this variable in your INO file to control block SPI [0:dont start, 1:start]

/// @brief This class deals with Ethercat communication between a Python master and the Arduino slave.
///
/// @remarks: Several methods were addapted from the Esmacatshield library to save space.
/// @ref "https://bitbucket.org/harmonicbionics/ease_arduinocode/"
///
/// @details: The Ethercat register a total of 8 16-bit entries to work with for each message.
/// Some components of the message use the for 16-bit registry entry and some seperate it
/// into 2 8-bit entries using the Union data type (@see: RegUnion).
///
/// @details: Message Structure Overview:
/// Incoming/outgoing Ethercat register message entries overview:
///
///	    INT 0:
///         i16[0]: Message ID [0-65535] (unique message id)
///	    INT 1:
///         i8[0]: Message Type [0-255] (@see: MessageType)
///	        i8[1]: Argument Length [0-10] (number of message args in bytes)
///	    INT NA or 2-N:
///         i16[NA or 2-6]: 16 Bit Data [0-65535]
///		    i8[NA or 4-13]: 8 Bit Data [0-255]
///	    INT N+1
///		    i8[0]: Footer [254]
///		    i8[1]: Footer [254]
///
/// @details: Incoming/Outgoing RegUnion message structure:
///
///		ui16[0], ui8[0][1]		// ui16 [msg id]
///		ui16[1], ui8[2][3]		// ui8  [msg type]      [err type]
///		ui16[2], ui8[4][5]		// ui8  [arg length]    [arg 0]
///		ui16[3], ui8[6][7]		// ui8  [arg 1]         [arg 2]
///     ui16[4], ui8[8][9]     	// ui8  [arg 3]         [arg 4]
///     ui16[5], ui8[10][11]   	// ui8  [arg 5]         [arg 6]
///     ui16[6], ui8[12][13]  	// ui8  [arg 7]         [arg 8]
///     ui16[7], ui8[14][15]  	// ui8  [footer]        [footer]
///
/// @details: Incoming/Outgoing RegUnion argument data structure:
///
///     MessageType.MOVE_WALLS argument structure:
///		ui16[2], ui8[4][5]		// ui8  [arg length]        [cham 0 wall byte]
///	    ui16[3], ui8[6][7]		// ui8  [cham 1 wall byte]  [cham 2 wall byte]
///     ui16[4], ui8[8][9]     	// ui8  [cham 3 wall byte]  [cham 4 wall byte]
///     ui16[5], ui8[10][11]   	// ui8  [cham 5 wall byte]  [cham 6 wall byte]
///     ui16[6], ui8[12]  	    // ui8  [cham 7 wall byte]  [cham 8 wall byte]
///
class Esmacat_Com
{

    // --------------VARIABLES--------------
public:
    bool isEcatConnected = false;     // flag to track setup handshake of ethercat coms
    const int dtEcatDisconnect = 750; // time in ms to wait before final ecat register clear

    const char message_type_str[6][30] = {
        "MSG_NONE",
        "HANDSHAKE",
        "INITIALIZE_CYPRESS",
        "INITIALIZE_WALLS",
        "SESTEM_RESET",
        "MOVE_WALLS",
    };
    enum MessageType
    {
        MSG_NONE = 0,
        HANDSHAKE = 1, // handshake must equal 1
        INITIALIZE_CYPRESS = 2,
        INITIALIZE_WALLS = 3,
        SESTEM_RESET = 4,
        MOVE_WALLS = 5,
        nMsgTypEnum
    };
    const char error_type_str[7][30] = {
        "ERR_NONE",
        "ECAT_ID_DISORDERED",
        "ECAT_NO_MSG_TYPE_MATCH",
        "ECAT_NO_ERR_TYPE_MATCH",
        "ECAT_MISSING_FOOTER",
        "I2C_FAILED",
        "WALL_MOVE_FAILED"};
    enum ErrorType
    {
        ERR_NONE = 0,
        ECAT_ID_DISORDERED = 1,
        ECAT_NO_MSG_TYPE_MATCH = 2,
        ECAT_NO_ERR_TYPE_MATCH = 3,
        ECAT_MISSING_FOOTER = 4,
        I2C_FAILED = 5,
        WALL_MOVE_FAILED = 6,
        nErrTypeEnum
    };

    union RegUnion
    {                     // union for storing ethercat 8 16-bit reg entries, shareable accross 16 and 16 8 bit data types
        byte ui8[16];     // (byte) 1 byte
        uint16_t ui16[8]; // (uint16_t) 2 byte
        int si16[8];      // (int) 2 byte
        uint64_t ui64[2]; // (uint64_t) 8 byte
        int64_t si64[2];  // (int64_t) 8 byte
    };

    struct UnionIndStruct // union index struct for tracking 8 and 16 bit set and get indexes
    {
        uint8_t ii8 = 0;  // 8 bit index
        uint8_t ii16 = 0; // 16 bit index

        uint8_t upd8(uint8_t b_i = 255);
        uint8_t upd16(uint8_t b_i = 255);
        void reset();
    };

    struct EcatMessageStruct // class for handeling ethercat messages
    {
        RegUnion RegU = {};   // Union for storing ethercat 8 16-bit reg entries
        UnionIndStruct getUI; // Union index handler for getting union data
        UnionIndStruct setUI; // Union index handler for getting union data

        uint16_t msgID = 0;                        // Ethercat message ID
        uint16_t msgID_last = 0;                   // Last Ethercat message ID
        MessageType msgTp = MessageType::MSG_NONE; // Ethercat message type enum
        uint8_t msgTp_val = 0;                     // Ethercat message type val
        ErrorType errTp = ErrorType::ERR_NONE;     // Ethercat message error enum
        uint8_t errTp_val = 0;                     // Ethercat message error val
        uint8_t msgFoot[2] = {0};                  // Ethercat message footer [254,254]

        uint8_t argLen = 0;   // Ethercat number of 8 bit message arguments
        RegUnion ArgU = {};   // Union for storing message arguments
        UnionIndStruct argUI; // Union index handler for argument union data

        bool isNew = false; // Ethercat message new flag
        bool isErr = false; // Ethercat message error flag

        char msg_tp_str[50] = {0}; // Ethercat message type string
        char err_tp_str[50] = {0}; // Ethercat error type string
    };
    EcatMessageStruct sndEM; //  initialize message handler instance for sending messages
    EcatMessageStruct rcvEM; //  initialize message handler instance for receiving messages

private:
    static Maze_Debug _Dbg; // local instance of Maze_Debug class
    int ecatPinCS;           // chip select pin for ethercat shield
    SPISettings ecatSettingsSPI; // SPI settings for ethercat shield

    // ---------------METHODS---------------

public:
    Esmacat_Com(int = 10);

public:
    void ecatWriteRegValue(int, int);

public:
    void ecatReadRegAll(int[8]);

private:
    int _ecatReadRegValue(int);

private:
    bool _uSetCheckReg(EcatMessageStruct &, int[], bool = false);

private:
    void _uSetMsgID(EcatMessageStruct &, uint16_t = 255);
    bool _uGetMsgID(EcatMessageStruct &);

private:
    void _uSetMsgType(EcatMessageStruct &, MessageType);
    bool _uGetMsgType(EcatMessageStruct &);

private:
    void _uSetErrType(EcatMessageStruct &, ErrorType);
    void _uGetErrType(EcatMessageStruct &);

private:
    void _uSetArgLength(EcatMessageStruct &, uint8_t);
    void _uGetArgLength(EcatMessageStruct &);

private:
    void _uSetArgData8(EcatMessageStruct &, uint8_t);
    void _uSetArgData16(EcatMessageStruct &, uint16_t);
    void _uGetArgData8(EcatMessageStruct &);

private:
    void _uSetFooter(EcatMessageStruct &);
    bool _uGetFooter(EcatMessageStruct &);

private:
    void _uReset(EcatMessageStruct &);

private:
    void _resetReg();

private:
    void _trackErrors(EcatMessageStruct &, ErrorType, bool = false);

public:
    void initEcat(bool);

public:
    void readEcatMessage();

public:
    void writeEcatAck(uint8_t[] = nullptr, uint8_t = 0);
    void writeEcatAck(ErrorType, uint8_t[] = nullptr, uint8_t = 0);

private:
    void _printEcatReg(Maze_Debug::MT);
    void _printEcatReg(Maze_Debug::MT, int[]);
    void _printEcatReg(Maze_Debug::MT, RegUnion);
};

#endif
