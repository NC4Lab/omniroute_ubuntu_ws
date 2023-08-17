// ######################################

//=========== Esmacat_Com.h ============

// ######################################
#ifndef ESMACAT_COM_H
#define ESMACAT_COM_H

//============= INCLUDE ================
#include "Arduino.h"
#include "Maze_Debug.h"
#include "Cypress_Com.h"
#include "Esmacatshield.h"

extern bool DO_ECAT_SPI; ///< set this variable in your INO file to control block SPI [0:dont start, 1:start]

/// @brief This class deals with incoming and outgoing Ethercat communication.
class Esmacat_Com
{

    // ---------VARIABLES-----------------
public:
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
        MISSING_FOOTER = 4,
        N_ErrorType
    };

    union RegUnion
    {                     ///< union for storing ethercat 8 16-bit reg entries, shareable accross 16 and 16 8 bit data types
        byte ui8[16];     ///< (byte) 1 byte
        uint16_t ui16[8]; ///< (uint16_t) 2 byte
        int i16[8];       ///< (int) 2 byte
        uint64_t ui64[2]; ///< (uint64_t) 8 byte
    };

    struct UnionIndStruct ///< union index struct for tracking 8 and 16 bit set and get indexes
    {
        uint8_t i8 = 0;  ///< 8 bit index
        uint8_t i16 = 0; ///< 16 bit index

        uint8_t upd8(uint8_t b_i = 255);
        uint8_t upd16(uint8_t b_i = 255);
        void reset();
    };

    struct EcatMessageStruct ///< class for handeling ethercat messages
    {
        RegUnion RegU;    ///< Union for storing ethercat 8 16-bit reg entries
        UnionIndStruct getUI; ///< Union index handler for getting union data
        UnionIndStruct setUI; ///< Union index handler for getting union data

        uint16_t msgID = 0;                        ///< Ethercat message ID
        MessageType msgTp = MessageType::MSG_NONE; ///< Ethercat message error
        uint8_t msgFoot[2] = {0};                  ///< Ethercat message footer [254,254]

        uint8_t argLen = 0; ///< Ethercat number of 8 bit message arguments
        RegUnion ArgU;      ///< Union for storing message arguments
        UnionIndStruct argUI;   ///< Union index handler for argument union data

        ErrorType errTp = ErrorType::ERROR_NONE; ///< Ethercat message error
        char msg_tp_str[50] = {0};               ///< Ethercat message type string
        char err_tp_str[50] = {0};               ///< Ethercat error type string
        uint8_t msg_tp_val = 0;                  ///< Ethercat message type value

        bool isDone = false; ///< flag for message exicution completion
    };
    EcatMessageStruct sndEM; ///<  initialize message handler instance for sending messages
    EcatMessageStruct rcvEM; ///<  initialize message handler instance for receiving messages
    EcatMessageStruct tmpEM; ///<  initialize message handler instance for temporary receiving messages

private:
    static Maze_Debug _Dbg;     ///< local instance of Maze_Debug class 
    Esmacatshield _ESMA; //< instance of Esmacatshield class

    // -----------METHODS-----------------

public:
    Esmacat_Com();

private:
    void _uSetMsgID(EcatMessageStruct &, uint16_t = 255);
    void _uGetMsgID(EcatMessageStruct &);

private:
    void _uSetMsgType(EcatMessageStruct &, MessageType);
    bool _uGetMsgType(EcatMessageStruct &);

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
    void _checkErr(EcatMessageStruct &, ErrorType, bool);

public:
    void msgReset();

public:
    void sendEcatMessage(MessageType, uint8_t[] = nullptr, uint8_t = 0);

public:
    uint8_t getEcatMessage();

private:
    void _printEcatReg(uint8_t, int[] = nullptr);
    void _printEcatReg(uint8_t, RegUnion);
};

#endif
