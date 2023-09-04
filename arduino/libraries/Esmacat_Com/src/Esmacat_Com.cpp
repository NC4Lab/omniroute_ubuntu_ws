// ######################################

//========== Esmacat_Com.cpp ===========

// ######################################

//============= INCLUDE ================
#include "Esmacat_Com.h"

//========DEFINES: Esmacat_Com==========

Maze_Debug Esmacat_Com::_Dbg; ///< static local instance of Maze_Debug class

//========CLASS: Esmacat_Com==========

/// @brief CONSTUCTOR: Create Esmacat_Com class instance
Esmacat_Com::Esmacat_Com()
{
    // Start SPI for Ethercat
    if (DO_ECAT_SPI)
        _ESMA.start_spi();

    // Setup/reset variables
    initEcat();
}

/// @brief Update union 8 bit and 16 bit index
/// @return Last updated 8 bit index
uint8_t Esmacat_Com::UnionIndStruct::upd8(uint8_t b_i)
{
    b_i = b_i == 255 ? ii8 : b_i; // if b_i is 255, use current union index
    ii8 = b_i + 1;
    ii16 = ii8 / 2;
    return b_i;
}

/// @brief Update union 16 bit and 8 bit index
/// @return Last updated 16 bit index
uint8_t Esmacat_Com::UnionIndStruct::upd16(uint8_t b_i)
{
    b_i = b_i == 255 ? ii16 : b_i; // if b_i is 255, use current union index
    ii16 = b_i + 1;
    ii8 = ii16 * 2;
    return b_i;
}

/// @brief Reset union 8 bit and 16 bit index
void Esmacat_Com::UnionIndStruct::reset()
{
    ii8 = 0;
    ii16 = 0;
}

bool Esmacat_Com::_uSetCheckReg(EcatMessageStruct &r_EM, int p_reg_arr[])
{
    // // Print register values TEMP
    // static int id_last = 0;
    // if (p_reg_arr[0] != id_last)
    // {
    //     _printEcatReg(_Dbg.MT::DEBUG, 0, p_reg_arr);
    //     id_last = p_reg_arr[0];
    // }

    // // TEMP
    // delay(1000);
    // r_EM.RegU.ui16[0] = 1;
    // r_EM.RegU.ui8[2] = 1;
    // r_EM.RegU.ui8[3] = 0;
    // r_EM.RegU.ui8[4] = 3;
    // r_EM.RegU.ui8[3] = 1;
    // r_EM.RegU.ui8[5] = 9;
    // r_EM.RegU.ui8[6] = 3;
    // r_EM.RegU.ui8[7] = 255;
    // r_EM.RegU.ui8[8] = 254;
    // r_EM.RegU.ui8[9] = 254;
    // return true;

    // Check if all reg_arr values equal to 0 or -1
    bool is_garbage = true;
    for (uint8_t i = 0; i < 8; i++)
        if (p_reg_arr[i] != 0 && p_reg_arr[i] != -1)
            is_garbage = false;
    if (is_garbage)
        return false;

    // Copy reg_arr values to union
    for (uint8_t i = 0; i < 8; i++)
        r_EM.RegU.si16[i] = p_reg_arr[i];
    return true;
}

/// @brief Set message ID entry in union and updated associated variable
void Esmacat_Com::_uSetMsgID(EcatMessageStruct &r_EM, uint16_t msg_id)
{
    // Set message ID union entry
    r_EM.RegU.ui16[r_EM.setUI.upd16(0)] = msg_id;
    _uGetMsgID(r_EM); // copy from union to associated struct variable
}

/// @brief Get message ID from union
///
/// @return True if message ID is valid (in sequence with last), false if not
bool Esmacat_Com::_uGetMsgID(EcatMessageStruct &r_EM)
{
    r_EM.msgID_last = r_EM.msgID; // store last message ID if
    r_EM.msgID = r_EM.RegU.ui16[r_EM.getUI.upd16(0)];
    // Check/log error skipped or out of sequence messages
    if (r_EM.msgID - r_EM.msgID_last != 1 &&
        r_EM.msgID != r_EM.msgID_last) // don't log errors for repeat message reads
    {
        _trackErrors(r_EM, ErrorType::ECAT_ID_DISORDERED);
        return false;
    }
    return true;
}

/// @brief Set message type entry in union and updated associated variable
void Esmacat_Com::_uSetMsgType(EcatMessageStruct &r_EM, MessageType msg_type_enum)
{
    // Get message type value
    uint8_t msg_type_val = static_cast<uint8_t>(msg_type_enum);

    // Set message type union entry
    r_EM.RegU.ui8[r_EM.setUI.upd8(2)] = msg_type_val;
    _uGetMsgType(r_EM); // copy from union to associated struct variable
}

/// @brief Get message type from union
///
/// @return True if message type is valid, false if not
bool Esmacat_Com::_uGetMsgType(EcatMessageStruct &r_EM)
{
    // Get message type value
    uint8_t msg_type_val = r_EM.RegU.ui8[r_EM.getUI.upd8(2)];

    // Check if 'msg_type_val' corresponds to any value in the 'MessageType' Enum.
    bool is_found = false;
    for (int i = 0; i < nMsgTypEnum; ++i)
    {
        if (msg_type_val == i)
        {
            is_found = true;
            break;
        }
    };
    r_EM.isErr = !is_found;

    // Log error and set type to none if not found
    if (r_EM.isErr)
    {
        msg_type_val = static_cast<uint8_t>(MessageType::MSG_NULL);
        _trackErrors(r_EM, ErrorType::ECAT_NO_MSG_TYPE_MATCH);
    }

    // Get message type enum and store val
    r_EM.msgTp = static_cast<Esmacat_Com::MessageType>(msg_type_val);
    r_EM.msgTp_val = msg_type_val;

    // Copy string to struct
    if (!r_EM.isErr)
        strncpy(r_EM.msg_tp_str, message_type_str[msg_type_val], sizeof(r_EM.msg_tp_str) - 1);
    else
        strncpy(r_EM.msg_tp_str, "MSG_NULL", sizeof(r_EM.msg_tp_str) - 1); // set to none if error
    r_EM.msg_tp_str[sizeof(r_EM.msg_tp_str) - 1] = '\0';                   // ensure null termination

    return is_found;
}

/// @brief Set error type entry in union and updated associated variable
void Esmacat_Com::_uSetErrType(EcatMessageStruct &r_EM, ErrorType err_type_enum)
{
    // Get error type value
    uint8_t err_type_val = static_cast<uint8_t>(err_type_enum);

    // Set error type union entry
    r_EM.RegU.ui8[r_EM.setUI.upd8(3)] = err_type_val;
    _uGetErrType(r_EM); // copy from union to associated struct variable
}

/// @brief Get error type from union
void Esmacat_Com::_uGetErrType(EcatMessageStruct &r_EM)
{
    // Get error type value
    r_EM.errTp_val = r_EM.RegU.ui8[r_EM.getUI.upd8(3)];

    // Get error type enum and store val
    r_EM.errTp = static_cast<Esmacat_Com::ErrorType>(r_EM.errTp_val);

    // Copy string to struct
    strncpy(r_EM.err_tp_str, error_type_str[r_EM.errTp_val], sizeof(r_EM.err_tp_str) - 1);
    r_EM.err_tp_str[sizeof(r_EM.err_tp_str) - 1] = '\0'; // ensure null termination
}

/// @brief Set message argument length entry in union and updated associated variable
void Esmacat_Com::_uSetArgLength(EcatMessageStruct &r_EM, uint8_t msg_arg_len)
{
    // Set argument length union entry
    r_EM.RegU.ui8[r_EM.setUI.upd8(4)] = msg_arg_len;
    _uGetArgLength(r_EM); // copy from union to associated struct variable
}

/// @brief Get message argument length
void Esmacat_Com::_uGetArgLength(EcatMessageStruct &r_EM)
{
    r_EM.argLen = r_EM.RegU.ui8[r_EM.getUI.upd8(4)];
}

/// @brief Set 8-bit message argument data and length entries in union
///
/// @note calls @ref Esmacat_Com::_uSetArgLength
void Esmacat_Com::_uSetArgData8(EcatMessageStruct &r_EM, uint8_t msg_arg_data8)
{
    // Store argurment length

    // Increment argument union index
    r_EM.argUI.upd8();

    // Update message argument length from argument union 8-bit index
    _uSetArgLength(r_EM, r_EM.argUI.ii8);

    // Get reg union index accounting for itterative calls to store more data
    uint8_t regu8_i = r_EM.argUI.ii8 + 4;

    // Set message argument data in reg union
    r_EM.RegU.ui8[r_EM.setUI.upd8(regu8_i)] = msg_arg_data8;
    _uGetArgData8(r_EM); // copy from union to associated struct variable
}

/// @brief Set 16-bit message argument data and length entries in union
///
/// @note calls @ref Esmacat_Com::_uSetArgLength
void Esmacat_Com::_uSetArgData16(EcatMessageStruct &r_EM, uint16_t msg_arg_data16)
{
    // Increment argument union index
    r_EM.argUI.upd16();

    // Update message argument length from argument union 8-bit index
    _uSetArgLength(r_EM, r_EM.argUI.ii8);

    // Get reg union index accounting for itterative calls to store more data
    uint8_t regu16_i = (r_EM.argUI.ii8 + 4) / 2;

    // Set message argument data in reg union
    r_EM.RegU.ui16[r_EM.setUI.upd16(regu16_i)] = msg_arg_data16; // set message argument data in reg union
    _uGetArgData8(r_EM);                                         // copy from union to associated struct variable
}

/// @brief Get 8-bit reg union message argument data and copy to arg union
void Esmacat_Com::_uGetArgData8(EcatMessageStruct &r_EM)
{
    _uGetArgLength(r_EM); // get argument length from union
    for (size_t i = 0; i < r_EM.argLen; i++)
        r_EM.ArgU.ui8[i] = r_EM.RegU.ui8[r_EM.getUI.upd8()]; // copy to 8-bit argument Union
}

/// @brief Set message footer entry in union and updated associated variable
void Esmacat_Com::_uSetFooter(EcatMessageStruct &r_EM)
{
    r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254;
    r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254;
    _uGetFooter(r_EM); // copy from union to associated struct variable
}

/// @brief Get message footer from union
///
/// @return True if footer is valid, false if not
bool Esmacat_Com::_uGetFooter(EcatMessageStruct &r_EM)
{
    r_EM.msgFoot[0] = r_EM.RegU.ui8[r_EM.getUI.upd8()]; // copy first footer byte
    r_EM.msgFoot[1] = r_EM.RegU.ui8[r_EM.getUI.upd8()]; // copy second footer byte

    // Log missing footer error
    if (r_EM.msgFoot[0] != 254 || r_EM.msgFoot[1] != 254) // check for valid footers
    {
        _trackErrors(r_EM, ErrorType::ECAT_MISSING_FOOTER);
        return false;
    }
    return true;
}

/// @brief Reset union data and indeces
void Esmacat_Com::_uReset(EcatMessageStruct &r_EM)
{
    // Reset union data to 0
    r_EM.RegU.ui64[0] = 0;
    r_EM.RegU.ui64[1] = 0;

    // Reset dynamic union indeces
    r_EM.setUI.reset();
    r_EM.getUI.reset();
    r_EM.argUI.reset();
}

/// @brief Reset all register values to -1 to clear buffer
///
/// @note  This is a workaround for the fact that the Esmacat does not clear
/// this can lead to only partial or overlapping messages being read
void Esmacat_Com::_resetReg()
{
    for (size_t i = 0; i < 8; i++)
        _ESMA.write_reg_value(i, -1);
}

/// @brief Check for, log and send any Ecat or runtime errors
///
/// @param err_tp: The type of error to be logged.
/// @param do_reset: OPTIONAL: If true, reset error type to none.
void Esmacat_Com::_trackErrors(EcatMessageStruct &r_EM, ErrorType err_tp, bool do_reset)
{
    // Check for error
    if (!do_reset)
    {
        // Only run once
        if (r_EM.errTp != err_tp)
        {
            // Set error type and flag
            r_EM.errTp = err_tp;
            r_EM.isErr = true;

            // Get error string
            uint8_t err_tp_val = static_cast<uint8_t>(err_tp);
            strncpy(r_EM.err_tp_str, error_type_str[err_tp_val], sizeof(r_EM.err_tp_str) - 1);
            r_EM.err_tp_str[sizeof(r_EM.err_tp_str) - 1] = '\0'; // ensure null termination

            // Print error
            _Dbg.printMsg(_Dbg.MT::ERROR, "Ecat: %s: id new[%d] id last[%d] type[%d][%s]", r_EM.err_tp_str, r_EM.msgID, r_EM.msgID_last, r_EM.msgTp_val, r_EM.msg_tp_str);
            _printEcatReg(_Dbg.MT::ERROR, 0, r_EM.RegU);

            // Send error ack
            writeEcatAck(r_EM.errTp);
        }
    }

    // Unset error type
    else
    {
        r_EM.errTp = ErrorType::ERR_NULL;
        r_EM.isErr = false;
    }
}

/// @brief: Initialize/reset all Ecat varables structs.
void Esmacat_Com::initEcat()
{

    // Reset message union data and indeces
    _uReset(sndEM);
    _uReset(rcvEM);

    // Reset message counters
    sndEM.msgID = 0;
    rcvEM.msgID = 0;

    // Add delay if this is a reinitialization
    if (isEcatConnected)
        delay(dtEcatDisconnect);

    // Reset Ecat register data
    _resetReg();

    // Check if this is a reinitialization
    if (isEcatConnected)
        _Dbg.printMsg(_Dbg.MT::ATTN, "ECAT COMMS DISCONNECTED");

    // Reset Ethercat handshake flag
    isEcatConnected = false;
}

/// @brief Used to get incoming ROS ethercat msg data.
void Esmacat_Com::readEcatMessage()
{

    // Bail if previous message not processed
    if (rcvEM.isNew)
        return;

    // Read esmacat buffer to get register data
    int reg_arr[8];
    _ESMA.get_ecat_registers(reg_arr);

    // // Print register values TEMP
    // static int id_last = 5;
    // if (reg_arr[0] != id_last)
    // {
    //     _printEcatReg(_Dbg.MT::DEBUG, 0, reg_arr);
    //     id_last = reg_arr[0];
    // }

    // Check register for garbage or incomplete data and copy register data into union
    if (!_uSetCheckReg(rcvEM, reg_arr))
        return;

    // Another check for garbage registry stuff
    if (!isEcatConnected &&         // check if still waiting for handshake
        (rcvEM.RegU.ui16[0] != 1 || // directly check union id entry for first message
         rcvEM.RegU.ui8[2] != 1))   // directly check union type entry for handshake (e.g., 1)
        return;

    // Get message id and check for out of sequence messages
    if (!_uGetMsgID(rcvEM))
        return; // skip message processing if error returned

    // Skip redundant messages
    if (rcvEM.msgID == rcvEM.msgID_last)
        return;

    // Get message type and check if valid
    if (!_uGetMsgType(rcvEM))
        return; // skip message processing if error returned

    // Get argument length and argument data
    _uGetArgData8(rcvEM);

    // Get footer and flag if not found
    if (!_uGetFooter(rcvEM))
        return; // skip message processing if error returned

    // Set new message flag
    rcvEM.isNew = true;

    _Dbg.printMsg("(%d)ECAT RECEIVED: %s", rcvEM.msgID, rcvEM.msg_tp_str);
    _printEcatReg(_Dbg.MT::DEBUG, 0, rcvEM.RegU); // TEMP
}

/// @brief: Used to send outgoing ROS ethercat msg data signalling which walls to raise.
///
///	@note: The outgoing register is structured uint16[8]
///	with all but first 16-bit value seperated into bytes
///
/// @note: The message length corresponds to number of bytes.
///
/// @param msg_type_enum: The type of the message to be sent.
/// @param error_type_enum: The type of error to be logged.
/// @param p_msg_arg_data: OPTIONAL: The data for the message arguments. DEFAULT: nullptr.
/// @param msg_arg_len: OPTIONAL: The length of the message arguments in uint8. DEFAULT: 255.
void Esmacat_Com::writeEcatAck(uint8_t p_msg_arg_data[], uint8_t msg_arg_len)
{
    writeEcatAck(ErrorType::ERR_NULL, p_msg_arg_data, msg_arg_len);
}

/// @overload: Option for including an error type to send with the message.
///
/// @param error_type_enum: Specifies an enum from for the @ref ErrorType enum.
void Esmacat_Com::writeEcatAck(ErrorType error_type_enum, uint8_t p_msg_arg_data[], uint8_t msg_arg_len)
{

    // Set all register values to -1 to clear buffer
    _resetReg();

    // Reset union variables and write to clear buffer
    _uReset(sndEM);

    // Store reused recieve message id in union
    _uSetMsgID(sndEM, rcvEM.msgID);

    // Store recived message type in union
    _uSetMsgType(sndEM, rcvEM.msgTp);

    // Store error type in union
    _uSetErrType(sndEM, error_type_enum);

    // 	------------- Store arguments -------------

    // Store error arguments
    if (msg_arg_len > 0 && p_msg_arg_data != nullptr) // store message arguments if provided
        for (size_t i = 0; i < msg_arg_len; i++)
            _uSetArgData8(sndEM, p_msg_arg_data[i]); // store message arguments if provided
    else
        _uSetArgLength(sndEM, 0); // set arg length to 0

    // Reset error type and flag
    _trackErrors(rcvEM, rcvEM.errTp, true);

    // 	------------- Finish setup and write -------------

    // Store footer
    _uSetFooter(sndEM); // in union

    // Write message
    for (size_t i = 0; i < 8; i++)
        _ESMA.write_reg_value(i, sndEM.RegU.si16[i]);

    // Reset new rcv message flag
    rcvEM.isNew = false;

    // Print ack message info with message type being acked
    _Dbg.printMsg("(%d)ECAT ACK SENT: %s:%s", sndEM.msgID, sndEM.msg_tp_str, sndEM.err_tp_str);
    _printEcatReg(_Dbg.MT::DEBUG, 0, sndEM.RegU); // TEMP
}

/// @brief Used for printing curren Ethercat register values.
///
/// @param msg_type_enum Enum specifying message type.
/// @param d_type: Specifies the data type to print. [0=uint8, 1=uint16, 2=int16]
void Esmacat_Com::_printEcatReg(Maze_Debug::MT msg_type_enum, uint8_t d_type)
{
    RegUnion U;

    // Get register values directly from Ethercat
    _ESMA.get_ecat_registers(U.si16);

    // Pass to other _printEcatU() method
    _printEcatReg(msg_type_enum, d_type, U);
}
/// @overload: Option for printing Ethercat register values from an array.
///
/// @param p_reg: An array of existing register values.
void Esmacat_Com::_printEcatReg(Maze_Debug::MT msg_type_enum, uint8_t d_type, int p_reg[])
{
    RegUnion U;

    // Copy to RegUnion
    for (size_t i = 1; i < 8; i++)
        U.ui16[i] = p_reg[i];

    // Pass to other _printEcatU() method
    _printEcatReg(msg_type_enum, d_type, U);
}
/// @overload Option for printing Ethercat register values stored in RegUnion.
///
/// @param U: A RegUnion object containing the register values to print.
void Esmacat_Com::_printEcatReg(Maze_Debug::MT msg_type_enum, uint8_t d_type, RegUnion U)
{
    // Print out register
    _Dbg.printMsg("\t Ecat Register");
    for (size_t i = 0; i < 8; i++)
    {
        if (d_type == 1)
            _Dbg.printMsg("\t ui16[%d] [%d]", i, U.si16[i]);
        if (d_type == 1)
            _Dbg.printMsg("\t ui16[%d] [%d]", i, U.ui16[i]);
        if (d_type == 0)
            _Dbg.printMsg("\t ui8[%d][%d]  [%d][%d]", 2 * i, 2 * i + 1, U.ui8[2 * i], U.ui8[2 * i + 1]);
    }
}
