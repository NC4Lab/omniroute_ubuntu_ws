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
    //_Dbg.printMsgTime("\t\t\t upd8: i8=%d, i16=%d b_i=%d", i8, i16, b_i); // TEMP
    b_i = b_i == 255 ? ii8 : b_i; // if b_i is 255, use current union index
    ii8 = b_i + 1;
    ii16 = ii8 / 2;
    return b_i;
}

/// @brief Update union 16 bit and 8 bit index
/// @return Last updated 16 bit index
uint8_t Esmacat_Com::UnionIndStruct::upd16(uint8_t b_i)
{
    //_Dbg.printMsgTime("\t\t\t upd16: i8=%d, i16=%d b_i=%d", i8, i16, b_i); // TEMP
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
    // Check if any reg_arr values == -1
    for (uint8_t i = 0; i < 8; i++)
    {
        if (p_reg_arr[i] == -1)
            return false;
    }

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
void Esmacat_Com::_uGetMsgID(EcatMessageStruct &r_EM)
{
    r_EM.msgID_last = r_EM.msgID; // store last message ID if
    r_EM.msgID = r_EM.RegU.ui16[r_EM.getUI.upd16(0)];
    // Check/log error skipped or out of sequence messages
    if (r_EM.msgID - r_EM.msgID_last != 1 &&
        r_EM.msgID != r_EM.msgID_last) // don't log errors for repeat message reads
        _trackErrType(r_EM, ErrorType::ECAT_ID_DISORDERED);
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
void Esmacat_Com::_uGetMsgType(EcatMessageStruct &r_EM)
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
        _trackErrType(r_EM, ErrorType::ECAT_NO_MSG_TYPE_MATCH);
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
void Esmacat_Com::_uGetFooter(EcatMessageStruct &r_EM)
{
    r_EM.msgFoot[0] = r_EM.RegU.ui8[r_EM.getUI.upd8()]; // copy first footer byte
    r_EM.msgFoot[1] = r_EM.RegU.ui8[r_EM.getUI.upd8()]; // copy second footer byte

    // Log missing footer error
    if (r_EM.msgFoot[0] != 254 || r_EM.msgFoot[1] != 254) // check for valid footers
        _trackErrType(r_EM, ErrorType::ECAT_MISSING_FOOTER);
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

/// @brief Check for and log any Ecat or runtime errors
void Esmacat_Com::_trackErrType(EcatMessageStruct &r_EM, ErrorType err_tp, bool do_reset)
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
            _Dbg.printMsgTime("!!ERROR: Ecat: %s: id[new,last]=[%d,%d] type=%s[%d]!!", r_EM.err_tp_str, r_EM.msgID, r_EM.msgID_last, r_EM.msg_tp_str, r_EM.msgTp_val);
            _printEcatReg(0, r_EM.RegU); // TEMP
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
        _Dbg.printMsgTime("=== ECAT COMMS DISCONNECTED ===");

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

    // Check register for garbage or incomplete data and copy register data into union
    if (!_uSetCheckReg(rcvEM, reg_arr))
        return;

    // Skip leftover register entries and ethercat setup junk (e.g., ui8 == 255)
    if (!isEcatConnected &&         // check if still waiting for handshake
        (rcvEM.RegU.ui16[0] != 1 || // directly check union id entry for first message
         rcvEM.RegU.ui8[2] != 1))   // directly check union type entry for handshake (e.g., 1)
        return;

    // Get message id and check for out of sequence messages
    _uGetMsgID(rcvEM);

    // Skip redundant messages
    if (rcvEM.msgID == rcvEM.msgID_last)
        return;

    // Get message type and check if valid
    _uGetMsgType(rcvEM);

    // Get argument length and argument data
    _uGetArgData8(rcvEM);

    // Get footer and flag if not found
    _uGetFooter(rcvEM);

    // Check for error and send error status
    if (rcvEM.isErr)
    {
        writeEcatAck(rcvEM.errTp);
        return; // skip processing message
    }

    // Set new message flag
    rcvEM.isNew = true;

    _Dbg.printMsgTime("(%d)ECAT RECEIVED: %s", rcvEM.msgID, rcvEM.msg_tp_str);
    _printEcatReg(0, rcvEM.RegU); // TEMP
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

/// OVERLOAD: Option for including an error type to send with the message.
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
    if (p_msg_arg_data != nullptr) // store message arguments if provided
        for (size_t i = 0; i < msg_arg_len; i++)
            _uSetArgData8(sndEM, p_msg_arg_data[i]); // store message arguments if provided
    else
        _uSetArgLength(sndEM, 0); // set arg length to 0

    // Reset error type and flag
    _trackErrType(rcvEM, rcvEM.errTp, true);

    // 	------------- Finish setup and write -------------

    // Store footer
    _uSetFooter(sndEM); // in union

    // Write message
    for (size_t i = 0; i < 8; i++)
        _ESMA.write_reg_value(i, sndEM.RegU.si16[i]);

    // Reset new rcv message flag
    rcvEM.isNew = false;

    // Print ack message info with message type being acked
    _Dbg.printMsgTime("(%d)ECAT ACK SENT: %s:%s", sndEM.msgID, sndEM.msg_tp_str, sndEM.err_tp_str);
    _printEcatReg(0, sndEM.RegU); // TEMP
}

/// @brief Used for printing Ethercat register values.
/// This version of the function accepts a RegUnion object.
/// @param d_type: Specifies the data type to print. [0=uint8, 1=uint16, 2=int16]
/// @param u: A RegUnion object containing the register values to print.
void Esmacat_Com::_printEcatReg(uint8_t d_type, RegUnion u_reg)
{
    // Print out register
    _Dbg.printMsgTime("\t Ecat Register");
    for (size_t i = 0; i < 8; i++)
    {
        if (d_type == 1)
            _Dbg.printMsgTime("\t\t ui16[%d] %d", i, u_reg.si16[i]);
        if (d_type == 1)
            _Dbg.printMsgTime("\t\t ui16[%d] %d", i, u_reg.ui16[i]);
        if (d_type == 0)
            _Dbg.printMsgTime("\t\t\t ui8[%d][%d]  %d %d", 2 * i, 2 * i + 1, u_reg.ui8[2 * i], u_reg.ui8[2 * i + 1]);
    }
    _Dbg.printMsgTime(" ");
}

/// OVERLOAD: Option for printing Ethercat register values.
/// This version of the function accepts an array of integer register values.
/// @param d_type: Specifies the data type to print. [0, 1] corresponds to [uint8, uint16].
/// @param p_reg: An array of existing register values. DEFAULT: the function reads the register values directly from Ethercat.
void Esmacat_Com::_printEcatReg(uint8_t d_type, int p_reg[])
{
    RegUnion u;
    int p_r[8]; // initialize array to handle null array argument

    // Get register values directly from Ethercat
    if (p_reg == nullptr)
        _ESMA.get_ecat_registers(p_r);
    else // copy array
        for (size_t i = 0; i < 8; i++)
            p_r[i] = p_reg[i];

    // Convert to RegUnion
    for (size_t i = 1; i < 8; i++)
        u.ui16[i] = p_reg[i];

    // Pass to other _printEcatU
    _printEcatReg(d_type, u);
}
