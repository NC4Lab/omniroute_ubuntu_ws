
    def _resetU(r_EM):
        # Clear union
        r_EM.RegU.ui64[0] = 0
        r_EM.RegU.ui64[1] = 0

        # Reset union indices
        r_EM.u8i = 0
        r_EM.u16i = 0

    def _seti8(r_EM, dat_8):
        # Store data
        r_EM.RegU.ui8[r_EM.u8i] = dat_8

        # Update union indices
        r_EM.u8i += 1
        r_EM.u16i = r_EM.u8i // 2

    def _seti16(r_EM, dat_16):
        # Store data
        r_EM.RegU.ui16[r_EM.u16i] = dat_16
        # Update union indices
        r_EM.u16i += 1
        r_EM.u8i = r_EM.u16i * 2

    def _geti8(r_EM):
        # Store data
        dat_8 = r_EM.RegU.ui8[r_EM.u8i]

        # Update union indices
        r_EM.u8i += 1
        r_EM.u16i = r_EM.u8i // 2

        return dat_8

        # Store data
        dat_16 = r_EM.RegU.ui16[r_EM.u16i]

        # Update union indices
        r_EM.u16i += 1
        r_EM.u8i = r_EM.u16i * 2

        return dat_16

    def _setupMsgStruct(r_EM, msg_id, msg_type_val):
        # Check for valid message type
        is_found = msg_type_val in MessageType.__members__

        # Set type to none if not found
        if not is_found:
            msg_type_val = MessageType.MSG_NONE.value

        # Store message id
        r_EM.msgID = msg_id

        # Store message type value
        r_EM.msg_tp_val = msg_type_val

        # Get message type enum
        r_EM.msgTp = MessageType(msg_type_val)

        # Copy string to struct
        r_EM.msg_tp_str = r_EM.msgTp.name if not is_err else "NULL"

        return not is_found

    def _checkErr(r_EM, err_tp, is_err):
        # Get error string
        err_tp_val = err_tp.value
        r_EM.err_tp_str = err_tp.name

        # Handle error
        if is_err:
            if r_EM.errTp != err_tp: # only run once
                # Set error type
                r_EM.errTp = err_tp
                # Assuming _DB.printMsgTime is a function to print debug messages
                _DB.printMsgTime("!!ERROR: Ecat: %s: id=%d type=%s[%d]!!", r_EM.err_tp_str, r_EM.msgID, r_EM.msg_tp_str, r_EM.msg_tp_val)
                printEcatU(0, r_EM.RegU) # TEMP
        elif r_EM.errTp == err_tp:
            r_EM.errTp = ErrorType.ERROR_NONE # unset error type

    def writeEthercatMessage(msg_type_enum, p_msg_arg_data=None, msg_arg_lng=255):
        # Assuming sndEM is a global instance of EsmacatMessage
        global sndEM

        # Reset union variables
        _resetU(sndEM)

        # Update message id: iterate id and roll over to 1 if max 16 bit value is reached
        msg_id = sndEM.msgID + 1 if sndEM.msgID < 65535 else 1

        # Store message type info
        _setupMsgStruct(sndEM, msg_id, msg_type_enum.value)

        # Store message id
        _seti16(sndEM, sndEM.msgID) # message id

        # Store message type value
        _seti8(sndEM, sndEM.msg_tp_val)

        # Store message argument length
        _seti8(sndEM, msg_arg_lng) # message argument length

        # Store footer
        _seti8(sndEM, 254)
        _seti8(sndEM, 254)

        # Set flag
        sndEM.isDone = False

        # Write message
        for i in range(8):
            # Replace with your EtherCAT write function
            pass

        # Print message
        # Assuming _DB.printMsgTime is a function to print debug messages
        _DB.printMsgTime("STORE Ecat Message: id=%d type=%s", sndEM.msgID, sndEM.msg_tp_str)

    def readEthercatMessage():
        # Assuming rcvEM is a global instance of EsmacatMessage
        global rcvEM

        tempEM = EsmacatMessage() # temp ethercat message struct
        is_err = False # error flag

        # Reset union variables
        _resetU(tempEM)

        # Read esmacat buffer and copy into union
        reg_dat = [0]*8 # Replace with your EtherCAT read function

        for i in range(8):
            tempEM.RegU.ui16[i] = reg_dat[i]

        # Skip ethercat setup junk (255)
        if tempEM.RegU.ui8[0] == 255 or tempEM.RegU.ui8[1] == 255:
            return 0

        # Store message id
        tempEM.msgID = _geti16(tempEM)

        # Store message type value
        tempEM.msg_tp_val = _geti8(tempEM)

        # Skip redundant messages
        if tempEM.msgID == rcvEM.msgID:
            return 0

        # Setup message struct and check for valid message type
        is_err = _setupMsgStruct(tempEM, tempEM.msgID, tempEM.msg_tp_val)

        # Run check error for valid message type
        _checkErr(tempEM, ErrorType.NO_MESSAGE_TYPE_MATCH, is_err)
        if is_err:
            return 2 # return error flag

        # Check if message is preceding handshake
        is_err = not isHandshakeDone and tempEM.msgTp != MessageType.HANDSHAKE
        _checkErr(tempEM, ErrorType.REGISTER_LEFTOVERS, is_err)
        if is_err:
            return 2 # return error flag

        # Check for skipped or out of sequence messages
        is_err = tempEM.msgID - rcvEM.msgID != 1
        _checkErr(tempEM, ErrorType.MESSAGE_ID_DISORDERED, is_err)
        if is_err:
            return 2 # return error flag

        # Get argument length
        tempEM.msg_arg_lng = _geti8(tempEM)

        # Parse 8 bit message arguments
        for i in range(tempEM.msg_arg_lng):
            tempEM.ArgDat[i] = _geti8(tempEM)

        # Check for footer
        is_err = _geti8(tempEM) != 254 or _geti8(tempEM) != 254
        _checkErr(tempEM, ErrorType.MISSING_FOOTER, is_err)
        if is_err:
            return 2 # return error flag

        # Set flag
        tempEM.isDone = False

        # Copy over data
        rcvEM = tempEM

        # Print message
        # Assuming _DB.printMsgTime is a function to print debug messages
        _DB.printMsgTime("RECIEVED Ecat Message: id=%d type=%s", rcvEM.msgID, rcvEM.msg_tp_str)

        # Return new message flag
        return 1
