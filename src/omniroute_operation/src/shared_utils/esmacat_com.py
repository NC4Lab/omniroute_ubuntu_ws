#!/usr/bin/env python
# Class for managing communication with the Esmacat shields via EtherCAT

# Custom Imports
from shared_utils.maze_debug import MazeDB

# Standard Library Imports
import ctypes
from enum import Enum

# ROS Imports
import rospy
from std_msgs.msg import *
from omniroute_esmacat_ros.msg import *
from omniroute_operation.msg import *

class EsmacatCom:
    """ 
    This class is used to communicate with the arduino via ethercat.
    It is used to send and receive messages from the arduino.

    Incoming/Outgoing RegUnion message structure:
	ui16[0], ui8[0][1]		i16  [msg id]
	ui16[1], ui8[2][3]		ui8  [msg type]      [err type]
	ui16[2], ui8[4][5]		ui8  [arg length]    [arg 0]
	ui16[3], ui8[6][7]		ui8  [arg 1]         [arg 2]
	ui16[4], ui8[8][9]     	ui8  [arg 3]         [arg 4]
	ui16[5], ui8[10][11]   	ui8  [arg 5]         [arg 6]
	ui16[6], ui8[12][13]    ui8  [arg 7]         [arg 8]
	ui16[7], ui8[14][15]    ui8  [footer]        [footer]
    """

    # ------------------------ CLASS VARIABLES ------------------------

    # Handshake finished flag
    isEcatConnected = False

    # Specify delay to start and check reading/writing Esmacat data
    dt_ecat_start = 1  # (sec)
    dt_ecat_check = 0.1  # (sec)

    # ------------------------ NESTED CLASSES ------------------------

    class MessageType(Enum):
        """ Enum for ethercat python to arduino message type ID """
        MSG_NONE = 0
        HANDSHAKE = 1  # handshake must equal 1
        INITIALIZE_CYPRESS = 2
        INITIALIZE_WALLS = 3
        REINITIALIZE_SYSTEM = 4
        RESET_SYSTEM = 5
        MOVE_WALLS = 6
        SYNC_SET_OPTITRACK_PIN = 100
        SYNC_SET_SPIKEGADGETS_PIN = 101
        GANTRY_INITIALIZE_GRBL = 200
        GANTRY_HOME = 201
        GANTRY_MOVE_REL = 202
        GANTRY_SET_FEEDER = 203
        GANTRY_RUN_PUMP = 204

    class ErrorType(Enum):
        """ Enum for tracking message errors """
        ERR_NONE = 0
        ECAT_ID_DISORDERED = 1
        ECAT_NO_MSG_TYPE_MATCH = 2
        ECAT_NO_ERR_TYPE_MATCH = 3
        ECAT_MISSING_FOOTER = 4
        I2C_FAILED = 5
        WALL_MOVE_FAILED = 6

    class RegUnion(ctypes.Union):
        """ C++ style Union for storing ethercat data shareable accross 8 and 16-bit data types """
        _fields_ = [("ui8", ctypes.c_uint8 * 16),
                    ("ui16", ctypes.c_uint16 * 8),
                    ("si16", ctypes.c_int16 * 8),
                    ("ui64", ctypes.c_uint64 * 2),
                    ("si64", ctypes.c_int64 * 2)]

    class UnionIndStruct:
        """ Struct for storing ethercat data shareable accross 8 and 16-bit data types """

        def __init__(self):
            self.ii8 = 0  # 8-bit index
            self.ii16 = 0  # 16-bit index

        def upd8(self, b_i=255):
            """Update union 8-bit and 16-bit index and return last updated 8-bit index"""
            b_i = b_i if b_i != 255 else self.ii8 # if b_i is 255, use current union index
            self.ii8 = b_i + 1
            self.ii16 = self.ii8 // 2
            return b_i

        def upd16(self, b_i=255):
            """Update union 16-bit and 8-bit index and return last updated 16-bit index"""
            b_i = b_i if b_i != 255 else self.ii16 # if b_i is 255, use current union index
            self.ii16 = b_i + 1
            self.ii8 = self.ii16 * 2 
            return b_i

        def reset(self):
            """Reset union 8-bit and 16-bit index"""
            self.ii8 = 0
            self.ii16 = 0

    class EcatMessageStruct:
        """ Struct for storing ethercat messages """

        def __init__(self):
            # Union for storing ethercat 8 16-bit reg entries
            self.RegU = EsmacatCom.RegUnion()
            # Union index handler for getting union data
            self.getUI = EsmacatCom.UnionIndStruct()
            # Union index handler for getting union data
            self.setUI = EsmacatCom.UnionIndStruct()

            self.msgID = 0                          # Message ID
            self.msgID_last = 0                     # Last message ID
            self.msgTp = EsmacatCom.MessageType.MSG_NONE  # Message type
            self.msgFoot = [0, 0]                  # Message footer

            self.argLen = 0                       # Message argument length
            self.ArgU = EsmacatCom.RegUnion()    # Union for storing message arguments
            self.argUI = EsmacatCom.UnionIndStruct() # Union index handler for argument union data

            self.isNew = False                         # New message flag
            self.isErr = False                         # Message error flag
            self.errTp = EsmacatCom.ErrorType.ERR_NONE  # Message error type

    def __init__(self, suffix):
        # Initialize message handler instances
        self.sndEM = self.EcatMessageStruct()
        self.rcvEM = self.EcatMessageStruct()

        # Construct topic names using the provided suffix
        write_topic = f'/Esmacat_write_{suffix}'
        read_topic = f'Esmacat_read_{suffix}'

        # ROS Publisher: Initialize ethercat message handler instance
        self.maze_ard0_pub = rospy.Publisher(
            write_topic, ease_registers, queue_size=1)  # Dynamic topic name for publishing

        # ROS Subscriber: Initialize ethercat message handler instance
        rospy.Subscriber(
            read_topic, ease_registers, self._ros_callback, queue_size=1, tcp_nodelay=True)  # Dynamic topic name for subscribing

        # Store the time the class instance was initialized
        self.ts_ecat_init = rospy.get_time()  # (sec)

    # ------------------------ PRIVATE METHODS ------------------------

    def _uSetCheckReg(self, r_EM, reg_arr_si16):
        """
        Set register values in union and check for -1 values indicating no or incomplete messages

        Note: This is a workaround for the fact that the Esmacat does not clear and sometimes will be
        read midway through writing a message. This can lead to only partial or overlapping messages being read.

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            reg_arr_si16 (list): Register array (signed int16).
        """

        # Copy data to temporary union
        temp_u = EsmacatCom.RegUnion()
        for i_16 in range(8):
            temp_u.si16[i_16] = reg_arr_si16[i_16]

        # Bail if registry value is 0 or max int16 value suggesting registry is cleared or garbage
        if temp_u.ui16[0] == 0 or temp_u.ui16[0] == 65535:
            return False

        # Another check for garbage registry stuff at start of coms
        if (self.isEcatConnected != 1 and       # check if still waiting for handshake
            (temp_u.ui16[0] != 1 or    # directly check union id entry for first message
             temp_u.ui8[2] != 1)):      # directly check union type entry for handshake (e.g., msg_type_enum == 1)
            return False

        # Check for footer indicating a complete write from sender
        is_footer = False
        for i_8 in range(15):
            if temp_u.ui8[i_8] == 254 and temp_u.ui8[i_8 + 1] == 254:
                is_footer = True
                break
        if not is_footer:
            return False

        # Copy over temp union
        r_EM.RegU.ui64[0] = temp_u.ui64[0]
        r_EM.RegU.ui64[1] = temp_u.ui64[1]

        return True

    def _uSetMsgID(self, r_EM, msg_id=255):
        """
        Set message ID entry in union and update associated variable

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
        """

        # Get new message ID: itterate id and roll over to 1 if max 16-bit value is reached
        msg_id = r_EM.msgID + 1 if r_EM.msgID < 65535-1 else 1

        # Set message ID entry in union
        r_EM.RegU.ui16[r_EM.setUI.upd16(0)] = msg_id
        self._uGetMsgID(r_EM)  # copy from union to associated struct variable

    def _uGetMsgID(self, r_EM):
        """
        Get message ID from union

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct

        Returns:
            bool: True if message ID is valid (in sequence with last), False if not
        """

        r_EM.msgID_last = r_EM.msgID  # store last message ID if
        r_EM.msgID = r_EM.RegU.ui16[r_EM.getUI.upd16(0)]

        # Check/log error skipped or out of sequence messages if not first message or id has rolled over
        if r_EM.msgID != 1:
            if r_EM.msgID - r_EM.msgID_last != 1 and \
                    r_EM.msgID != r_EM.msgID_last:  # don't log errors for repeat message reads
                self._trackParseErrors(
                    r_EM, EsmacatCom.ErrorType.ECAT_ID_DISORDERED)
                return False
        return True

    def _uSetMsgType(self, r_EM, msg_type_enum):
        """
        Set message type entry in union and update associated variable

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            msg_type_enum (EsmacatCom.MessageType): Message type enum
        """

        # Set message type entry in union
        r_EM.RegU.ui8[r_EM.setUI.upd8(2)] = msg_type_enum.value
        # copy from union to associated struct variable
        self._uGetMsgType(r_EM)

    def _uGetMsgType(self, r_EM):
        """
        Get message type from union and check if valid

        Args:   
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct

        Returns:
            bool: True if message type is valid, False if not
        """

        # Get message type value
        msg_type_val = r_EM.RegU.ui8[r_EM.getUI.upd8(2)]

        # Check if 'msg_type_val' corresponds to any value in the 'EsmacatCom.MessageType' Enum.
        is_found = msg_type_val in [e.value for e in EsmacatCom.MessageType]
        r_EM.isErr = not is_found  # Update struct error flag

        # Log error and set message type to none if not found
        if r_EM.isErr:
            msg_type_val = EsmacatCom.MessageType.MSG_NONE
            self._trackParseErrors(
                r_EM, EsmacatCom.ErrorType.ECAT_NO_MSG_TYPE_MATCH)
        else:
            r_EM.msgTp = EsmacatCom.MessageType(msg_type_val)

        return is_found

    def _uSetErrType(self, r_EM, err_type_enum):
        """
        Set message type entry in union and update associated variable

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            err_type_enum (EsmacatCom.ErrorType): Error type enum  
        """

        # Set message type entry in union
        r_EM.RegU.ui8[r_EM.setUI.upd8(3)] = err_type_enum.value
        # copy from union to associated struct variable
        self._uGetErrType(r_EM)

    def _uGetErrType(self, r_EM):
        """
        Get message type from union and check if valid

        Args:   
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct

        Returns:
            bool: True if error type is valid, False if not
        """

        # Get message type value
        err_type_val = r_EM.RegU.ui8[r_EM.getUI.upd8(3)]

        # Check if 'err_type_val' corresponds to any value in the 'EsmacatCom.ErrorType' Enum.
        is_found = err_type_val in [e.value for e in EsmacatCom.ErrorType]
        r_EM.isErr = not is_found  # Update struct error flag

        # Log error and set error type to none if not found
        if r_EM.isErr:
            err_type_val = EsmacatCom.ErrorType.ERR_NONE
            self._trackParseErrors(
                r_EM, EsmacatCom.ErrorType.ECAT_NO_ERR_TYPE_MATCH)
        else:
            r_EM.errTp = EsmacatCom.ErrorType(err_type_val)

        return is_found

    def _uSetArgLength(self, r_EM, msg_arg_len):
        """
        Set message argument length entry in union and update associated variable

        Args:   
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct 
            msg_arg_len (int): Message argument length (unsigned int8)
        """

        # Set argument length union entry
        r_EM.RegU.ui8[r_EM.setUI.upd8(4)] = msg_arg_len
        # copy from union to associated struct variable
        self._uGetArgLength(r_EM)

    def _uGetArgLength(self, r_EM):
        """
        Get message argument length

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
        """

        # Get argument length union entry
        r_EM.argLen = r_EM.RegU.ui8[r_EM.getUI.upd8(4)]

    def _uSetArgData8(self, r_EM, msg_arg_data8):
        """
        Set message 8-bit argument data entry in union

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            msg_arg_data8 (unsigned int8): Message argument data (unsigned int8)
        """

        if isinstance(msg_arg_data8, int) and msg_arg_data8 <= 255:  # check for 8-bit int data

            # Increment argument union index
            r_EM.argUI.upd8()

            # Update message argument length from argument union 8-bit index
            self._uSetArgLength(r_EM, r_EM.argUI.ii8)

            # Get 8-bit union index 
            regu8_i = r_EM.argUI.ii8 + 4

            # Set message argument data in reg union
            r_EM.RegU.ui8[r_EM.setUI.upd8(regu8_i)] = msg_arg_data8
            # copy from union to associated struct variable
            self._uGetArgData8(r_EM)

        else:
            MazeDB.printMsg('WARNING', "Ecat: 8-bit argument data out of range: data[%d]", msg_arg_data8)

    def _uSetArgData16(self, r_EM, msg_arg_data16):
        """
        Set message 16-bit argument data entry in union

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            msg_arg_data16 (unsigned int16): Message argument data (unsigned int16)
        """

        if isinstance(msg_arg_data16, int) and msg_arg_data16 <= 65535:  # check for 16-bit int data

            # Increment argument union index
            r_EM.argUI.upd16()

            # Update message argument length from argument union 8-bit index
            self._uSetArgLength(r_EM, r_EM.argUI.ii8)

            # Get 16-bit union index
            regu16_i = (r_EM.argUI.ii8 + 4) // 2

            # Set message argument data in reg union
            r_EM.RegU.ui16[r_EM.setUI.upd16(regu16_i)] = msg_arg_data16
            # copy from union to associated struct variable
            self._uGetArgData8(r_EM)

        else:
            MazeDB.printMsg('WARNING', "Ecat: 16-bit argument data out of range: data[%d]", msg_arg_data16)

    def _uGetArgData8(self, r_EM):
        """
        Get reg union 8-bit message argument data and copy to arg union

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
        """

        self._uGetArgLength(r_EM)  # get argument length from union
        for i in range(r_EM.argLen):
            # copy to 8-bit argument Union
            r_EM.ArgU.ui8[i] = r_EM.RegU.ui8[r_EM.getUI.upd8()]

    def _uSetFooter(self, r_EM):
        """
        Set message footer entry in union and update associated variable

        Args:  
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
        """

        r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254
        r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254
        self._uGetFooter(r_EM)  # copy from union to associated struct variable

    def _uGetFooter(self, r_EM):
        """
        Get message footer from union

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct

        Returns:
            bool: True if footer is valid, False if not
        """

        # copy first footer byte
        r_EM.msgFoot[0] = r_EM.RegU.ui8[r_EM.getUI.upd8()]
        # copy second footer byte
        r_EM.msgFoot[1] = r_EM.RegU.ui8[r_EM.getUI.upd8()]

        # Log missing footer error
        if r_EM.msgFoot[0] != 254 or r_EM.msgFoot[1] != 254:  # check if footer is valid
            self._trackParseErrors(
                r_EM, EsmacatCom.ErrorType.ECAT_MISSING_FOOTER)
            return False
        return True

    def _uReset(self, r_EM):
        """Reset union data and indices"""

        # Reset union data to 0
        r_EM.RegU.ui64[0] = 0
        r_EM.RegU.ui64[1] = 0

        # Reset union indices
        r_EM.setUI.reset()
        r_EM.getUI.reset()
        r_EM.argUI.reset()

    def _resetReg(self):
        """
        Reset EtherCAT register data by setting all values to -1

        Note: This is a workaround for the fact that the Esmacat does not clear
        this can lead to only partial or overlapping messages being read
        """

        reg_arr = [-1] * 8
        self.maze_ard0_pub.publish(*reg_arr)

    def _trackParseErrors(self, r_EM, err_tp, do_reset=False):
        """
        Check for and log any Ecat message parsing errors

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            err_tp (EsmacatCom.ErrorType): Error type enum
            do_reset (bool): Reset error type flag if true (Optional)
        """

        # Check for error
        if not do_reset:

            # Run only once
            if r_EM.errTp != err_tp:

                # Set error type and flag
                r_EM.errTp = err_tp
                r_EM.isErr = True

                # Print message as warning
                MazeDB.printMsg(
                    'WARNING', "Ecat: %s: id new[%d] id last[%d] type[%d][%s]", r_EM.errTp.name, r_EM.msgID, r_EM.msgID_last, r_EM.msgTp.value, r_EM.msgTp.name)
                self._printEcatReg('WARNING', r_EM.RegU)

        # Unset error type
        elif r_EM.errTp == err_tp:
            r_EM.errTp = EsmacatCom.ErrorType.ERR_NONE
            r_EM.isErr = False

    def _printEcatReg(self, level, reg_u):
        """
        Print EtherCAT register data

        Args:
            level (str): ROS log level
            reg_u (EsmacatCom.RegUnion): EtherCAT register union
        """

        # Print message data
        MazeDB.printMsg(level, "\t Ecat 16-Bit Register:")
        for i in range(8):
            MazeDB.printMsg(level, "\t\t ui16[%d] [%d]", i, reg_u.ui16[i])
        MazeDB.printMsg(level, "\t Ecat 8-Bit Register:")
        for i in range(8):
            if i < 5:
                MazeDB.printMsg(level, "\t\t ui8[%d][%d]   [%d][%d]", 2 * i,
                                2 * i + 1, reg_u.ui8[2 * i], reg_u.ui8[2 * i + 1])
            else:
                MazeDB.printMsg(level, "\t\t ui8[%d][%d] [%d][%d]", 2 * i,
                                2 * i + 1, reg_u.ui8[2 * i], reg_u.ui8[2 * i + 1])

    def _ros_callback(self, msg):
        """ ROS callback for Esmacat_read topic """

        # Wait for interface to initialize
        current_time = rospy.get_time()
        if (current_time - self.ts_ecat_init) < self.dt_ecat_start:  # Less than 100ms
            return

        # Store ethercat message in class variable
        reg_arr_si16 = [0]*8
        reg_arr_si16[0] = msg.INT0
        reg_arr_si16[1] = msg.INT1
        reg_arr_si16[2] = msg.INT2
        reg_arr_si16[3] = msg.INT3
        reg_arr_si16[4] = msg.INT4
        reg_arr_si16[5] = msg.INT5
        reg_arr_si16[6] = msg.INT6
        reg_arr_si16[7] = msg.INT7

        # Check for new messages
        self._readEcatMessage(reg_arr_si16)

    def _readEcatMessage(self, reg_arr_si16):
        """
        Used to parse new incoming ROS ethercat msg data.

        Args:
            reg_arr_si16 (list): Register array (signed int16).

        Returns:
            int: new message flag [0:no message, 1:new message].
        """

        # Bail if previous message not processed
        if self.rcvEM.isNew:
            return False

        # Check register for garbage or incomplete data and copy register data into union
        if not self._uSetCheckReg(self.rcvEM, reg_arr_si16):
            return False

        # Get message id and check for out of sequence messages
        if not self._uGetMsgID(self.rcvEM):
            return False

        # Skip redundant messages
        if self.rcvEM.msgID == self.rcvEM.msgID_last:
            return False

        # Get message type and check if not valid
        if not self._uGetMsgType(self.rcvEM):
            return False

        # Get error type and flag if not valid
        self._uGetErrType(self.rcvEM)

        # Get argument length and arguments
        self._uGetArgData8(self.rcvEM)

        # Get footer and check if not found
        if not self._uGetFooter(self.rcvEM):
            return False

        # Set new message flag
        self.rcvEM.isNew = True

        # Print message
        MazeDB.printMsg('INFO', "(%d)ECAT ACK RECEIVED: %s",
                        self.rcvEM.msgID, self.rcvEM.msgTp.name)
        self._printEcatReg('DEBUG', self.rcvEM.RegU)  # TEMP

        return True

    # ------------------------ PUBLIC METHODS ------------------------

    def writeEcatMessage(self, msg_type_enum, msg_arg_data_i8=None, msg_arg_data_i16=None, do_print=True):
        """
        Used to send outgoing ROS ethercat msg data.

        Args:
            msg_type_enum (EsmacatCom.MessageType): Message type enum.
            msg_arg_data_i8 (list or scalar): 8-bit message argument data array.
            msg_arg_data_i16 (list or scalar): 16-bit message argument data array.
            do_print (bool): Print message to console if true (Optional).

        Returns:
            int: Success/error codes [0:no message, 1:new message, 2:error]
        """

        # Set all register values to -1 to clear buffer
        self._resetReg()

        # Reset union variables
        self._uReset(self.sndEM)

        # Store new message id
        self._uSetMsgID(self.sndEM)

        # Store new message type
        self._uSetMsgType(self.sndEM, msg_type_enum)

        # 	------------- Store arguments -------------

        # Store 8-bit message argument
        if msg_arg_data_i8 is not None:

            # Store scalar message argument
            if not isinstance(msg_arg_data_i8, list):
                self._uSetArgData8(self.sndEM, msg_arg_data_i8)

            # Store list of arguments
            else:
                for i in range(len(msg_arg_data_i8)):
                    self._uSetArgData8(self.sndEM, msg_arg_data_i8[i])

        # Store 16-bit message argument
        if msg_arg_data_i16 is not None:

             # Store scalar message argument in list
            if not isinstance(msg_arg_data_i16, list):
                msg_arg_data_i16 = [msg_arg_data_i16]

            # Copy 16-bit argument data to union 8-bit entries
            temp_U = EsmacatCom.RegUnion()
            for i in range(len(msg_arg_data_i16)):
                
                # Copy to temp union
                temp_U.ui16[0] = msg_arg_data_i16[i]

                # Copy to 8-bit union
                self._uSetArgData8(self.sndEM, temp_U.ui8[0])
                self._uSetArgData8(self.sndEM, temp_U.ui8[1])
                
        # set arg length to 0 if no message arguments provided
        if msg_arg_data_i8 is None and msg_arg_data_i16 is None:
            self._uSetArgLength(self.sndEM, 0)

        # 	------------- Finish setup and write -------------

        # Store footer
        self._uSetFooter(self.sndEM)

        # Publish to union uint16 type data to ease_registers topic
        self.maze_ard0_pub.publish(*self.sndEM.RegU.si16)

        # Print message
        if do_print:
            MazeDB.printMsg('INFO', "(%d)ECAT SENT: %s",
                            self.sndEM.msgID, self.sndEM.msgTp.name)
            self._printEcatReg('DEBUG', self.sndEM.RegU) # TEMP

    def resetEcat(self):
        """Reset all message structs."""

        # Reset EtherCAT register data
        self._resetReg()

        # Reset message union data and indeces
        self._uReset(self.sndEM)
        self._uReset(self.rcvEM)

        # Reset message counters
        self.sndEM.msgID = 0
        self.rcvEM.msgID = 0

        # Setup Ethercat handshake flag
        self.isEcatConnected = False