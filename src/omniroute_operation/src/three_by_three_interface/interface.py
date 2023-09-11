#!/usr/bin/env python

# ======================== PACKAGES ========================
# Standard Library Imports
import os
import glob
import subprocess
import time
import math
import csv
import ctypes
from typing import List
from enum import Enum

# Third-party Imports
import numpy as np
from scipy.io import loadmat
from colorama import Fore, Style

# ROS Imports
import rospy
from std_msgs.msg import *
from omniroute_esmacat_ros.msg import *
from omniroute_operation.msg import *

# PyQt and PySide Imports
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsItemGroup, QGraphicsLineItem, QGraphicsTextItem
from PyQt5.QtCore import Qt, QRectF, QCoreApplication
from PyQt5.QtGui import QPen, QColor, QFont
from python_qt_binding import loadUi, QtOpenGL
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from qt_gui.plugin import Plugin

# It seems you have some redundant imports, especially from python_qt_binding and PyQt5. You should remove
# the ones that are not necessary to avoid confusion and make the code cleaner.


# ======================== GLOBAL VARS ========================

# Maze configuration variables
DB_VERBOSE = True  # debug verbose flag
NUM_ROWS_COLS = 3  # number of rows and columns in maze
WALL_MAP = {  # wall map for 3x3 maze [chamber_num][wall_num]
    0: [0, 1, 2, 3, 4, 5, 6, 7],
    1: [1, 3, 5, 7, 2],
    2: [0, 1, 2, 3, 4, 5, 6, 7],
    3: [1, 3, 5, 7, 0],
    4: [0, 1, 2, 3, 4, 5, 6, 7],
    5: [1, 3, 5, 7, 4],
    6: [0, 1, 2, 3, 4, 5, 6, 7],
    7: [1, 3, 5, 7, 6],
    8: [0, 1, 2, 3, 4, 5, 6, 7]
}

# ======================== GLOBAL CLASSES ========================


class EsmacatCom:
    """ 
    This class is used to communicate with the arduino via ethercat.
    It is used to send and receive messages from the arduino.
    """

    # ------------------------ CLASS VARIABLES ------------------------

    # Handshake finished flag
    isEcatConnected = False

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
            b_i = b_i if b_i != 255 else self.ii8
            self.ii8 = b_i + 1
            self.ii16 = self.ii8 // 2
            return b_i

        def upd16(self, b_i=255):
            """Update union 16-bit and 8-bit index and return last updated 16-bit index"""
            b_i = b_i if b_i != 255 else self.ii16
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
            # Union index handler for argument union data
            self.argUI = EsmacatCom.UnionIndStruct()

            self.isNew = False                         # New message flag
            self.isErr = False                         # Message error flag
            self.errTp = EsmacatCom.ErrorType.ERR_NONE  # Message error type

    def __init__(self):

        # Initialize ethercat message handler instances
        # Initialize message handler instance for sending messages
        self.sndEM = self.EcatMessageStruct()
        # Initialize message handler instance for receiving messages
        self.rcvEM = self.EcatMessageStruct()

        # ROS Publisher: Initialize ethercat message handler instances
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)  # Esmacat write maze ard0 ease publisher

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

            # Get 8-bit union index and set 8-bit argument data entry in union
            regu8_i = r_EM.argUI.ii8 + 4

            # Set message argument data in reg union
            r_EM.RegU.ui8[r_EM.setUI.upd8(regu8_i)] = msg_arg_data8
            # copy from union to associated struct variable
            self._uGetArgData8(r_EM)

    def _uSetArgData16(self, r_EM, msg_arg_data16):
        """
        Set message 16-bit argument data entry in union

        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            msg_arg_data16 (unsigned int16): Message argument data (unsigned int16)
        """

        if isinstance(msg_arg_data16, int) and msg_arg_data16 > 255:  # check for 16-bit int data

            # Increment argument union index
            r_EM.argUI.upd16()

            # Update message argument length from argument union 8-bit index
            self._uSetArgLength(r_EM, r_EM.argUI.ii8)

            # Get 16-bit union index and set 16-bit argument data entry in union
            regu16_i = (r_EM.argUI.ii8 + 4) // 2

            # Set message argument data in reg union
            r_EM.RegU.ui16[r_EM.setUI.upd16(regu16_i)] = msg_arg_data16
            # copy from union to associated struct variable
            self._uGetArgData8(r_EM)

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

    # ------------------------ PUBLIC METHODS ------------------------

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

    def readEcatMessage(self, reg_arr_si16):
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

    def writeEcatMessage(self, msg_type_enum, msg_arg_data=None):
        """
        Used to send outgoing ROS ethercat msg data.

        Args:
            msg_type_enum (EsmacatCom.MessageType): Message type enum.
            msg_arg_data_arr (list or scalar): Message argument data array.

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

        # Convert scalar value to list if it's not already a list
        if msg_arg_data is not None and not isinstance(msg_arg_data, list):
            msg_arg_data = [msg_arg_data]

        if msg_arg_data is not None:  # store message arguments if provided
            for i in range(len(msg_arg_data)):
                self._uSetArgData8(self.sndEM, msg_arg_data[i])
        else:
            self._uSetArgLength(self.sndEM, 0)  # set arg length to 0

        # 	------------- Finish setup and write -------------

        # Store footer
        self._uSetFooter(self.sndEM)

        # Publish to union uint16 type data to ease_registers topic
        self.maze_ard0_pub.publish(*self.sndEM.RegU.si16)

        # Print message
        MazeDB.printMsg('INFO', "(%d)ECAT SENT: %s",
                        self.sndEM.msgID, self.sndEM.msgTp.name)
        self._printEcatReg('DEBUG', self.sndEM.RegU)


class MazeDB(QGraphicsView):
    """ MazeDebug class to plot the maze """

    @classmethod
    def printMsg(cls, level, msg, *args):
        """ Log to ROS in color """

        # Exit if DB_VERBOSE is false
        if not DB_VERBOSE:
            return

        # Log message by type and color to ROS
        if level == 'ATTN':
            # Add '=' header and footer for ATTN condition
            f_msg = cls._frmt_msg(Fore.BLUE, msg, *args)
            n = max(0, 30 - len(f_msg) // 2)
            f_msg = '=' * n + " " + f_msg + " " + '=' * n
            rospy.loginfo(f_msg)
        elif level == 'INFO':
            rospy.loginfo(cls._frmt_msg(Fore.BLUE, msg, *args))
        elif level == 'ERROR':
            rospy.logerr(cls._frmt_msg(Fore.RED, msg, *args))
        elif level == 'WARNING':
            rospy.logwarn(cls._frmt_msg(Fore.YELLOW, msg, *args))
        elif level == 'DEBUG':
            rospy.loginfo(cls._frmt_msg(Fore.GREEN, msg, *args))
        else:
            rospy.loginfo(cls._frmt_msg(Fore.BLACK, msg, *args))

    @classmethod
    def _frmt_msg(cls, color, msg, *args):
        """ Format message with color """

        colored_message = f"{color}{msg}{Style.RESET_ALL}"
        return colored_message % args

    def arrStr(input_list):
        """Converts a list/array to a string"""

        result = "[" + ",".join(map(str, input_list)) + "]"
        return result


class WallConfig:
    """ 
    Used to stores the wall configuration of the maze for CSV and Ethercat for the maze.
    """

    # ------------------------ CLASS VARIABLES ------------------------

    # Stores the wall configuration list
    cw_wall_num_list = []  # cham_num x [wall_num]
    cw_wall_byte_list = []  # cham_num x wall_byte

    # ------------------------ CLASS METHODS ------------------------

    @classmethod
    def reset(cls):
        """Resets the wall configuration list"""

        cls.cw_wall_num_list = []
        cls.cw_wall_byte_list = []

    @classmethod
    def get_len(cls):
        """Returns the number of entries in the wall configuration list"""

        return len(cls.cw_wall_num_list)

    @classmethod
    def add_wall(cls, chamber_num, wall_num):
        """Adds a wall to the wall configuration list"""
        for item in cls.cw_wall_num_list:
            if item[0] == chamber_num:
                if wall_num not in item[1]:
                    item[1].append(wall_num)
                    return
        cls.cw_wall_num_list.append([chamber_num, [wall_num]])

    @classmethod
    def remove_wall(cls, chamber_num, wall_num):
        """Removes a wall from the wall configuration list"""
        for item in cls.cw_wall_num_list[:]:  # Iterate over a copy of the list
            if item[0] == chamber_num:
                if wall_num in item[1]:
                    item[1].remove(wall_num)
                    if not item[1]:  # If the second column is empty, remove the entire row
                        cls.cw_wall_num_list.remove(item)
                    return

    @classmethod
    def make_byte2num_cw_list(cls, _cw_wall_byte_list, do_print=False):
        """
        Used to convert imported CSV with wall byte mask values to a list with wall numbers

        Args:
            _cw_wall_byte_list (list): 2D list: col_1 = chamber number, col_2 = wall byte mask

        Returns:
            2D list: col_1 = chamber number, col_2 = nested wall numbers
        """

        # Clear/reset the existing wall_config_list
        cls.reset()

        # Convert the byte values to arrays and update the wall_config_list
        for row in _cw_wall_byte_list:
            chamber_num = row[0]
            byte_value = row[1]

            # Convert the byte_value back to an array of wall numbers
            wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

            cls.cw_wall_num_list.append([chamber_num, wall_numbers])

        # loop thorugh wall_byte_config_list and print each element
        if do_print:
            for row in cls.cw_wall_num_list:
                MazeDB.logMsg('DEBUG', "[%d][%d]", row[0], row[1])

        return cls.cw_wall_num_list

    @classmethod
    def make_num2byte_cw_list(cls):
        """
        Used to covert wall number arrays to byte values for saving to CSV

        Returns:
            2D list: col_1 = chamber number, col_2 = wall byte mask
        """

        cls.cw_wall_byte_list = []

        for row in cls.cw_wall_num_list:  # row = [chamber_num, wall_numbers]
            chamber_num = row[0]
            wall_arr = row[1]
            # Initialize the byte value
            byte_value = 0
            # Iterate over the array of values
            for wall_i in wall_arr:
                if 0 <= wall_i <= 7:
                    # Set the corresponding bit to 1 using bitwise OR
                    byte_value |= (1 << wall_i)
            cls.cw_wall_byte_list.append([chamber_num, byte_value])

        return cls.cw_wall_byte_list

    @classmethod
    def get_wall_byte_list(cls):
        """
        Used to generate a 1D list with only byte values for each chamber corespoinding to the wall configuration
        For use with the EsmacatCom class

        Returns: 
            1D list with byte values for all chambers
        """

        cls.cw_wall_byte_list = cls.make_num2byte_cw_list()

        # Update U_arr with corresponding chamber and wall byte
        _wall_byte_list = [0] * len(WALL_MAP)
        # wall_arr = [0] * len(cls.wallConfigList)
        for cw in cls.cw_wall_byte_list:
            _wall_byte_list[cw[0]] = cw[1]

        return _wall_byte_list

    @classmethod
    def _sort_entries(cls):
        """Sorts the entries in the wall configuration list by chamber number and wall numbers"""

        # Sort the rows by the entries in the based on the first chamber number
        cls.cw_wall_num_list.sort(key=lambda row: row[0])

        # Sort the arrays in the second column
        for row in cls.cw_wall_num_list:
            row[1].sort()

    @classmethod
    def __iter__(cls):
        """Returns an iterator for the wall configuration list"""
        return iter(cls.cw_wall_num_list)

    @classmethod
    def __str__(cls):
        """Returns the wall configuration list as a string"""
        return str(cls.cw_wall_num_list)


class MazePlot(QGraphicsView):
    """ MazePlot class to plot the maze """

    # ------------------------ NESTED CLASSES ------------------------

    class Status(Enum):
        """ 
        Enum for tracking status of the maze hardware 
        This is a special type of enum where each enum has an associated QColor

        """

        EXCLUDED = (0, QColor(255, 255, 255))  # White
        UNINITIALIZED = (1, QColor(200, 200, 200))  # Gray
        INITIALIZED = (2, QColor(0, 0, 0))         # Black
        WARNING = (3, QColor(191, 124, 0))         # 25% Darker Orange
        ERROR = (4, QColor(191, 0, 0))             # 25% Darker Red
        DOWN = (5, QColor(0, 0, 0))                # Black
        UP = (6, QColor(0, 191, 0))                # 25% Darker Green

        def __new__(cls, value, color):
            obj = object.__new__(cls)
            obj._value_ = value
            obj.color = color
            return obj

        def __int__(self):
            return self.value

    # CLASS: Wall
    class Wall(QGraphicsItemGroup):
        def __init__(self, _chamber_num, _wall_num, _p0, _p1, _wall_width, _label_pos=None, parent=None):
            super().__init__(parent)

            # Store arguments parameters
            self.chamber_num = _chamber_num
            self.wall_num = _wall_num

            # Plot wall line
            self.line = QGraphicsLineItem(
                QLineF(_p0[0], _p0[1], _p1[0], _p1[1]))
            pen = QPen()
            pen.setWidth(_wall_width)
            self.line.setPen(pen)
            self.addToGroup(self.line)

            # Plot wall numbers
            self.label = QGraphicsTextItem(str(_wall_num))
            font_size = _wall_width
            self.label.setFont(QFont("Arial", font_size, QFont.Bold))
            self.label.setPos(_label_pos[0], _label_pos[1])
            self.addToGroup(self.label)
            # Center text
            MazePlot._centerText(self.label, _label_pos[0], _label_pos[1])

            # Create wall status var and set status
            self.status = MazePlot.Status.UNINITIALIZED
            self.setStatus(MazePlot.Status.UNINITIALIZED)

        def setStatus(self, status_enum, do_force=False):
            """
            Set/update wall status and set UI object colors

            Args:
                status_enum (MazePlot.Status): Status enum
                do_force (bool): Force status update if true (Optional)
            """

            # Check and set the new status
            if not MazePlot.checkStatus(self.status, status_enum, do_force):
                return
            self.status = status_enum

            # Set text color
            self.label.setDefaultTextColor(self.status.color)

            # Set wall color
            current_pen = self.line.pen()
            current_pen.setColor(self.status.color)
            self.line.setPen(current_pen)

        def togglePosStatus(self):
            """Toggle the wall status between UP and DOWN"""

            if self.status == MazePlot.Status.DOWN or \
                    self.status == MazePlot.Status.INITIALIZED:
                self.setStatus(MazePlot.Status.UP)
            elif self.status == MazePlot.Status.UP:
                self.setStatus(MazePlot.Status.DOWN)

        def mousePressEvent(self, event):
            """Handles mouse press events and sets the wall status"""

            # Bail if wall is not enabled
            if not MazePlot.isEnabled(self.status):
                return

            if event.button() == Qt.LeftButton:

                # Toggle wall status
                self.togglePosStatus()

                # Update configuration list
                if self.status == MazePlot.Status.UP:  # add list entry
                    WallConfig.add_wall(self.chamber_num, self.wall_num)
                else:  # remove list entry
                    WallConfig.remove_wall(self.chamber_num, self.wall_num)

    # CLASS: Chamber
    class Chamber(QGraphicsItemGroup):
        def __init__(self, _chamber_num, _center_x, _center_y, _chamber_width, _wall_width, parent=None):
            super().__init__(parent)

            # Store arguments parameters
            self.chamber_num = _chamber_num
            self.center_x = _center_x
            self.center_y = _center_y
            self.chamber_width = _chamber_width

            # Compute chamber octogon parameters
            octagon_vertices = self.getOctagonVertices(
                _center_x, _center_y, _chamber_width/2, math.radians(22.5))
            octagon_points = [QPointF(i[0], i[1]) for i in octagon_vertices]
            wall_angular_offset = 2*math.pi/32  # This decides the angular width of the wall
            wall_vertices_0 = self.getOctagonVertices(
                _center_x, _center_y, _chamber_width/2, -math.pi/8+wall_angular_offset)
            wall_vertices_1 = self.getOctagonVertices(
                _center_x, _center_y, _chamber_width/2, -math.pi/8-wall_angular_offset)
            wall_label_pos = self.getOctagonVertices(
                _center_x, _center_y, _chamber_width/2.75, 0)

            # Initialize wall instances
            self.Walls = [MazePlot.Wall(_chamber_num=_chamber_num,
                                        _wall_num=i,
                                        _p0=wall_vertices_0[i],
                                        _p1=wall_vertices_1[i+1],
                                        _wall_width=_wall_width,
                                        _label_pos=wall_label_pos[i])
                          for i in range(8)]

            # Plot backround chamber octogons
            self.octagon = QGraphicsPolygonItem(QPolygonF(octagon_points))
            self.octagon.setBrush(QBrush(QColor(240, 240, 240)))
            self.addToGroup(self.octagon)

            # Plot cahamber numbers
            self.label = QGraphicsTextItem(str(_chamber_num))
            font_size = _wall_width*1.75
            self.label.setFont(QFont("Arial", font_size, QFont.Bold))
            self.addToGroup(self.label)
            # Center the text over the chamber's center
            MazePlot._centerText(self.label, _center_x, _center_y)

            # Create chamber status status var and set status
            self.status = MazePlot.Status.UNINITIALIZED
            self.setStatus(MazePlot.Status.UNINITIALIZED)

        def getOctagonVertices(self, x, y, w, offset):
            vertices_list = [(round(x + w*math.cos(i)), round(y+w*math.sin(i)))
                             for i in np.linspace(math.pi, 3*math.pi, 9) + offset]
            return vertices_list

        def setStatus(self, status_enum, do_force=False):
            """
            Set/update chamber status and set UI object colors

            Args:
                status_enum (MazePlot.Status): Status enum
                do_force (bool): Force status update if true (Optional)
            """

            # Check and set the new status
            if not MazePlot.checkStatus(self.status, status_enum, do_force):
                return
            self.status = status_enum

            # Set text color
            self.label.setDefaultTextColor(self.status.color)

            # Set chamber color
            # self.octagon.setBrush(QBrush(self.status.color))

        def mousePressEvent(self, event):
            """Handles mouse press events and sets the chamber status"""
            # @todo: Figure out why this is not working
            MazeDB.printMsg('DEBUG', "Chamber %d clicked", self.chamber_num)

            # Bail if chamber is not enabled
            if not MazePlot.isEnabled(self.status):
                return

            if event.button() == Qt.LeftButton:

                # Change state setting of all walls
                for _, wall in enumerate(self.Walls):
                    wall.togglePosStatus()

    # ------------------------ METHODS ------------------------

    # CLASS: Maze
    class Maze(QGraphicsItemGroup):
        def __init__(self, _num_rows_cols, _chamber_width, _wall_width, parent=None):
            super().__init__(parent)

            # Store arguments
            self.num_rows_cols = _num_rows_cols
            self.chamber_width = _chamber_width

            # Store and compute graphical parameters
            maze_width = self.chamber_width * self.num_rows_cols
            maze_height = self.chamber_width * self.num_rows_cols
            half_width = _chamber_width/2
            x_pos = np.linspace(half_width, int(
                maze_width - half_width),  self.num_rows_cols)
            y_pos = np.linspace(half_width, int(
                maze_height - half_width),  self.num_rows_cols)

            # Create a list of chamber instances
            self.Chambers = []
            k = 0
            for y in y_pos:
                for x in x_pos:
                    self.Chambers.append(
                        MazePlot.Chamber(_chamber_num=k, _center_x=x, _center_y=y, _chamber_width=_chamber_width, _wall_width=_wall_width))
                    k = k+1

            # # Create status text label
            # self.label = QGraphicsTextItem("TESTING")
            # font_size = _wall_width*1.75
            # self.label.setFont(QFont("Arial", font_size, QFont.Bold))
            # self.addToGroup(self.label)
            # MazePlot._centerText(self.label, 10, 10)

            # Create maze status var and set status
            self.status = MazePlot.Status.UNINITIALIZED
            self.setStatus(MazePlot.Status.UNINITIALIZED)

        def setStatus(self, status_enum, do_force=False):
            """
            Set/update maze status and set UI object colors

            Args:
                status_enum (MazePlot.Status): Status enum
                do_force (bool): Force status update if true (Optional)
            """

            # Check and set the new status
            if not MazePlot.checkStatus(self.status, status_enum, do_force):
                return
            self.status = status_enum

            # # Set status text and color color
            # self.label.setDefaultTextColor(self.status.color)
            # self.label.setPlainText(self.status.name)

        def updatePlotFromWallConfig(self):
            """Updates the plot based on the wall configuration list"""

            for chamber in self.Chambers:
                for wall in chamber.Walls:

                    # Check if there is an entry for the current chamber and wall in WallConfig
                    chamber_num = chamber.chamber_num
                    wall_num = wall.wall_num
                    entry_found = any(
                        chamber_num == entry[0] and wall_num in entry[1] for entry in WallConfig.cw_wall_num_list)

                    # Set the wall status to up if entry found
                    if entry_found:
                        wall.setStatus(MazePlot.Status.UP)
                    else:
                        wall.setStatus(MazePlot.Status.DOWN)

    def _centerText(text_item, center_x, center_y):
        """Centers the text item over the given coordinates"""

        # Set the text item's position relative to its bounding rectangle
        # Allow the text item to resize its width automatically
        text_item.setTextWidth(0)
        text_item.setHtml(
            '<div style="text-align: center; vertical-align: middle;">{}</div>'.format(text_item.toPlainText()))

        # Get the bounding rectangle of the text item
        text_rect = text_item.boundingRect()

        # Center the text both vertically and horizontally over the given coordinates
        x_pos = center_x - text_rect.width() / 2
        y_pos = center_y - text_rect.height() / 2
        text_item.setPos(x_pos, y_pos)

    def checkStatus(current_status_enum, new_status_enum, do_force=False):
        """
        Checks if the new status is valid and updates the status if it is

        Args:
            current_status_enum (MazePlot.Status): Current status enum
            new_status_enum (MazePlot.Status): New status enum to set
            do_force (bool): Force status update if true (Optional)
        """

        # Check if change is forced
        if do_force:
            return True

        # Check if status is valid based on following rules
        if current_status_enum == MazePlot.Status.EXCLUDED:
            return False
        elif current_status_enum == MazePlot.Status.ERROR:
            return False
        elif current_status_enum == MazePlot.Status.UNINITIALIZED:
            if new_status_enum != MazePlot.Status.UNINITIALIZED and \
                    new_status_enum != MazePlot.Status.INITIALIZED and \
                    new_status_enum != MazePlot.Status.ERROR and \
                    new_status_enum != MazePlot.Status.WARNING:
                return False
        return True

    def isEnabled(current_status_enum):
        """ Checks if instance is enabled """

        return current_status_enum == MazePlot.Status.INITIALIZED or \
            current_status_enum == MazePlot.Status.DOWN or \
            current_status_enum == MazePlot.Status.UP

# ======================== MAIN UI CLASS ========================


class Interface(Plugin):
    """ Interface plugin """

    # ------------------------ CLASS VARIABLES ------------------------

    # Define signals
    signal_Esmacat_read_maze_ard0_ease = Signal()

    # ------------------------ NESTED CLASSES ------------------------

    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

        # ................ QT UI Setup ................

        # Give QObjects reasonable names
        self.setObjectName('Interface')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Extend the widget with all attributes and children from UI file
        loadUi(os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'interface.ui'), self._widget)

        self._widget.setObjectName('InterfacePluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.plotMazeView.setViewportUpdateMode(
            QGraphicsView.FullViewportUpdate)
        self.scene = QGraphicsScene()
        self._widget.plotMazeView.setScene(self.scene)

        # Set the fixed size of the main window based on the dimensions from the UI file
        main_window_width = self._widget.geometry().width()
        main_window_height = self._widget.geometry().height()
        self._widget.setFixedSize(main_window_width, main_window_height)

        # Set the size hint of the main window to match the size of the _widget
        self._widget.window().setMinimumSize(main_window_width, main_window_height)
        self._widget.window().setMaximumSize(main_window_width, main_window_height)

        # Set the background color of the scene to white
        self._widget.plotMazeView.setBackgroundBrush(QColor(255, 255, 255))
        self._widget.plotMazeView.setViewport(QtOpenGL.QGLWidget())

        # Move the window
        self.move_ui_window(
            self._widget, horizontal_alignment='left', vertical_alignment='top')

        # Get the absolute path of the current script file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Specify the defualt directory
        config_dir_default = os.path.abspath(os.path.join(
            script_dir, '..', '..', '..', '..', 'config', 'paths'))
        # Set to default config file path
        self._widget.fileDirEdit.setText(config_dir_default)

        # Initialize file list and index
        self.current_file_index = 0  # set to zero
        self.csv_files = []

        # Create an array of system setting edit boxes
        self.sys_widgets = [
            self._widget.sysSettingEdit_1,  # Number of chambers to initialize in maze
            self._widget.sysSettingEdit_2,  # Max chamb to move per block
            self._widget.sysSettingEdit_3,  # Max number of attempts to move a walls
            self._widget.sysSettingEdit_4,  # PWM duty cycle for wall motors
            self._widget.sysSettingEdit_5,  # Timeout for wall movement (ms)
        ]

        # ................ Maze Setup ................

        # Default system settings [default][min][max]
        self.sysDefaults = [
            [9, 1, 9],          # Num chamb init
            [3, 1, 1],          # Max chamb to move per block
            [2, 1, 3],          # Max move attempt
            [255, 0, 255],      # PWM duty
            [1000, 500, 2000]   # Move timeout (ms)
        ]

        # Setup/initialize system settings edit boxes
        self.getParamTxtBox()

        # Calculate chamber width and wall line width and offset
        chamber_width = self._widget.plotMazeView.width()*0.9/NUM_ROWS_COLS
        wall_width = chamber_width*0.1

        # Create MazePlot and populate walls according to WALL_MAP
        self.MP = MazePlot.Maze(_num_rows_cols=NUM_ROWS_COLS,
                                _chamber_width=chamber_width,
                                _wall_width=wall_width)

        # Add chambers and disable walls entries that do not exist in that chamber
        for cham_i, chamber in enumerate(self.MP.Chambers):
            self.scene.addItem(chamber)
            for wall_i, wall in enumerate(chamber.Walls):
                if wall_i not in WALL_MAP[cham_i]:
                    # set status to excluded
                    wall.setStatus(MazePlot.Status.EXCLUDED)
                    wall.setEnabled(False)  # disable wall graphics object
                    wall.setVisible(False)  # hide wall graphics object

        # Add walls - this is a new loop so that they are drawn above the chambers
        for chamber in self.MP.Chambers:
            for wall in chamber.Walls:
                self.scene.addItem(wall)

        # ................ Ecat Setup ................

        # Create EsmacatCom object
        self.EsmaCom = EsmacatCom()

        # Store the time the interface was initialized
        self.ts_interface_init = rospy.get_time()  # (sec)

        # Specify delay to start and check reading/writing Esmacat data
        self.dt_ecat_start = 1  # (sec)
        self.dt_ecat_check = 0.5  # (sec)

        # Couter to incrementally shut down opperations
        self.cnt_shutdown_step = 0  # tracks the current step
        self.cnt_shutdown_ack_check = 0  # tracks number of times ack has been checked
        self.dt_shutdown_step = 0.25  # (sec)

        # ................ QT Callback Setup ................

        # QT UI wall config button callback setup
        self._widget.fileBrowseBtn.clicked.connect(
            self.qt_callback_fileBrowseBtn_clicked)
        self._widget.filePreviousBtn.clicked.connect(
            self.qt_callback_filePreviousBtn_clicked)
        self._widget.fileNextBtn.clicked.connect(
            self.qt_callback_fileNextBtn_clicked)
        self._widget.plotClearBtn.clicked.connect(
            self.qt_callback_plotClearBtn_clicked)
        self._widget.plotSaveBtn.clicked.connect(
            self.qt_callback_plotSaveBtn_clicked)
        self._widget.plotSendBtn.clicked.connect(
            self.qt_callback_plotSendBtn_clicked)
        # QT UI system button callback setup
        self._widget.sysStartBtn.clicked.connect(
            self.qt_callback_sysStartBtn_clicked)
        self._widget.sysReinitBtn.clicked.connect(
            self.qt_callback_sysReinitBtn_clicked)
        self._widget.sysQuiteBtn.clicked.connect(
            self.qt_callback_sysQuiteBtn_clicked)
        # Other object callbacks
        self._widget.fileListWidget.itemClicked.connect(
            self.qt_callback_fileListWidget_item_clicked)

        # Disable all but start and quit buttons
        self._widget.sysReinitBtn.setEnabled(False)
        self._widget.fileBrowseBtn.setEnabled(False)
        self._widget.filePreviousBtn.setEnabled(False)
        self._widget.fileNextBtn.setEnabled(False)
        self._widget.plotClearBtn.setEnabled(False)
        self._widget.plotSaveBtn.setEnabled(False)
        self._widget.plotSendBtn.setEnabled(False)

        # QT timer setup for UI updating
        self.timer_updateUI = QTimer()
        self.timer_updateUI.timeout.connect(self.timer_callback_updateUI_loop)
        self.timer_updateUI.start(20)  # set incriment (ms)

        # QT timer to send and check handshake confirmation after a delay
        self.timer_sendHandshake = QTimer()
        self.timer_sendHandshake.timeout.connect(
            self.timer_callback_sendHandshake_once)
        self.timer_sendHandshake.setSingleShot(True)  # Run only once

        # QT timer to send and check handshake confirmation after a delay
        self.timer_endSession = QTimer()
        self.timer_endSession.timeout.connect(
            self.timer_callback_endSession_once)
        self.timer_endSession.setSingleShot(True)  # Run only once

        # ................ ROS Setup ................

        # @obsolete ROS Sublisher: @obsolete
        # wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)

        # ROS Subscriber: Esmacat arduino maze ard0 ease
        rospy.Subscriber(
            'Esmacat_read_maze_ard0_ease', ease_registers, self.ros_callback_Esmacat_read_maze_ard0_ease, tcp_nodelay=True)

        # Signal callback setup
        self.signal_Esmacat_read_maze_ard0_ease.connect(
            self.sig_callback_Esmacat_read_maze_ard0_ease)

        MazeDB.printMsg('ATTN', "FINISHED INTERFACE SETUP")

    # ------------------------ FUNCTIONS: Ecat Communicaton ------------------------

    def procEcatMessage(self):
        """ Used to parse new incoming ROS ethercat msg data. """

        # Print confirmation message
        MazeDB.printMsg('INFO', "(%d)ECAT PROCESSING ACK: %s",
                        self.EsmaCom.rcvEM.msgID, self.EsmaCom.rcvEM.msgTp.name)

        # ................ Process Ack Error First ................

        if self.EsmaCom.rcvEM.errTp != EsmacatCom.ErrorType.ERR_NONE:

            MazeDB.printMsg(
                'ERROR', "(%d)ECAT ERROR: %s", self.EsmaCom.rcvEM.msgID, self.EsmaCom.rcvEM.errTp.name)

            # I2C_FAILED
            if self.EsmaCom.rcvEM.errTp == EsmacatCom.ErrorType.I2C_FAILED:

                # Update number of init chambers setting based on chambers i2c status
                cham_cnt = sum(1 for i in range(
                    self.EsmaCom.rcvEM.argLen) if self.EsmaCom.rcvEM.ArgU.ui8[i] == 0)
                self.setParamTxtBox(param_ind=0, arg_val=cham_cnt)

                # Loop through chambers and set enable flag for chamber and wall
                for cham_i, chamber in enumerate(self.MP.Chambers):

                    # Skip chambers not acknowldged
                    if cham_i >= self.EsmaCom.rcvEM.argLen:
                        continue

                    # Store i2c status from arguments
                    i2c_status = self.EsmaCom.rcvEM.ArgU.ui8[cham_i]

                    # Check if status not equal to 0 and set corresponding chamber to error
                    if i2c_status != 0:

                        # Set corresponding chamber to error
                        chamber.setStatus(MazePlot.Status.ERROR)

                        # Log i2c error for this chamber
                        MazeDB.printMsg(
                            'ERROR', "\t chamber[%d] i2c_status[%d]", cham_i, i2c_status)

            # WALL_MOVE_FAILED
            if self.EsmaCom.rcvEM.errTp == EsmacatCom.ErrorType.WALL_MOVE_FAILED:

                # Loop through arguments
                for cham_i, chamber in enumerate(self.MP.Chambers):

                    # Skip chambers not acknowldged
                    if cham_i >= self.EsmaCom.rcvEM.argLen:
                        continue

                    # Store wall error byte from arguments
                    wall_err_byte = self.EsmaCom.rcvEM.ArgU.ui8[cham_i]

                    # Check if status not equal to 0
                    if wall_err_byte != 0:

                        # Get wall numbers from byte
                        wall_numbers = [i for i in range(
                            8) if wall_err_byte & (1 << i)]

                        # Loop through wall numbers and set to error
                        for wall_i in wall_numbers:
                            chamber.Walls[wall_i].setStatus(
                                MazePlot.Status.ERROR)

                        # Log walls with errors for this chamber
                        MazeDB.printMsg(
                            'ERROR', "\t chamber[%d] walls%s", cham_i, MazeDB.arrStr(wall_numbers))

        # ................ Process Ack Message ................

        # HANDSHAKE
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.HANDSHAKE:
            MazeDB.printMsg('ATTN', "SYSTEM INITIALIZED")

            # Set the handshake flag
            self.EsmaCom.isEcatConnected = True

            # Set maze hardware status to initialized
            self.MP.setStatus(MazePlot.Status.INITIALIZED)

            # Send INITIALIZE_CYPRESS message
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.INITIALIZE_CYPRESS)
            
            # Enable buttons
            self._widget.sysReinitBtn.setEnabled(True)
            self._widget.fileBrowseBtn.setEnabled(True)
            self._widget.filePreviousBtn.setEnabled(True)
            self._widget.fileNextBtn.setEnabled(True)
            self._widget.plotClearBtn.setEnabled(True)
            self._widget.plotSaveBtn.setEnabled(True)
            self._widget.plotSendBtn.setEnabled(True)

        # INITIALIZE_CYPRESS
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.INITIALIZE_CYPRESS:
            MazeDB.printMsg('ATTN', "CYPRESS I2C INITIALIZED")

            # Loop through chambers and set enable flag for chamber and wall
            for cham_i, chamber in enumerate(self.MP.Chambers):

                # Set chambers not acknowldged to excluded and skip
                if cham_i >= self.EsmaCom.rcvEM.argLen:
                    chamber.setStatus(MazePlot.Status.EXCLUDED)
                    continue

                # Skip if chamber is not in the uninitialized state (e.g., Excluded, Error or Warning)
                if chamber.status != MazePlot.Status.UNINITIALIZED:
                    continue

                # Set corresponding chamber to initialized
                chamber.setStatus(MazePlot.Status.INITIALIZED)

            # Disable text edit for total number of chambers
            self.sys_widgets[0].setEnabled(False)

            # Send INITIALIZE_WALLS message
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.INITIALIZE_WALLS)

        # INITIALIZE_WALLS
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.INITIALIZE_WALLS:
            MazeDB.printMsg('ATTN', "WALLS INITIALIZED")

            # Loop through chambers and set enable flag for walls
            for cham_i, chamber in enumerate(self.MP.Chambers):

                # Skip if chamber is not in the itialized state (e.g., Excluded, Error or Warning)
                if chamber.status != MazePlot.Status.INITIALIZED:
                    continue

                # Loop through walls
                for _, wall in enumerate(chamber.Walls):

                    # Skip if wall not set to uninitialized (e.g., Error)
                    if wall.status != MazePlot.Status.UNINITIALIZED:
                        continue

                    # Set walls to initialized
                    wall.setStatus(MazePlot.Status.INITIALIZED)

        # REINITIALIZE_SYSTEM
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.REINITIALIZE_SYSTEM:
            MazeDB.printMsg('ATTN', "SYSTEM REINITIALIZED")

            # Send INITIALIZE_WALLS message again
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.INITIALIZE_WALLS)

        # RESET_SYSTEM
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.RESET_SYSTEM:
            MazeDB.printMsg('ATTN', "ECAT COMMS DISCONNECTED")

            # Reset the handshake flag
            self.EsmaCom.isEcatConnected = False

        # Reset new message flag
        self.EsmaCom.rcvEM.isNew = False

    # ------------------------ CALLBACKS: ROS ------------------------

    def ros_callback_Esmacat_read_maze_ard0_ease(self, msg):
        """ ROS callback for Esmacat_read_maze_ard0_ease topic """

        # Wait for interface to initialize
        current_time = rospy.get_time()
        if (current_time - self.ts_interface_init) < self.dt_ecat_start:  # Less than 100ms
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

        # Parse the message and check if it is new
        if self.EsmaCom.readEcatMessage(reg_arr_si16):
            # Emit signal to process new message and update UI as UI should not be updated from a non-main thread
            self.signal_Esmacat_read_maze_ard0_ease.emit()

    def sig_callback_Esmacat_read_maze_ard0_ease(self):
        """ Signal callback for Esmacat_read_maze_ard0_ease topic """

        # Process new message arguments
        self.procEcatMessage()

    # ------------------------ CALLBACKS: Timers ------------------------

    def timer_callback_updateUI_loop(self):
        """ Timer callback to update UI. """

        # Update graphics
        self.scene.update()
        self._widget.plotMazeView.update()

    def timer_callback_sendHandshake_once(self):
        """ Timer callback to send handshake message once resend till confirmation. """

        # Check for HANDSHAKE message confirm recieved flag
        if self.EsmaCom.isEcatConnected == False:

            # Give up after 3 attempts based on message ID
            if self.EsmaCom.sndEM.msgID > 2:
                MazeDB.printMsg(
                    'ERROR', "SHUTDOWN: Handshake Failure Final [%d]", self.EsmaCom.sndEM.msgID)

                # Set maze hardware status to error
                self.MP.setStatus(MazePlot.Status.ERROR)

                # Set arduino list widget to error color
                self.MP.setStatus(MazePlot.Status.ERROR)

                return

            # Print warning if more than 1 message has been sent
            elif self.EsmaCom.sndEM.msgID > 0:
                MazeDB.printMsg(
                    'WARNING', "SHUTDOWN: Handshake Failure [%d]", self.EsmaCom.sndEM.msgID)

            # Send HANDSHAKE message with current system settings
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.HANDSHAKE, self.getParamTxtBox())

            # Restart check/send timer
            self.timer_sendHandshake.start(self.dt_ecat_check*1000)

    def timer_callback_endSession_once(self):
        """ Timer callback to incrementally shutdown session. """

        if self.cnt_shutdown_step == 0:
            # Send RESET_SYSTEM message if connected
            if self.EsmaCom.isEcatConnected:
                self.EsmaCom.writeEcatMessage(
                    EsmacatCom.MessageType.RESET_SYSTEM)

        elif self.EsmaCom.isEcatConnected == True:
            if self.cnt_shutdown_ack_check > int(5/self.dt_shutdown_step):
                MazeDB.printMsg(
                    'ERROR', "FAILED: SHUTDOWN: RESET_SYSTEM CONFIRMATION AFTER 5 SEC")
                self.EsmaCom.isEcatConnected = False
            else:
                self.cnt_shutdown_ack_check += 1
                # Wait for RESET_SYSTEM message confirmation and restart timer
                self.timer_endSession.start(self.dt_shutdown_step*1000)
                return

        elif self.cnt_shutdown_step == 1:
            # Kill self.signal_Esmacat_read_maze_ard0_ease thread
            self.signal_Esmacat_read_maze_ard0_ease.disconnect()
            MazeDB.printMsg(
                'INFO', "SHUTDOWN: Disconnected from Esmacat read timer thread")

        elif self.cnt_shutdown_step == 2:
            # Kill specific nodes
            self.terminate_ros_node("/Esmacat_application_node")
            self.terminate_ros_node("/interface_test_node")
            MazeDB.printMsg('INFO', "SHUTDOWN: Killed specific nodes")

        elif self.cnt_shutdown_step == 3:
            # Kill all nodes (This will also kill this script's node)
            os.system("rosnode kill -a")
            MazeDB.printMsg('INFO', "SHUTDOWN: Killed all nodes")

        elif self.cnt_shutdown_step == 4:
            # Process any pending events in the event loop
            QCoreApplication.processEvents()
            MazeDB.printMsg('INFO', "SHUTDOWN: Processed all events")

        elif self.cnt_shutdown_step == 5:
            # Close the UI window
            self._widget.close()
            MazeDB.printMsg('INFO', "SHUTDOWN: Closed UI window")

        elif self.cnt_shutdown_step == 6:
            # Send a shutdown request to the ROS master
            rospy.signal_shutdown("User requested shutdown")
            MazeDB.printMsg('INFO', "SHUTDOWN: Sent shutdown request to ROS master")

        elif self.cnt_shutdown_step == 7:
            # End the application
            QApplication.quit()
            MazeDB.printMsg('INFO', "SHUTDOWN: Ended application")
            return  # Return here to prevent the timer from being restarted after the application is closed

        # Increment the shutdown step after ecat disconnected
        self.cnt_shutdown_step += 1

        # Restart the timer for the next step
        self.timer_endSession.start(self.dt_shutdown_step*1000)

    # ------------------------ CALLBACKS: UI ------------------------

    def qt_callback_fileBrowseBtn_clicked(self):
        """ Callback for the "Browse" button. """

        # Get stored config file path
        stored_dir_path = self._widget.fileDirEdit.text()

        # Filter only CSV files
        filter = "CSV Files (*.csv)"
        csv_file_paths, _ = QFileDialog.getOpenFileNames(
            None, "Select files to add", stored_dir_path, filter)

        if csv_file_paths:

            # Update file list widget
            self.updateListCSV(csv_file_paths)

            # Load CSV data from active file
            self.loadFromCSV(0)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(self.csv_files) > 1:
                self._widget.fileNextBtn.setEnabled(True)
                self._widget.filePreviousBtn.setEnabled(True)
            else:
                self._widget.fileNextBtn.setEnabled(False)
                self._widget.filePreviousBtn.setEnabled(False)

    def qt_callback_plotSaveBtn_clicked(self):
        """ Callback function for the "Save" button."""

        # Get stored config file path
        stored_dir_path = self._widget.fileDirEdit.text()

        # Open the folder specified by in an explorer window
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        save_file_path, _ = QFileDialog.getSaveFileName(
            None, "Save CSV File", stored_dir_path, "CSV Files (*.csv);;All Files (*)", options=options)

        if save_file_path:
            # Ensure the file name ends with ".csv"
            if not save_file_path.endswith(".csv"):
                save_file_path += ".csv"

            # Call the function to save wall config data to the CSV file with the wall array values converted to bytes
            self.saveToCSV(save_file_path, WallConfig.make_num2byte_cw_list())

            # Use glob to get a list of CSV file paths
            save_dir_path = os.path.dirname(save_file_path)
            search_pattern = os.path.join(save_dir_path, "*.csv")
            csv_file_paths = glob.glob(search_pattern)

            # Update file list widget
            save_file_name = os.path.basename(save_file_path)
            self.updateListCSV(csv_file_paths, save_file_name)

            # Load CSV data from active filev
            self.loadFromCSV(0)

    def qt_callback_fileListWidget_item_clicked(self, item):
        """ Callback function for the file list widget."""

        # Get the index of the clicked item and set it as the current file index
        self.current_file_index = self._widget.fileListWidget.currentRow()

        # Update file index and load csv
        self.loadFromCSV(0)

    def qt_callback_fileNextBtn_clicked(self):
        """ Callback function for the "Next" button."""

        # Update file index and load csv
        self.loadFromCSV(1)

    def qt_callback_filePreviousBtn_clicked(self):
        """ Callback function for the "Previous" button."""

        # Update file index and load csv
        self.loadFromCSV(-1)

    def qt_callback_plotClearBtn_clicked(self):
        """ Callback function for the "Clear" button."""

        WallConfig.reset()  # reset all values in list
        self.MP.updatePlotFromWallConfig()  # update walls

    def qt_callback_plotSendBtn_clicked(self):
        """ Callback function for the "Send" button."""

        # Sort entries
        WallConfig._sort_entries()

        # Send MOVE_WALLS message with wall byte array
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.MOVE_WALLS, WallConfig.get_wall_byte_list())

    def qt_callback_sysStartBtn_clicked(self):
        """ Callback function for the "Start" button."""

        # Start handshake callback timer imidiatly
        self.timer_sendHandshake.start(0)

        # Disable Start button
        self._widget.sysStartBtn.setEnabled(False)

        # Enable Reinitialize button
        self._widget.sysReinitBtn.setEnabled(True)

    def qt_callback_sysReinitBtn_clicked(self):
        """ Callback function for the "Reinitialize" button."""

        # Send REINITIALIZE_SYSTEM message with current system settings
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.REINITIALIZE_SYSTEM, self.getParamTxtBox())

        # Reset wall status to uninitialized
        for _, chamber in enumerate(self.MP.Chambers):
            if MazePlot.isEnabled(chamber.status):
                for _, wall in enumerate(chamber.Walls):
                    if wall.status != MazePlot.Status.EXCLUDED:
                        wall.setStatus(MazePlot.Status.UNINITIALIZED, do_force=True)

    def qt_callback_sysQuiteBtn_clicked(self):
        """ Callback function for the "Quit" button."""

        # Call timer callback to incrementally shutdown session
        self.timer_endSession.start(0)

    from PyQt5.QtWidgets import QApplication

    # ------------------------ FUNCTIONS: CSV File Handling ------------------------

    def updateListCSV(self, csv_file_paths, selected_file_name=None):
        """ 
        Update the file list widget with the CSV files from csv_file_paths list.
        If selected_file_name is provided, set the currentRow to that file.
        """

        if csv_file_paths:

            # Extract the file names from the full paths and store them in self.files
            self.csv_files = [os.path.basename(
                file) for file in csv_file_paths]

            # Get the directory of the first selected file
            new_dir_path = os.path.dirname(csv_file_paths[0])

            # Set the directory in the file path text box
            self._widget.fileDirEdit.setText(new_dir_path)

            # Update the file list widget
            self._widget.fileListWidget.clear()
            self._widget.fileListWidget.addItems(self.csv_files)

            # If a selected file name is provided, set the currentRow to that file
            if selected_file_name:
                items = self._widget.fileListWidget.findItems(
                    selected_file_name, Qt.MatchExactly)
                if items:
                    self.current_file_index = self._widget.fileListWidget.row(
                        items[0])
                    self._widget.fileListWidget.setCurrentRow(
                        self.current_file_index)

    def loadFromCSV(self, list_increment):
        """ Function to load the wall config data from a CSV file """

        # Update the current file index
        self.current_file_index += list_increment

        # Loop back to the end or start if start or end reached, respectively
        if list_increment < 0 and self.current_file_index < 0:
            self.current_file_index = len(self.csv_files) - 1  # set to end
        elif list_increment > 0 and self.current_file_index >= len(self.csv_files):
            self.current_file_index = 0  # set to start

        # Set the current file in the list widget
        self._widget.fileListWidget.setCurrentRow(self.current_file_index)

        # Get the currently selected file path
        dir_path = self._widget.fileDirEdit.text()
        file_name = self.csv_files[self.current_file_index]
        file_path = os.path.join(dir_path, file_name)

        # Load and store CSV data
        try:
            with open(file_path, 'r') as csv_file:
                csv_reader = csv.reader(csv_file)
                wall_byte_config_list = [
                    [int(row[0]), int(row[1])] for row in csv_reader]
                WallConfig.make_byte2num_cw_list(wall_byte_config_list)
                MazeDB.printMsg(
                    'INFO', "CSV: Data Loaded from File: %s", file_name)
        except Exception as e:
            MazeDB.printMsg('ERROR', "CSV: Loading Data Error: %s", str(e))

        # Update plot walls
        self.MP.updatePlotFromWallConfig()

    def saveToCSV(self, save_file_path, wall_config_list):
        """ Function to save the wall config data to a CSV file """

        try:
            with open(save_file_path, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in wall_config_list:
                    csv_writer.writerow(row)
            save_file_name = os.path.basename(save_file_path)
            MazeDB.printMsg('INFO', "CSV: Data Saved to File: %s",
                            save_file_name)
        except Exception as e:
            MazeDB.printMsg('ERROR', "CSV: Saving Data Error: %s", str(e))

    # ------------------------ FUNCTIONS: System Operations ------------------------

    def getParamTxtBox(self):
        """ 
        Function to get the maze settings from the UI line edit boxes and return them as a list.

        Returns:
            unsigned 8-bit ctypes: List of maze settings
        """

        def scale_to_byte(val):
            """ Function to scale values greater than 255 and set to 8-bit for Ecat"""

            # Scale values greater than 255
            if val != 0 and val > 255:
                factor = 10
                for _ in range(4):
                    if val / factor < 255:
                        break
                    factor *= 10
                val = int(val / factor)

            # Return unsigned 8-bit ctypes
            return ctypes.c_uint8(val).value

        def check_get_field(field, default, min, max):
            """ Function to check the value of a field """

            # Check for empty fields
            if field.text() == "":
                field.setText(str(default))
            # Check if values are integers and in range
            try:
                val = int(field.text())
                val = scale_to_byte(val)
                if default < min or default > max:
                    raise ValueError
            except ValueError:
                field.setText(str(default))
                val = scale_to_byte(default)
            return val

        # Check the fieilds for each entry in sys_widgets
        read_settings = [0] * len(self.sysDefaults)
        for i, (field, defaults) in enumerate(zip(self.sys_widgets, self.sysDefaults)):
            read_settings[i] = check_get_field(
                field, defaults[0], defaults[1], defaults[2])

        # Print current status
        MazeDB.printMsg('INFO', "SETTINGS: CHAM INIT[%d] CHAM PER BLOCK[%d] ATTEMPTS MOVE[%d] PWM[%d] TIMEOUT[%d]",
                        *read_settings)

        # Return settings as a list
        return read_settings

    def setParamTxtBox(self, param_ind=None, arg_val=None):
        """ 
        Function to set the maze settings in the UI 
        """
        def set_field(field, defaults, arg):
            """ Function to check the value of a field """

            # Cast arg as intiger
            arg = int(arg)

            # Check if arg is in range
            if arg >= defaults[1] or arg <= defaults[2]:

                # Reenable the field if it was disabled
                is_enabled = field.isEnabled()
                field.setEnabled(True)

                # Save arg data and set the field value
                defaults[0] = arg
                field.setText(str(arg))
                field.setEnabled(is_enabled)
            return defaults

        # Update widgets and data for given argument
        if param_ind is not None and arg_val is not None:
            self.sysDefaults[param_ind] = set_field(
                self.sys_widgets[param_ind], self.sysDefaults[param_ind], arg_val)

        # Print current status
        MazeDB.printMsg('INFO', "SETTINGS: CHAM INIT[%d] CHAM PER BLOCK[%d] ATTEMPTS MOVE[%d] PWM[%d] TIMEOUT[%d]",
                        * [self.sysDefaults[i][0] for i in range(len(self.sysDefaults))])

    def move_ui_window(self, widget, horizontal_alignment, vertical_alignment):
        """
        Move the given widget to the specified position on the given monitor.

        Arguments:
            widget: The widget or window to move.
            horizontal_alignment: A string specifying the horizontal alignment. Valid values are 'left', 'center', and 'right'.
            vertical_alignment: A string specifying the vertical alignment. Valid values are 'top', 'middle', and 'bottom'.
        """

        # Get the geometry of the main monitor
        screen_geometry = QApplication.desktop().screenGeometry(0)

        if horizontal_alignment == "left":
            x = 0
        elif horizontal_alignment == "center":
            x = (screen_geometry.width() - widget.width()) / 2
        elif horizontal_alignment == "right":
            x = screen_geometry.width() - widget.width()
        else:
            raise ValueError(
                f"Invalid horizontal_alignment value: {horizontal_alignment}")

        if vertical_alignment == "top":
            y = 0
        elif vertical_alignment == "middle":
            y = (screen_geometry.height() - widget.height()) / 2
        elif vertical_alignment == "bottom":
            y = screen_geometry.height() - widget.height()
        else:
            raise ValueError(
                f"Invalid vertical_alignment value: {vertical_alignment}")

        widget.window().move(x, y)

    def terminate_ros_node(self, s):
        """ Function to terminate a ROS node when exiting the application """

        list_cmd = subprocess.Popen(
            "rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.decode().split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    """ @todo: Implement this function to handle the window close event """

    def closeEvent(self, event):
        """ Function to handle the window close event """

        MazeDB.printMsg('INFO', "Closing window...")
        # Call function to shut down the ROS session
        self.end_ros_session()
        event.accept()  # let the window close
        # TEMP
