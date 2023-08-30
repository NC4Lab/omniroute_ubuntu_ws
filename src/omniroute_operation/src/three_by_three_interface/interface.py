#!/usr/bin/env python

#======================== PACKAGES ========================
# Standard Library Imports
import os
import signal
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


#======================== GLOBAL VARS ========================

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
N_CHAMBERS = 9  # number of chambers in maze

#======================== GLOBAL CLASSES ========================

class EsmacatCom:
    """ 
    This class is used to communicate with the arduino via ethercat.
    It is used to send and receive messages from the arduino.
    """ 

    #------------------------ CLASS VARIABLES ------------------------

    # Handshake finished flag
    isEcatConnected = False

    #------------------------ NESTED CLASSES ------------------------

    class MessageType(Enum):
        """ Enum for ethercat python to arduino message type ID """
        MSG_NULL = 0
        HANDSHAKE = 1 # handshake must equal 1
        INITIALIZE_CYPRESS = 2
        INITIALIZE_WALLS = 3
        REINITIALIZE_ALL = 4
        MOVE_WALLS = 5

    class ErrorType(Enum):
        """ Enum for tracking message errors """
        ERR_NULL = 0
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
            self.ii16 = 0 # 16-bit index
        
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
            self.RegU = EsmacatCom.RegUnion()    # Union for storing ethercat 8 16-bit reg entries
            self.getUI = EsmacatCom.UnionIndStruct() # Union index handler for getting union data
            self.setUI = EsmacatCom.UnionIndStruct() # Union index handler for getting union data

            self.msgID = 0
            self.msgID_last = 0                     # Last message ID   
            self.msgTp = EsmacatCom.MessageType.MSG_NULL # Message type
            self.msgFoot = [0, 0]                  # Message footer

            self.argLen = 0                       # Message argument length
            self.ArgU = EsmacatCom.RegUnion()    # Union for storing message arguments
            self.argUI = EsmacatCom.UnionIndStruct() # Union index handler for argument union data

            self.isNew = False                         # New message flag
            self.isErr = False                         # Message error flag
            self.errTp = EsmacatCom.ErrorType.ERR_NULL # Message error type
    
    def __init__(self):

        # Initialize ethercat message handler instances
        self.sndEM = self.EcatMessageStruct()  # Initialize message handler instance for sending messages
        self.rcvEM = self.EcatMessageStruct()  # Initialize message handler instance for receiving messages

        # ROS Publisher: Initialize ethercat message handler instances
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)  # Esmacat write maze ard0 ease publisher
    
    #------------------------ PRIVATE METHODS ------------------------
    
    def _uSetCheckReg(self, r_EM, reg_arr_si16):
        """
        Set register values in union and check for -1 values indicating no or incomplete messages
        
        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
            reg_arr_si16 (list): Register array (signed int16).
        """

        # Bail if any register values are equal to -1
        if -1 in reg_arr_si16:
            return False

        # Set register values in union uint16 type
        for i in range(8):
            r_EM.RegU.si16[i] = reg_arr_si16[i]

        return True

    def _uSetMsgID(self, r_EM, msg_id=255):
        """
        Set message ID entry in union and update associated variable
        
        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
        """
        
        # Get new message ID: itterate id and roll over to 1 if max 16-bit value is reached
        msg_id = r_EM.msgID + 1 if r_EM.msgID < 65535 else 1

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

        r_EM.msgID_last = r_EM.msgID # store last message ID if
        r_EM.msgID = r_EM.RegU.ui16[r_EM.getUI.upd16(0)]

        # Check/log error skipped or out of sequence messages
        if r_EM.msgID - r_EM.msgID_last != 1 and \
            r_EM.msgID != r_EM.msgID_last: # don't log errors for repeat message reads
            self._trackErrType(r_EM, EsmacatCom.ErrorType.ECAT_ID_DISORDERED)
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
        self._uGetMsgType(r_EM)  # copy from union to associated struct variable

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
        r_EM.isErr = not is_found # Update struct error flag

        # Log error and set message type to none if not found
        if r_EM.isErr:
            msg_type_val = EsmacatCom.MessageType.MSG_NULL
            self._trackErrType(r_EM, EsmacatCom.ErrorType.ECAT_NO_MSG_TYPE_MATCH) 
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
        self._uGetErrType(r_EM)  # copy from union to associated struct variable

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
        r_EM.isErr = not is_found # Update struct error flag

        # Log error and set error type to none if not found
        if r_EM.isErr:
            err_type_val = EsmacatCom.ErrorType.ERR_NULL
            self._trackErrType(r_EM, EsmacatCom.ErrorType.ECAT_NO_ERR_TYPE_MATCH) 
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
        self._uGetArgLength(r_EM) # copy from union to associated struct variable

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
            self._uGetArgData8(r_EM) # copy from union to associated struct variable

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
            self._uGetArgData8(r_EM) # copy from union to associated struct variable

    def _uGetArgData8(self, r_EM):
        """
        Get reg union 8-bit message argument data and copy to arg union
        
        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
        """

        self._uGetArgLength(r_EM) # get argument length from union
        for i in range(r_EM.argLen):
            r_EM.ArgU.ui8[i] = r_EM.RegU.ui8[r_EM.getUI.upd8()] # copy to 8-bit argument Union

    def _uSetFooter(self, r_EM):
        """
        Set message footer entry in union and update associated variable
        
        Args:  
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct
        """

        r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254
        r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254
        self._uGetFooter(r_EM) # copy from union to associated struct variable

    def _uGetFooter(self, r_EM):
        """
        Get message footer from union
        
        Args:
            r_EM (EsmacatCom.EcatMessageStruct): EtherCAT message struct

        Returns:
            bool: True if footer is valid, False if not
        """

        r_EM.msgFoot[0] = r_EM.RegU.ui8[r_EM.getUI.upd8()] # copy first footer byte
        r_EM.msgFoot[1] = r_EM.RegU.ui8[r_EM.getUI.upd8()] # copy second footer byte

        # Log missing footer error
        if r_EM.msgFoot[0] != 254 or r_EM.msgFoot[1] != 254: # check if footer is valid
            self._trackErrType(r_EM, EsmacatCom.ErrorType.ECAT_MISSING_FOOTER) 
            return False
        return True

    def _uReset(self, r_EM):
        """Reset union data and indices"""

        # Reset union data to 0
        r_EM.RegU.si64[0] = 0
        r_EM.RegU.si64[1] = 0

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

    def _trackErrType(self, r_EM, err_tp, do_reset=False):
        """
        Check for and log any Ecat or runtime errors
        
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

                # Print error message
                MazeDB.logMsg(
                    'ERROR', "Ecat: %s: id new[%d] id last[%d] type[%d][%s]", r_EM.errTp.name, r_EM.msgID, r_EM.msgID_last, r_EM.msgTp.value, r_EM.msgTp.name)
                self._printEcatReg('ERROR', 0, r_EM.RegU)  # TEMP
        
        # Unset error type
        elif r_EM.errTp == err_tp:
                r_EM.errTp = EsmacatCom.ErrorType.ERR_NULL  
                r_EM.isErr = False

    def _printEcatReg(self, level, d_type, reg_u):
        """
        Print EtherCAT register data
        
        Args:
            level (str): ROS log level
            d_type (int): Data type [0:ui8, 1:ui16, 2:si16]
            reg_u (EsmacatCom.RegUnion): EtherCAT register union
        """

        # Print heading with type
        MazeDB.logMsg(level, "\t Ecat Register")

        # Print message data
        for i in range(8):
            if d_type == 2:
                MazeDB.logMsg(level, "\t si16[%d] [%d]", i, reg_u.si16[i]) 
            if d_type == 1:
                MazeDB.logMsg(level, "\t ui16[%d] [%d]", i, reg_u.ui16[i])
            if d_type == 0:
                MazeDB.logMsg(level, "\t ui8[%d][%d]  [%d][%d]", 2 * i, 2 * i + 1, reg_u.ui8[2 * i], reg_u.ui8[2 * i + 1])

    #------------------------ PUBLIC METHODS ------------------------

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

        # # Skip leftover register entries and ethercat setup junk (e.g., ui16[0] == 65535)
        # # Check if still waiting for handshake
        # # Directly check union id entry for first message
        # if self.isEcatConnected != 1 and \
        #     self.rcvEM.RegU.ui16[0] != 1:
        #     return False

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

        # @todo: figure out what to do if message corrupted

        # Set new message flag
        self.rcvEM.isNew = True

        # Print message
        MazeDB.logMsg('INFO', "(%d)ECAT ACK RECEIVED: %s", self.rcvEM.msgID, self.rcvEM.msgTp.name)
        self._printEcatReg('INFO', 0, self.rcvEM.RegU)  # TEMP

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
        MazeDB.logMsg('INFO', "(%d)ECAT SENT: %s", self.sndEM.msgID, self.sndEM.msgTp.name)
        self._printEcatReg('INFO', 0, self.sndEM.RegU)  # TEMP

class MazeDB(QGraphicsView):
    """ MazeDebug class to plot the maze """

    @classmethod
    def logMsg(cls, level, msg, *args):
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

    #------------------------ CLASS VARIABLES ------------------------

    # Stores the wall configuration list
    cw_wall_num_list = [] # cham_num x [wall_num]
    cw_wall_byte_list = [] # cham_num x wall_byte

    #------------------------ CLASS METHODS ------------------------

    def reset(self):
        """Resets the wall configuration list"""

        self.cw_wall_num_list = []
        self.cw_wall_byte_list = []

    def get_len(self):
        """Returns the number of entries in the wall configuration list"""

        return len(self.cw_wall_num_list)

    def add_wall(self, chamber_num, wall_num):
        """Adds a wall to the wall configuration list"""
        for item in self.cw_wall_num_list:
            if item[0] == chamber_num:
                if wall_num not in item[1]:
                    item[1].append(wall_num)
                    return
        self.cw_wall_num_list.append([chamber_num, [wall_num]])

    def remove_wall(self, chamber_num, wall_num):
        """Removes a wall from the wall configuration list"""
        for item in self.cw_wall_num_list[:]:  # Iterate over a copy of the list
            if item[0] == chamber_num:
                if wall_num in item[1]:
                    item[1].remove(wall_num)
                    if not item[1]:  # If the second column is empty, remove the entire row
                        self.cw_wall_num_list.remove(item)
                    return

    def make_byte2num_cw_list(self, _cw_wall_byte_list):
        """
        Used to convert imported CSV with wall byte mask values to a list with wall numbers

        Args:
            _cw_wall_byte_list (list): 2D list: col_1 = chamber number, col_2 = wall byte mask
        
        Returns:
            2D list: col_1 = chamber number, col_2 = nested wall numbers
        """

        # Clear/reset the existing wall_config_list
        self.reset()

        # Convert the byte values to arrays and update the wall_config_list
        for row in _cw_wall_byte_list:
            chamber_num = row[0]
            byte_value = row[1]

            # Convert the byte_value back to an array of wall numbers
            wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

            self.cw_wall_num_list.append([chamber_num, wall_numbers])

            return self.cw_wall_num_list

    def make_num2byte_cw_list(self):
        """
        Used to covert wall number arrays to byte values for saving to CSV
        
        Returns:
            2D list: col_1 = chamber number, col_2 = wall byte mask
        """  

        self.cw_wall_byte_list = []

        for row in self.cw_wall_num_list:  # row = [chamber_num, wall_numbers]
            chamber_num = row[0]
            wall_arr = row[1]
            # Initialize the byte value
            byte_value = 0
            # Iterate over the array of values
            for wall_i in wall_arr:
                if 0 <= wall_i <= 7:
                    # Set the corresponding bit to 1 using bitwise OR
                    byte_value |= (1 << wall_i)
            self.cw_wall_byte_list.append([chamber_num, byte_value])

        return self.cw_wall_byte_list

    def get_wall_byte_list(self):
        """
        Used to generate a 1D list with only byte values for each chamber corespoinding to the wall configuration
        For use with the EsmacatCom class
        
        Returns: 
            1D list with byte values for all chambers
        """

        self.cw_wall_byte_list = self.make_num2byte_cw_list()

        # Update U_arr with corresponding chamber and wall byte
        _wall_byte_list = [0] * N_CHAMBERS
        #wall_arr = [0] * len(self.wallConfigList)
        for cw in self.cw_wall_byte_list:
            _wall_byte_list[cw[0]] = cw[1]

        return _wall_byte_list

    def _sort_entries(self):
        """Sorts the entries in the wall configuration list by chamber number and wall numbers"""

        # Sort the rows by the entries in the based on the first chamber number
        self.cw_wall_num_list.sort(key=lambda row: row[0])

        # Sort the arrays in the second column
        for row in self.cw_wall_num_list:
            row[1].sort()

    def __iter__(self):
        """Returns an iterator for the wall configuration list"""
        return iter(self.cw_wall_num_list)

    def __str__(self):
        """Returns the wall configuration list as a string"""
        return str(self.cw_wall_num_list)
    
class StateQColors:
    """ Class for defining state colors """

    up = QColor(0, 255, 0)
    down = QColor(0, 0, 0)
    enabled = QColor(0, 0, 0)
    disabled = QColor(200, 200, 200)
    warning = QColor(255, 165, 0)
    error = QColor(255, 0, 0)

class MazePlot(QGraphicsView):
    """ MazePlot class to plot the maze """

    #------------------------ NESTED CLASSES ------------------------

    # CLASS: Wall
    class Wall(QGraphicsItemGroup):
        def __init__(self, _chamber_num, _wall_num, _p0, _p1, _wall_width, _label_pos=None, parent=None):
            super().__init__(parent)

            # Flags for wall state
            self.is_up = False
            self.is_enabled = False

            # Store arguments parameters
            self.chamber_num = _chamber_num
            self.wall_num = _wall_num
            
            # Plot wall line
            self.line = QGraphicsLineItem(QLineF(_p0[0], _p0[1], _p1[0], _p1[1]))
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

            # Set wall state
            self.setState('DISABLED')

        def setState(self, state_str):
            """Sets the wall state and color"""

            # Get state color
            state_col = MazePlot._getStateColor(state_str)

            # Set text color
            self.label.setDefaultTextColor(state_col)

            # Set wall color
            current_pen = self.line.pen()
            current_pen.setColor(state_col)
            self.line.setPen(current_pen)

            # Set enabled flag 
            self.is_enabled = state_str != 'DISABLED' and state_str != 'WARNING' and state_str != 'ERROR'

            # Set wall up flag to true if if the state string is 'UP'
            if state_str == 'UP':
                self.is_up = True
            else:
                self.is_up = False

        def mousePressEvent(self, event):
            """Handles mouse press events and sets the wall state"""

            # Bail if wall is disabled
            if not self.is_enabled:
                return

            if event.button() == Qt.LeftButton:
                
                # Toggle wall state
                if self.is_up:
                    self.setState('DOWN')
                    
                # TEMP wall_clicked_pub.publish(self.chamber_num, self.wall_num, self.pos_state)
                
                # Update configuration list
                if self.is_up:  # add list entry
                    WallConfig.add_wall(self.chamber_num, self.wall_num)
                else:  # remove list entry
                    WallConfig.remove_wall(self.chamber_num, self.wall_num)

    # CLASS: Chamber
    class Chamber(QGraphicsItemGroup):
        def __init__(self, _chamber_num, _center_x, _center_y, _chamber_width, _wall_width, parent=None):
            super().__init__(parent)

            # Flags for chamber state
            self.is_up = False
            self.is_enabled = False
            self.is_err_i2c = False
            
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
                                        _wall_num=k,
                                        _p0=wall_vertices_0[k],
                                        _p1=wall_vertices_1[k+1],
                                        _wall_width=_wall_width,
                                        _label_pos=wall_label_pos[k])
                          for k in range(8)]

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

        def getOctagonVertices(self, x, y, w, offset):
            vertices_list = [(round(x + w*math.cos(k)), round(y+w*math.sin(k)))
                             for k in np.linspace(math.pi, 3*math.pi, 9) + offset]
            return vertices_list

        def setState(self, state_str):
            """Sets the chamber state flags and color"""

            # Set enabled flag 
            self.is_enabled = state_str != 'DISABLED' and state_str != 'WARNING' and state_str != 'ERROR'

            # Get state color
            state_col = MazePlot._getStateColor(state_str)

            # Set text color
            self.label.setDefaultTextColor(state_col)
            
            # Set chamber color
            #self.octagon.setBrush(QBrush(state_col))
            
        def mousePressEvent(self, event):
            """Handles mouse press events and sets the chamber state"""

            # Bail if chamber is not enabled
            if not self.is_enabled:
                return
            
            if event.button() == Qt.LeftButton:
                pass

    #------------------------ METHODS ------------------------

    def __init__(self, _num_rows_cols, _chamber_width, _wall_width):

        self.num_rows_cols = _num_rows_cols
        self.chamber_width = _chamber_width

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


    def updatePlotFromWallConfig(self):
        """Updates the plot based on the wall configuration list"""

        for chamber in self.Chambers:
            for wall in chamber.Walls:
                # Check if there is an entry for the current chamber and wall in WallConfig
                chamber_num = chamber.chamber_num
                wall_num = wall.wall_num
                entry_found = any(
                    chamber_num == entry[0] and wall_num in entry[1] for entry in WallConfig.cw_wall_num_list)

                # Set the wall state based on whether the entry is found or not
                if entry_found:
                    wall.setState('UP')
                else:
                    wall.setState('DOWN') 

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

    def _getStateColor(state_str):
            """Get plot color for a given state"""

            # Get plot color associated with each state
            if state_str == 'UP':
                state_col = StateQColors.up
            elif state_str == 'DOWN':
                state_col = StateQColors.down
            elif state_str == 'ENABLED':
                state_col = StateQColors.enabled
            elif state_str == 'DISABLED':
                state_col = StateQColors.disabled
            elif state_str == 'WARNING':
                state_col = StateQColors.warning
            elif state_str == 'ERROR':
                state_col = StateQColors.error
            else:
                state_col = StateQColors.down

            return state_col

#======================== MAIN UI CLASS ========================
class Interface(Plugin):
    """ Interface plugin """

    #------------------------ CLASS VARIABLES ------------------------

    # Define signals
    signal_Esmacat_read_maze_ard0_ease = Signal()

    #------------------------ NESTED CLASSES ------------------------

    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

        #................ QT UI Setup ................ 

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

        # Initialize file list text and index
        self.current_file_index = 0  # set to zero
        self._widget.fileDirEdit.setText(
            self.getPathConfigDir())  # set to default path

        # Initialize ardListWidget with arduino names labeled Arduino 0-4
        for i in range(5):
            item = QListWidgetItem("Arduino " + str(i))
            # Set to disabled color
            item.setForeground(QColor(StateQColors.disabled))
            # Add item to list
            self._widget.ardListWidget.addItem(item)
        # Hide the blue selection bar
        #self._widget.ardListWidget.setStyleSheet("QListWidget::item { border-bottom: 1px solid black; }")

        #................ Maze Setup ................

        # Calculate chamber width and wall line width and offset
        maze_view_size = self._widget.plotMazeView.width()
        chamber_width = self._widget.plotMazeView.width()*0.9/NUM_ROWS_COLS
        wall_width = chamber_width*0.1

        # Create MazePlot and populate walls according to WALL_MAP
        self.MP = MazePlot(_num_rows_cols=NUM_ROWS_COLS,
                                  _chamber_width=chamber_width,
                                  _wall_width=wall_width)

        # Add chambers and disable walls not connected
        for k, c in enumerate(self.MP.Chambers):
            self.scene.addItem(c)
            for j, w in enumerate(c.Walls):
                if j not in WALL_MAP[k]:
                    w.setEnabled(False)
                    w.setVisible(False)

        # Add walls - this is a new loop so that they are drawn above the chambers
        for c in self.MP.Chambers:
            for w in c.Walls:
                self.scene.addItem(w)

        #................ Ecat Setup ................

        # Create EsmacatCom object
        self.EsmaCom_A0 = EsmacatCom()

        # Store the time the interface was initialized
        self.ts_interface_init = rospy.get_time() # (sec)

        # Specify delay to start and check reading/writing Esmacat data 
        self.dt_ecat_start = 1 # (sec)
        self.dt_ecat_check = 0.5 # (sec)

        # Couter to incrementally shut down opperations
        self.cnt_shutdown_step = 0
        self.dt_shutdown_step = 0.25 # (sec)

        #................ QT Callback Setup ................

        # QT UI object callback setup
        self._widget.fileListWidget.itemClicked.connect(
            self.qt_callback_fileListWidget_clicked)
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
        self._widget.sysQuiteBtn.clicked.connect(
            self.qt_callback_sysQuiteBtn_clicked)

        # QT timer setup for UI updating
        self.timer_updateUI = QTimer()
        self.timer_updateUI.timeout.connect(self.timer_callback_updateUI_loop)
        self.timer_updateUI.start(20)  # set incriment (ms)

        # QT timer to send and check handshake confirmation after a delay
        self.timer_sendHandshake = QTimer()
        self.timer_sendHandshake.timeout.connect(
            self.timer_callback_sendHandshake_once)
        self.timer_sendHandshake.setSingleShot(True)  # Run only once
        self.timer_sendHandshake.start(self.dt_ecat_start*1000) # (ms)

        # QT timer to send and check handshake confirmation after a delay
        self.timer_endSession = QTimer()
        self.timer_endSession.timeout.connect(
            self.timer_callback_endSession_once)
        self.timer_endSession.setSingleShot(True)  # Run only once

        #................ ROS Setup ................

        # @obsolete ROS Sublisher: @obsolete 
        # wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)

        # ROS Subscriber: Esmacat arduino maze ard0 ease
        rospy.Subscriber(
            'Esmacat_read_maze_ard0_ease', ease_registers, self.ros_callback_Esmacat_read_maze_ard0_ease, tcp_nodelay=True)

        # Signal callback setup
        self.signal_Esmacat_read_maze_ard0_ease.connect(
            self.sig_callback_Esmacat_read_maze_ard0_ease)
        
        MazeDB.logMsg('ATTN', "FINISHED INTERFACE SETUP")


    #------------------------ FUNCTIONS: Ecat Communicaton ------------------------

    def procEcatMessage(self):
        """
        Used to parse new incoming ROS ethercat msg data.
        
        Returns:
            int: Success/error codes [0:no message, 1:new message, 2:error]
        """

        # Print confirmation message
        MazeDB.logMsg('INFO', "(%d)ECAT PROCESSING ACK: %s", 
                    self.EsmaCom_A0.rcvEM.msgID, self.EsmaCom_A0.rcvEM.msgTp.name)
                        
        
        #................ Process Ack Message ................ 

        # HANDSHAKE
        if self.EsmaCom_A0.rcvEM.msgTp == EsmacatCom.MessageType.HANDSHAKE:

            # Set the handshake flag
            self.EsmaCom_A0.isEcatConnected = True
            MazeDB.logMsg('ATTN', "ECAT COMMS CONNECTED")

            # Set arduino list widget to enabled color
            self._widget.ardListWidget.item(0).setForeground(StateQColors.enabled)

            # Send INITIALIZE_CYPRESS message
            self.EsmaCom_A0.writeEcatMessage(EsmacatCom.MessageType.INITIALIZE_CYPRESS)

        # INITIALIZE_CYPRESS
        if self.EsmaCom_A0.rcvEM.msgTp == EsmacatCom.MessageType.INITIALIZE_CYPRESS:
            # Set the handshake flag
            self.EsmaCom_A0.isEcatConnected = True
            MazeDB.logMsg('ATTN', "I2C INITIALIZED")

            # Loop through chambers and 

            # Send INITIALIZE_WALLS message
            self.EsmaCom_A0.writeEcatMessage(EsmacatCom.MessageType.INITIALIZE_WALLS)

        # INITIALIZE_WALLS
        if self.EsmaCom_A0.rcvEM.msgTp == EsmacatCom.MessageType.INITIALIZE_WALLS:
             # Set the handshake flag
            self.EsmaCom_A0.isEcatConnected = True
            MazeDB.logMsg('ATTN', "WALLS INITIALIZED")

        # REINITIALIZE_ALL
        if self.EsmaCom_A0.rcvEM.msgTp == EsmacatCom.MessageType.REINITIALIZE_ALL:
            # Set the handshake flag
            self.EsmaCom_A0.isEcatConnected = False
            MazeDB.logMsg('ATTN', "ECAT COMMS DISCONNECTED")

        # Reset new message flag
        self.EsmaCom_A0.rcvEM.isNew = False

        #................ Process Ack Error ................ 

        if self.EsmaCom_A0.rcvEM.errTp != EsmacatCom.ErrorType.ERR_NULL:

            MazeDB.logMsg(
                'ERROR', "%s", self.EsmaCom_A0.rcvEM.errTp.name)
            
            # I2C_FAILED
            if self.EsmaCom_A0.rcvEM.errTp == EsmacatCom.ErrorType.I2C_FAILED:

                # Loop through arguments
                for i in range(self.EsmaCom_A0.rcvEM.argLen):
                    i2c_status = self.EsmaCom_A0.rcvEM.ArgU.ui8[i]

                    # Check if status not equal to 0
                    if i2c_status != 0: 
                        # Set corresponding chamber to error
                        self.MP.Chambers[i].setState('ERROR')
                        MazeDB.logMsg('ERROR', "\t chamber[%d] i2c status[%d]", i, i2c_status)
                    else:
                        # Set corresponding chamber to enabled
                        self.MP.Chambers[i].setState('ENABLED')

            # WALL_MOVE_FAILED
            if self.EsmaCom_A0.rcvEM.errTp == EsmacatCom.ErrorType.WALL_MOVE_FAILED:

                # Loop through arguments
                for i in range(self.EsmaCom_A0.rcvEM.argLen):
                    err_byte = self.EsmaCom_A0.rcvEM.ArgU.ui8[i]

                    # Check if status not equal to 0
                    if err_byte != 0: 

                        # Loop through wall numbers
                        wall_numbers = [i for i in range(8) if err_byte & (1 << i)]
                        MazeDB.logMsg('ERROR', "\t chamber[%d] walls[%s]", i, MazeDB.arrStr(wall_numbers))
                        for wall_num in wall_numbers:
                            # Set corresponding wall to error
                            self.MP.Chambers[i].Walls[wall_num].setState('ERROR')
    
    #------------------------ CALLBACKS: ROS ------------------------

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
        if self.EsmaCom_A0.readEcatMessage(reg_arr_si16):
            # Emit signal to process new message and update UI as UI should not be updated from a non-main thread
            self.signal_Esmacat_read_maze_ard0_ease.emit()           

    def sig_callback_Esmacat_read_maze_ard0_ease(self):
        """ Signal callback for Esmacat_read_maze_ard0_ease topic """

        # Process new message arguments
        self.procEcatMessage()

    #------------------------ CALLBACKS: Timers ------------------------

    def timer_callback_updateUI_loop(self):
            """ Timer callback to update UI. """

            # Update graphics
            self.scene.update()
            self._widget.plotMazeView.update()

    def timer_callback_sendHandshake_once(self):
        """ Timer callback to send handshake message once resend till confirmation. """
        
        # Check for HANDSHAKE message confirm recieved flag
        if self.EsmaCom_A0.isEcatConnected == False:

            # Give up after 3 attempts based on message ID
            if self.EsmaCom_A0.sndEM.msgID > 2:
                MazeDB.logMsg(
                    'ERROR', "Handshake Failure [%d]", self.EsmaCom_A0.sndEM.msgID)
                # Set arduino list widget to error color
                self._widget.ardListWidget.item(0).setForeground(StateQColors.error)
                return
            
            # Print warning if more than 1 message has been sent
            elif self.EsmaCom_A0.sndEM.msgID > 0:
                MazeDB.logMsg(
                    'WARNING', "Handshake Failure Final [%d]", self.EsmaCom_A0.sndEM.msgID)
                return

            # Send HANDSHAKE message to arduino with number of chambers to initialize
            #self.EsmaCom_A0.writeEcatMessage(EsmacatCom.MessageType.HANDSHAKE, N_CHAMBERS)
            self.EsmaCom_A0.writeEcatMessage(EsmacatCom.MessageType.HANDSHAKE, 3)

            # Restart check/send timer
            self.timer_sendHandshake.start(self.dt_ecat_check*1000)

    def timer_callback_endSession_once(self):
        """ Timer callback to incrementally shutdown session. """
        
        if self.cnt_shutdown_step == 0:
            # Send REINITIALIZE_ALL message to arduino
            self.EsmaCom_A0.writeEcatMessage(EsmacatCom.MessageType.REINITIALIZE_ALL)

        elif self.EsmaCom_A0.isEcatConnected == True:
            # Wait for REINITIALIZE_ALL message confirmation and restart timer
            self.timer_endSession.start(self.dt_shutdown_step*1000)
            return
            
        elif self.cnt_shutdown_step == 1:
            # Kill self.signal_Esmacat_read_maze_ard0_ease thread
            self.signal_Esmacat_read_maze_ard0_ease.disconnect()
            MazeDB.logMsg('INFO', "Disconnected from Esmacat read timer thread")

        elif self.cnt_shutdown_step == 2:
            # Kill specific nodes
            self.terminate_ros_node("/Esmacat_application_node")
            self.terminate_ros_node("/interface_test_node")
            MazeDB.logMsg('INFO', "Killed specific nodes")

        elif self.cnt_shutdown_step == 3:
            # Kill all nodes (This will also kill this script's node)
            os.system("rosnode kill -a")
            MazeDB.logMsg('INFO', "Killed all nodes")

        elif self.cnt_shutdown_step == 4:
            # Process any pending events in the event loop
            QCoreApplication.processEvents()
            MazeDB.logMsg('INFO', "Processed all events")

        elif self.cnt_shutdown_step == 5:
            # Close the UI window
            self._widget.close()
            MazeDB.logMsg('INFO', "Closed UI window")

        elif self.cnt_shutdown_step == 6:
            # Send a shutdown request to the ROS master
            rospy.signal_shutdown("User requested shutdown")
            MazeDB.logMsg('INFO', "Sent shutdown request to ROS master")

        elif self.cnt_shutdown_step == 7:
            # End the application
            QApplication.quit()
            MazeDB.logMsg('INFO', "Ended application")
            return  # Return here to prevent the timer from being restarted after the application is closed
        
        # Increment the shutdown step after ecat disconnected
        self.cnt_shutdown_step += 1

        # Restart the timer for the next step
        self.timer_endSession.start(self.dt_shutdown_step*1000)

    #------------------------ CALLBACKS: UI ------------------------

    def qt_callback_fileBrowseBtn_clicked(self):
        """ Callback for the "Browse" button. """

        # Filter only CSV files
        filter = "CSV Files (*.csv)"
        files, _ = QFileDialog.getOpenFileNames(
            None, "Select files to add", self.getPathConfigDir(), filter)

        if files:
            # Clear the list widget to remove any previous selections
            self._widget.fileListWidget.clear()

            # Extract the file names from the full paths and store them in self.files
            self.files = [os.path.basename(file) for file in files]

            # Add the selected file names to the list widget
            self._widget.fileListWidget.addItems(self.files)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(self.files) > 1:
                self._widget.fileNextBtn.setEnabled(True)
                self._widget.filePreviousBtn.setEnabled(True)
            else:
                self._widget.fileNextBtn.setEnabled(False)
                self._widget.filePreviousBtn.setEnabled(False)

            # Update file index and load csv
            self.loadFromCSV(0)

    def qt_callback_fileListWidget_clicked(self, item):
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

    def qt_callback_plotSaveBtn_clicked(self):
        """ Callback function for the "Save" button."""

        # Open the folder specified by self.getPathConfigDir() in an explorer window
        folder_path = self.getPathConfigDir()
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getSaveFileName(
            None, "Save CSV File", folder_path, "CSV Files (*.csv);;All Files (*)", options=options)

        if file_name:
            # Ensure the file name ends with ".csv"
            if not file_name.endswith(".csv"):
                file_name += ".csv"

            # The user has specified a file name, you can perform additional actions here
            MazeDB.logMsg('INFO', "Selected file:", file_name)

            # Call the function to save wall config data to the CSV file with the wall array values converted to bytes
            self.saveToCSV(file_name, WallConfig.make_num2byte_cw_list())

    def qt_callback_plotSendBtn_clicked(self):
        """ Callback function for the "Send" button."""

        # Sort entries
        WallConfig._sort_entries()

        # Send the wall byte array to the arduino
        self.EsmaCom_A0.writeEcatMessage(EsmacatCom.MessageType.MOVE_WALLS, WallConfig.get_wall_byte_list())

    def qt_callback_sysQuiteBtn_clicked(self):
        """ Callback function for the "Quit" button."""

        # Call timer callback to incrementally shutdown session
        self.timer_endSession.start(0)

    #------------------------ FUNCTIONS: CSV File Handling ------------------------

    def getPathConfigDir(self, file_name=None):
        """ Function to get the path to the "config" directory four levels up from the current script file. """

        # Get the absolute path of the current script file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Create the path to the "config" directory four levels up
        dir_path = os.path.abspath(os.path.join(
            script_dir, '..', '..', '..', '..', 'config', 'paths'))
        # Return file or dir path
        if file_name is not None:
            return os.path.join(dir_path, file_name)
        else:
            return dir_path

    def saveToCSV(self, file_name, wall_config_list):
        """ Function to save the wall config data to a CSV file """

        try:
            with open(file_name, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in wall_config_list:
                    csv_writer.writerow(row)
            MazeDB.logMsg('INFO', "Data saved to:", file_name)
        except Exception as e:
            MazeDB.logMsg('ERROR', "Error saving data to CSV:", str(e))

    def loadFromCSV(self, list_increment):
        """ Function to load the wall config data from a CSV file """

        # Update the current file index
        self.current_file_index += list_increment

        # Loop back to the end or start if start or end reached, respectively
        if list_increment < 0 and self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1  # set to end
        elif list_increment > 0 and self.current_file_index >= len(self.files):
            self.current_file_index = 0  # set to start

        # Set the current file in the list widget
        self._widget.fileListWidget.setCurrentRow(self.current_file_index)

        # Get the currently selected file path
        file_name = self.files[self.current_file_index]
        folder_path = self.getPathConfigDir()
        file_path = os.path.join(folder_path, file_name)

        # Load and store CSV data
        try:
            with open(file_path, 'r') as csv_file:
                csv_reader = csv.reader(csv_file)
                wall_byte_config_list = [
                    [int(row[0]), int(row[1])] for row in csv_reader]
                WallConfig.make_byte2num_cw_list(wall_byte_config_list)
                MazeDB.logMsg('INFO', "Data loaded successfully.")
        except Exception as e:
            MazeDB.logMsg('ERROR', "Error loading data from CSV:", str(e))

        # Update plot walls
        self.MP.updatePlotFromWallConfig()

    #------------------------ FUNCTIONS: System Operations ------------------------

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
        
        MazeDB.logMsg('INFO', "Closing window...")
        # Call function to shut down the ROS session
        self.end_ros_session()
        event.accept()  # let the window close

