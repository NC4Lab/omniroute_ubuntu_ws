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

#======================== GLOBAL FUNCTIONS ========================

def rospyLogCol(level, msg, *args):
    """ Log to ROS in color """

    def frmt_msg(color, msg, *args):
        """ Format message with color """
        colored_message = f"{color}{msg}{Style.RESET_ALL}"
        return colored_message % args

    # Exit if DB_VERBOSE is false
    if not DB_VERBOSE:
        return
    
    # Log message by type and color to ROS
    if level == 'ERROR':
        rospy.logerr(frmt_msg(Fore.RED, msg, *args))
    elif level == 'WARNING':
        rospy.logwarn(frmt_msg(Fore.YELLOW, msg, *args))
    elif level == 'INFO':
        rospy.loginfo(frmt_msg(Fore.BLUE, msg, *args))
    elif level == 'DEBUG':
        rospy.loginfo(frmt_msg(Fore.GREEN, msg, *args))
    else:
        rospy.loginfo(frmt_msg(Fore.BLACK, msg, *args))


#======================== GLOBAL CLASSES ========================

class Maze_Plot(QGraphicsView):
    """ Maze_Plot class to plot the maze """

    #------------------------ NESTED CLASSES ------------------------

    # CLASS: Wall
    class Wall(QGraphicsItemGroup):
        def __init__(self, p0=(0, 0), p1=(1, 1), wall_width=-1, chamber_num=-1, wall_num=-1, state=False, label_pos=None, parent=None):
            super().__init__(parent)

            self.chamber_num = chamber_num
            self.wall_num = wall_num
            self.state = state

            self.line = QGraphicsLineItem(QLineF(p0[0], p0[1], p1[0], p1[1]))
            self.upPen = QPen(Qt.red)
            self.upPen.setWidth(wall_width)
            self.downPen = QPen(Qt.gray)
            self.downPen.setWidth(wall_width)

            self.setState(state)

            self.addToGroup(self.line)

            # Plot wall numbers
            self.label = QGraphicsTextItem(str(wall_num))
            font_size = wall_width
            self.label.setFont(QFont("Arial", font_size, QFont.Bold))

            if label_pos == None:
                label_pos = ((p0[0]+p1[0])/2, (p0[1]+p1[1])/2)
            self.label.setPos(label_pos[0], label_pos[1])
            self.addToGroup(self.label)

            Maze_Plot.centerText(self.label, label_pos[0], label_pos[1])

        def mousePressEvent(self, event):
            if event.button() == Qt.LeftButton:
                # rospy_log_col('INFO', "\t chamber %d wall %d clicked in UI" % (self.chamber_num, self.wall_num))
                self.setState(not self.state)
                # wall_clicked_pub.publish(self.chamber_num, self.wall_num, self.state)
                if self.state:  # add list entry
                    Wall_Config.addWall(self.chamber_num, self.wall_num)
                else:  # remove list entry
                    Wall_Config.removeWall(self.chamber_num, self.wall_num)

        def setState(self, state: bool):
            if state:
                self.line.setPen(self.upPen)
            else:
                self.line.setPen(self.downPen)

            self.state = state

    # CLASS: Chamber
    class Chamber(QGraphicsItemGroup):
        def __init__(self, center_x, center_y, chamber_width, chamber_num, wall_width, parent=None):
            super().__init__(parent)
            self.center_x = center_x
            self.center_y = center_y
            self.chamber_width = chamber_width
            self.chamber_num = chamber_num

            # Plot backround chamber octogons
            octagon_vertices = self.getOctagonVertices(
                center_x, center_y, chamber_width/2, math.radians(22.5))
            octagon_points = [QPointF(i[0], i[1]) for i in octagon_vertices]
            self.octagon = QGraphicsPolygonItem(QPolygonF(octagon_points))
            self.octagon.setBrush(QBrush(QColor(200, 200, 200)))
            self.addToGroup(self.octagon)

            # Plot cahamber numbers
            self.label = QGraphicsTextItem(str(chamber_num))
            font_size = wall_width*2
            self.label.setFont(QFont("Arial", font_size, QFont.Bold))
            self.addToGroup(self.label)

            # Center the text over the chamber's center
            Maze_Plot.centerText(self.label, center_x, center_y)

            wall_angular_offset = 2*math.pi/32  # This decides the angular width of the wall
            wall_vertices_0 = self.getOctagonVertices(
                center_x, center_y, chamber_width/2, -math.pi/8+wall_angular_offset)
            wall_vertices_1 = self.getOctagonVertices(
                center_x, center_y, chamber_width/2, -math.pi/8-wall_angular_offset)
            wall_label_pos = self.getOctagonVertices(
                center_x, center_y, chamber_width/3, 0)

            self.walls = [Maze_Plot.Wall(p0=wall_vertices_0[k],
                                         p1=wall_vertices_1[k+1],
                                         chamber_num=chamber_num,
                                         wall_num=k,
                                         wall_width=wall_width,
                                         label_pos=wall_label_pos[k])
                          for k in range(8)]

        def getOctagonVertices(self, x, y, w, offset):
            vertices_list = [(round(x + w*math.cos(k)), round(y+w*math.sin(k)))
                             for k in np.linspace(math.pi, 3*math.pi, 9) + offset]
            return vertices_list

        def mousePressEvent(self, event):
            if event.button() == Qt.LeftButton:
                pass
                # rospy_log_col('INFO', "\t chamber %d clicked in UI" % self.chamber_num)

    # CLASS: Maze
    class Maze:

        def __init__(self, num_rows, num_cols, chamber_width, wall_width):

            self.num_rows = num_rows
            self.num_cols = num_cols
            self.chamber_width = chamber_width

            maze_width = self.chamber_width * self.num_cols
            maze_height = self.chamber_width * self.num_rows
            half_width = chamber_width/2
            x_pos = np.linspace(half_width, int(
                maze_width - half_width),  num_cols)
            y_pos = np.linspace(half_width, int(
                maze_height - half_width),  num_rows)

            self.chambers = []
            k = 0
            for y in y_pos:
                for x in x_pos:
                    self.chambers.append(
                        Maze_Plot.Chamber(center_x=x, center_y=y, chamber_width=chamber_width, wall_width=wall_width, chamber_num=k))
                    k = k+1

        def updateWalls(self):
            cw_list = Wall_Config.wallConfigList

            for chamber in self.chambers:
                for wall in chamber.walls:
                    # Check if there is an entry for the current chamber and wall in Wall_Config
                    chamber_num = chamber.chamber_num
                    wall_num = wall.wall_num
                    entry_found = any(
                        chamber_num == entry[0] and wall_num in entry[1] for entry in cw_list)

                    # Set the wall state based on whether the entry is found or not
                    wall.setState(entry_found)

    #------------------------ FUNCTIONS ------------------------

    def centerText(text_item, center_x, center_y):
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

class Wall_Config:
    """ Stores the wall configuration for the maze """

    #------------------------ CLASS VARIABLES ------------------------
    # Stores the wall configuration list
    wallConfigList = []

    @classmethod
    def Reset(cls):
        """Resets the wall configuration list"""
        cls.wallConfigList = []

    @classmethod
    def getLen(cls):
        """Returns the number of entries in the wall configuration list"""
        return len(cls.wallConfigList)

    @classmethod
    def addWall(cls, chamber_num, wall_num):
        """Adds a wall to the wall configuration list"""
        for item in cls.wallConfigList:
            if item[0] == chamber_num:
                item[1].append(wall_num)
                return
        cls.wallConfigList.append([chamber_num, [wall_num]])

    @classmethod
    def removeWall(cls, chamber_num, wall_num):
        """Removes a wall from the wall configuration list"""
        for item in cls.wallConfigList:
            if item[0] == chamber_num:
                item[1].remove(wall_num)
                if not item[1]:  # If the second column is empty, remove the entire row
                    cls.wallConfigList.remove(item)
                return

    @classmethod
    def makeByte2WallList(cls, wall_byte_config_list):
        """Converts a list of byte values to a wall configuration list entries"""

        # Clear the existing wall_config_list
        cls.wallConfigList = []

        # Convert the byte values to arrays and update the wall_config_list
        for row in wall_byte_config_list:
            chamber_num = row[0]
            byte_value = row[1]

            # Convert the byte_value back to an array of wall numbers
            wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

            cls.wallConfigList.append([chamber_num, wall_numbers])

    @classmethod
    def makeWall2ByteList(cls):
        """Returns a list with byte values for each chamber corespoinding to the wall configuration"""  

        wall_byte_config_list = []
        for row in cls.wallConfigList:  # row = [chamber_num, wall_numbers]
            chamber_num = row[0]
            wall_arr = row[1]
            # Initialize the byte value
            byte_value = 0
            # Iterate over the array of values
            for wall_i in wall_arr:
                if 0 <= wall_i <= 7:
                    # Set the corresponding bit to 1 using bitwise OR
                    byte_value |= (1 << wall_i)
            wall_byte_config_list.append([chamber_num, byte_value])
        return wall_byte_config_list

    @classmethod
    def getWall2ByteList(cls):
        """Returns a list with byte values for each chamber corespoinding to the wall configuration"""

        wall_byte_config_list = cls.makeWall2ByteList()

        # Update U_arr with corresponding chamber and wall byte
        wall_arr = [0] * len(cls.wallConfigList)
        for cw in wall_byte_config_list:
            wall_arr[cw[0]] = cw[1]

        return wall_arr

    @classmethod
    def sortEntries(cls):
        """Sorts the entries in the wall configuration list by chamber number and wall numbers"""

        # Sort the rows by the entries in the based on the first chamber number
        cls.wallConfigList.sort(key=lambda row: row[0])

        # Sort the arrays in the second column
        for row in cls.wallConfigList:
            row[1].sort()

    @classmethod
    def __iter__(cls):
        """Returns an iterator for the wall configuration list"""
        return iter(cls.wallConfigList)

    @classmethod
    def __str__(cls):
        """Returns the wall configuration list as a string"""
        return str(cls.wallConfigList)

class MessageType(Enum):
    """ Enum for ethercat python to arduino message type ID """
    MSG_NONE = 0
    CONFIRM_DONE = 1
    HANDSHAKE = 2
    MOVE_WALLS = 3
    START_SESSION = 4
    END_SESSION = 5
    ERROR = 6

class ErrorType(Enum):
    """ Enum for tracking errors """
    ERROR_NONE = 0
    MESSAGE_ID_DISORDERED = 1
    NO_MESSAGE_TYPE_MATCH = 2
    REGISTER_LEFTOVERS = 3
    MISSING_FOOTER = 4
    N_ErrorType = 5

class Esmacat_Com:
    """ 
    This class is used to communicate with the arduino via ethercat.
    It is used to send and receive messages from the arduino.
    """ 
    #------------------------ CLASS VARIABLES ------------------------

    # Handshake finished flag
    isHandshakeDone = False

    #------------------------ NESTED CLASSES ------------------------

    class RegUnion(ctypes.Union):
        """ C++ style Union for storing ethercat data shareable accross 8 and 16-bit data types """
        _fields_ = [("ui8", ctypes.c_uint8 * 16),
                    ("ui16", ctypes.c_uint16 * 8),
                    ("i16", ctypes.c_int16 * 8),
                    ("ui64", ctypes.c_uint64 * 2)]
    
    class UIndStruct:
        """ Struct for storing ethercat data shareable accross 8 and 16-bit data types """
        def __init__(self):
            self.i8 = 0  # 8-bit index
            self.i16 = 0 # 16-bit index
        
        def upd8(self, b_i=255):
            """Update union 8-bit and 16-bit index and return last updated 8-bit index"""
            b_i = b_i if b_i != 255 else self.i8
            self.i8 = b_i + 1
            self.i16 = self.i8 // 2
            return b_i

        def upd16(self, b_i=255):
            """Update union 16-bit and 8-bit index and return last updated 16-bit index"""
            b_i = b_i if b_i != 255 else self.i16
            self.i16 = b_i + 1
            self.i8 = self.i16 * 2
            return b_i

        def reset(self):
            """Reset union 8-bit and 16-bit index"""
            self.i8 = 0
            self.i16 = 0
    
    class EcatMessageStruct:
        """ Struct for storing ethercat messages """
        def __init__(self):
            self.RegU = Esmacat_Com.RegUnion()    # Union for storing ethercat 8 16-bit reg entries
            self.getUI = Esmacat_Com.UIndStruct() # Union index handler for getting union data
            self.setUI = Esmacat_Com.UIndStruct() # Union index handler for getting union data
            self.msgID = 0
            self.msgTp = MessageType.MSG_NONE
            self.msgFoot = [0, 0]
            self.argLen = 0
            self.ArgU = Esmacat_Com.RegUnion()    # Union for storing message arguments
            self.argUI = Esmacat_Com.UIndStruct() # Union index handler for argument union data
            self.errTp = ErrorType.ERROR_NONE
            self.isDone = False
    
    def __init__(self):

        # Initialize ethercat message handler instances
        self.sndEM = self.EcatMessageStruct()  # Initialize message handler instance for sending messages
        self.rcvEM = self.EcatMessageStruct()  # Initialize message handler instance for receiving messages
        self.tmpEM = self.EcatMessageStruct()  # Initialize message handler instance for temporary receiving messages

        # ROS Publisher: Initialize ethercat message handler instances
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)  # Esmacat write maze ard0 ease publisher

    #------------------------ PRIVATE METHODS ------------------------

    def _uSetMsgID(self, r_EM, msg_id=255):
        """Set message ID entry in union and update associated variable"""
        
        # Get new message ID: itterate id and roll over to 1 if max 16-bit value is reached
        if msg_id == 255:
            msg_id = r_EM.msgID + 1 if r_EM.msgID < 65535 else 1

        # Set message ID entry in union
        r_EM.RegU.ui16[r_EM.setUI.upd16(0)] = msg_id
        self._uGetMsgID(r_EM)  # copy from union to associated struct variable

    def _uGetMsgID(self, r_EM):
        """Get message ID from union"""

        r_EM.msgID = r_EM.RegU.ui16[r_EM.getUI.upd16(0)]

    def _uSetMsgType(self, r_EM, msg_type_enum):
        """Set message type entry in union and update associated variable"""

        # Get new message type: convert enum to int
        msg_type_val = msg_type_enum.value

        # Set message type entry in union
        r_EM.RegU.ui8[r_EM.setUI.upd8(2)] = msg_type_val
        self._uGetMsgType(r_EM)  # copy from union to associated struct variable

    def _uGetMsgType(self, r_EM):
        """Get message type from union and check if valid"""

        # Get message type value
        msg_type_val = r_EM.RegU.ui8[r_EM.getUI.upd8(2)]

        # Check if 'msg_type_val' corresponds to any value in the 'MessageType' Enum.
        is_found = msg_type_val in [e.value for e in MessageType]
        is_err = not is_found

        # If not found, set message type to 'MSG_NONE'
        if is_err:
            msg_type_val = MessageType.MSG_NONE
        else:
            r_EM.msgTp = MessageType(msg_type_val)

        return is_err

    def _uSetArgLength(self, r_EM, msg_arg_len):
        """Set message argument length entry in union and update associated variable"""
        
        # Set argument length union entry
        r_EM.RegU.ui8[r_EM.setUI.upd8(3)] = msg_arg_len
        self._uGetArgLength(r_EM) # copy from union to associated struct variable

    def _uGetArgLength(self, r_EM):
        """Get message argument length"""

        # Get argument length union entry
        r_EM.argLen = r_EM.RegU.ui8[r_EM.getUI.upd8(3)]

    def _uSetArgData8(self, r_EM, msg_arg_data8):
        """Set message 8-bit argument data entry in union"""
        
        if isinstance(msg_arg_data8, int) and msg_arg_data8 <= 255:  # check for 8-bit int data

            # Increment argument union index
            r_EM.argUI.upd8()

            # Update message argument length from argument union 8-bit index
            self._uSetArgLength(r_EM, r_EM.argUI.i8)

            # Get 8-bit union index and set 8-bit argument data entry in union
            regu8_i = r_EM.argUI.i8 + 3

            # Set message argument data in reg union
            r_EM.RegU.ui8[r_EM.setUI.upd8(regu8_i)] = msg_arg_data8
            self._uGetArgData8(r_EM) # copy from union to associated struct variable

    def _uSetArgData16(self, r_EM, msg_arg_data16):
        """Set message 16-bit argument data entry in union"""

        if isinstance(msg_arg_data16, int) and msg_arg_data16 > 255:  # check for 16-bit int data

            # Increment argument union index
            r_EM.argUI.upd16()
            
            # Update message argument length from argument union 8-bit index
            self._uSetArgLength(r_EM, r_EM.argUI.i8)
            
            # Get 16-bit union index and set 16-bit argument data entry in union
            regu16_i = (r_EM.argUI.i8 + 3) // 2
            
            # Set message argument data in reg union
            r_EM.RegU.ui16[r_EM.setUI.upd16(regu16_i)] = msg_arg_data16
            self._uGetArgData8(r_EM) # copy from union to associated struct variable

    def _uGetArgData8(self, r_EM):
        """Get reg union 8-bit message argument data and copy to arg union"""

        self._uGetArgLength(r_EM); # get argument length from union
        for i in range(r_EM.argLen):
            r_EM.ArgU.ui8[i] = r_EM.RegU.ui8[r_EM.getUI.upd8()] # copy to 8-bit argument Union

    def _uSetFooter(self, r_EM):
        """Set message footer entry in union and update associated variable"""

        r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254
        r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254
        self._uGetFooter(r_EM) # copy from union to associated struct variable

    def _uGetFooter(self, r_EM):
        """Get message footer from union"""

        r_EM.msgFoot[0] = r_EM.RegU.ui8[r_EM.getUI.upd8()] # copy first footer byte
        r_EM.msgFoot[1] = r_EM.RegU.ui8[r_EM.getUI.upd8()] # copy second footer byte
        is_err = r_EM.msgFoot[0] != 254 or r_EM.msgFoot[1] != 254 # check if footer is valid
        return is_err

    def _uReset(self, r_EM):
        """Reset union data and indices"""

        # Reset union data
        r_EM.RegU.ui64[0] = 0
        r_EM.RegU.ui64[1] = 0

        # Reset union indices
        r_EM.setUI.reset()
        r_EM.getUI.reset()
        r_EM.argUI.reset()

    def _checkErr(self, r_EM, err_tp, is_err):
        """Check if error type is valid and set error type"""

        # Check for error
        if is_err:

            # Run only once
            if r_EM.errTp != err_tp:  
                
                # Set error type
                r_EM.errTp = err_tp

                # Print error message
                rospyLogCol(
                    'ERROR', "!!ERROR: Ecat: %s: id=%d type=%s[%d]!!", r_EM.errTp.name, r_EM.msgID, r_EM.msgTp.name, r_EM.msgTp.value)
                self._printEcatReg('DEBUG', 0, r_EM.RegU)  # TEMP
        
        # Unset error type
        else:
            if r_EM.errTp == err_tp:
                r_EM.errTp = ErrorType.ERROR_NONE  

    def _printEcatReg(self, level, d_type, reg_u):
        """Print EtherCAT register data"""

        # Print heading with type
        rospyLogCol(level, "\t Ecat Register")

        # Print message data
        for i in range(8):
            if d_type == 1 or i == 0:
                rospyLogCol(level, "\t\t ui16[%d] %d", i, reg_u.ui16[i])
            if d_type == 0:
                rospyLogCol(level, "\t\t\t ui8[%d]  %d %d", i, reg_u.ui8[2 * i], reg_u.ui8[2 * i + 1])

    #------------------------ PUBLIC METHODS ------------------------

    def msgReset(self):
        """Reset all message structs."""

        # Reset message counters
        self.sndEM.msgID = 0
        self.tmpEM.msgID = 0
        self.rcvEM.msgID = 0

        # Reset Ethercat handshake flag
        self.isHandshakeDone = False

    def sendEcatMessage(self, msg_type_enum, msg_arg_data_arr=None, msg_arg_len=0):
        """
        Used to parse new incoming ROS ethercat msg data.

        Args:
            msg_type_enum (MessageType): Message type enum.
            msg_arg_data_arr (list): Message argument data array.
            msg_arg_len (int): Message argument length (bytes).

        Returns:
            int: Success/error codes [0:no message, 1:new message, 2:error]
        """

        # Reset union variables
        self._uReset(self.sndEM)

        # Store new message id
        self._uSetMsgID(self.sndEM)

        # Store new message type
        self._uSetMsgType(self.sndEM, msg_type_enum)

        # Store arguments based on the message type
        if self.sndEM.msgTp == MessageType.CONFIRM_DONE:
            self._uSetArgData16(self.sndEM, self.rcvEM.msgID)  # store 16-bit received message id
            self._uSetArgData8(self.sndEM, self.rcvEM.msgTp.value)  # received message type value

        else:
            if msg_arg_data_arr is not None:  # store message arguments if provided
                for i in range(msg_arg_len):
                    self._uSetArgData8(self.sndEM, msg_arg_data_arr[i])

        # Update associated argument variables in struct
        self._uGetArgLength(self.sndEM)

        # Store footer
        self._uSetFooter(self.sndEM)

        # Set flag
        self.sndEM.isDone = False

         # Store Union 16-bit values cast as signed and publish to ease_registers topic
        reg_arr = [0] * 8
        for i in range(8):
            reg_arr[i] = self.sndEM.RegU.i16[i] # cast as ctype signed int (e.g., uint16))
        self.maze_ard0_pub.publish(*reg_arr)  

        # Print message
        rospyLogCol('INFO', "SENT Ecat Message: id=%d type=%s",
                    self.sndEM.msgID, self.sndEM.msgTp.name)
        self._printEcatReg('INFO', 0, self.sndEM.RegU)  # TEMP

    def parseEcatMessage(self, reg_arr):
        """
        Used to parse new incoming ROS ethercat msg data.

        Args:
            reg_arr (list): Array of length 8 from the ethercat register.
        
        Returns:
            int: Success/error codes [0:no message, 1:new message, 2:error]
        """
        is_err = False  # error flag

         # Copy register data into union
        self._uReset(self.sndEM) # reset union variables
        for i in range(8):
            self.tmpEM.RegU.ui16[i] = reg_arr[i]

        # Skip ethercat setup junk (255)
        if self.tmpEM.RegU.ui8[0] == 255 or self.tmpEM.RegU.ui8[1] == 255:
            return 0

        # Get message id
        self._uGetMsgID(self.tmpEM)

        # Skip redundant messages
        if self.tmpEM.msgID == self.rcvEM.msgID:
            return 0

        # Get message type and check if valid
        is_err = self._uGetMsgType(self.tmpEM)

        # Run check error for valid message type
        self._checkErr(self.tmpEM, ErrorType.NO_MESSAGE_TYPE_MATCH, is_err)
        if is_err:
            return 2  # return error flag

        # Check if message is preceding handshake and confirm done messages
        is_err = not self.isHandshakeDone and (
            self.tmpEM.msgTp != MessageType.CONFIRM_DONE or self.tmpEM.msgID > 1
        )

        self._checkErr(self.tmpEM, ErrorType.REGISTER_LEFTOVERS, is_err)
        if is_err:
            return 2  # return error flag

        # Check for skipped or out of sequence messages
        is_err = self.tmpEM.msgID - self.rcvEM.msgID != 1
        self._checkErr(self.tmpEM, ErrorType.MESSAGE_ID_DISORDERED, is_err)
        if is_err:
            return 2  # return error flag

        # Get argument length and arguments
        self._uGetArgLength(self.tmpEM)
        self._uGetArgData8(self.tmpEM) 

        # Get and check for footer
        is_err = self._uGetFooter(self.tmpEM)

        # Run check error for valid footer
        self._checkErr(self.tmpEM, ErrorType.MISSING_FOOTER, is_err)
        if is_err:
            return 2  # return error flag

        # Set flag
        self.tmpEM.isDone = False

        # Copy over data
        self.rcvEM = self.tmpEM

        # Print message
        rospyLogCol('INFO', "RECEIVED Ecat Message: id=%d type=%s",
                    self.rcvEM.msgID, self.rcvEM.msgTp.name)
        self._printEcatReg('INFO', 0, self.rcvEM.RegU)  # TEMP

        # Return new message flag
        return 1

#======================== MAIN UI CLASS ========================
# CLASS: Interface plugin
class Interface(Plugin):
    """ Interface plugin """

    # Define signals
    signal_Esmacat_read_maze_ard0_ease = Signal()

    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

        #................ QT UI Setup ................ 

        # Give QObjects reasonable names
        self.setObjectName('Interface')
        rospyLogCol('INFO', "Running Interface setup")

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

        # Initialize ardListWidget with arduino names.
        # Add 8 arduinos to the list labeled Arduino 0-9
        for i in range(8):
            self._widget.ardListWidget.addItem("Arduino " + str(i))
        # Hide the blue selection bar
        self._widget.ardListWidget.setStyleSheet(
            "QListWidget::item { border-bottom: 1px solid black; }")
        # Change the text color for all items in the list to light gray
        self._widget.ardListWidget.setStyleSheet(
            "QListWidget { color: lightgray; }")

        #................ Maze Setup ................

        # Calculate chamber width and wall line width and offset
        maze_view_size = self._widget.plotMazeView.width()
        chamber_width = self._widget.plotMazeView.width()*0.9/NUM_ROWS_COLS
        wall_width = chamber_width*0.1

        # Create Maze_Plot.Maze and populate walls according to WALL_MAP
        self.MazePlot = Maze_Plot.Maze(num_rows=NUM_ROWS_COLS,
                                  num_cols=NUM_ROWS_COLS,
                                  chamber_width=chamber_width,
                                  wall_width=wall_width)

        # Add chambers and disable walls not connected
        for k, c in enumerate(self.MazePlot.chambers):
            self.scene.addItem(c)
            for j, w in enumerate(c.walls):
                if j not in WALL_MAP[k]:
                    w.setEnabled(False)
                    w.setVisible(False)

        # Add walls - this is a new loop so that they are drawn above the chambers
        for c in self.MazePlot.chambers:
            for w in c.walls:
                self.scene.addItem(w)

        #................ Ecat Setup ................

        # Create Esmacat_Com object
        self.EsmaCom_A0 = Esmacat_Com()

        # Store the time the interface was initialized
        self.ts_interface_init = rospy.get_time() # (sec)

        # Specify delay to start reading/writing Esmacat data 
        self.dt_ecat_start = 1 # (sec)

        # Specify delay between Esmacat checks reads
        self.dt_ecat_check = 0.5 # (sec)

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

        #................ ROS Setup ................

        # @obsolete ROS Sublisher: @obsolete 
        # wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)

        # ROS Subscriber: Esmacat arduino maze ard0 ease
        rospy.Subscriber(
            'Esmacat_read_maze_ard0_ease', ease_registers, self.ros_callback_Esmacat_read_maze_ard0_ease, tcp_nodelay=True)

        # Signal callback setup
        self.signal_Esmacat_read_maze_ard0_ease.connect(
            self.sig_callback_Esmacat_read_maze_ard0_ease)


    #------------------------ FUNCTIONS: Ecat Communicaton ------------------------

    def procEcatArguments(self):
        """
        Used to parse new incoming ROS ethercat msg data.
        
        Returns:
            int: Success/error codes [0:no message, 1:new message, 2:error]
        """

        #................ Process Arguments ................ 

        # Create temp EcatMessageStruct object for confirmation message
        cnf_em = self.EsmaCom_A0.EcatMessageStruct()

        # CONFIRM_DONE
        if self.EsmaCom_A0.rcvEM.msgTp == MessageType.CONFIRM_DONE:

            # Get confirmation message number
            cnf_em.msgID = self.EsmaCom_A0.rcvEM.ArgU.ui16[0]

            # Get message number
            cnf_em.msgTp = MessageType(self.EsmaCom_A0.rcvEM.ArgU.ui8[2])

            # Print confirmation message
            rospyLogCol('INFO', "CONFIRMED DONE Ecat Message: type=%s id=%d",
                        cnf_em.msgTp, cnf_em.msgID)

        # ERROR
        elif self.EsmaCom_A0.rcvEM.msgTp == MessageType.ERROR:
            pass

        #................ Execute Arguments ................ 

        # CONFIRM_DONE
        if self.EsmaCom_A0.rcvEM.msgTp == MessageType.CONFIRM_DONE:

            # Check if message is a handshake confirmation
            if cnf_em.msgTp == MessageType.HANDSHAKE:

                # Set the handshake flag
                self.EsmaCom_A0.isHandshakeDone = True
                rospyLogCol('INFO', "ECAT COMMS ESTABLISHED");

                # Send START_SESSION message
                self.EsmaCom_A0.sendEcatMessage(MessageType.START_SESSION)
    
    #------------------------ CALLBACKS: ROS ------------------------

    def ros_callback_Esmacat_read_maze_ard0_ease(self, msg):
        """ ROS callback for Esmacat_read_maze_ard0_ease topic """

        # Wait for interface to initialize
        current_time = rospy.get_time()
        if (current_time - self.ts_interface_init) < self.dt_ecat_start:  # Less than 100ms
            return

        # Store ethercat message in class variable
        si16_arr = [0]*8
        si16_arr[0] = msg.INT0
        si16_arr[1] = msg.INT1
        si16_arr[2] = msg.INT2
        si16_arr[3] = msg.INT3
        si16_arr[4] = msg.INT4
        si16_arr[5] = msg.INT5
        si16_arr[6] = msg.INT6
        si16_arr[7] = msg.INT7

        # Convert 16-bit signed ints to 16-bit unsigned ints array
        ui16_arr = [0]*8
        for i in range(8):
            ui16_arr[i] = ctypes.c_uint16(si16_arr[i]).value

        # Parse the message and check if its new
        if self.EsmaCom_A0.parseEcatMessage(ui16_arr) != 1:
            return
        
        # Process the message arguments
        self.procEcatArguments()
                
        # Emit signal to update UI as UI should not be updated from a non-main thread
        # TEMP self.signal_Esmacat_read_maze_ard0_ease.emit()

    def sig_callback_Esmacat_read_maze_ard0_ease(self):
        """ Signal callback for Esmacat_read_maze_ard0_ease topic """

    #------------------------ CALLBACKS: Timers ------------------------

    def timer_callback_sendHandshake_once(self):
        """ Timer callback to send handshake message once resend till confirmation. """

        # Kill handshake check timer
        # self.timer_sendHandshake.stop()
        
        # Check for HANDSHAKE message confirm recieved flag
        if self.EsmaCom_A0.isHandshakeDone == False:

            # Give up is more that 3 messages have been sent
            if self.EsmaCom_A0.sndEM.msgID > 3:
                rospyLogCol(
                    'ERROR', "!!ERROR: Handshake failed: msg_id=%d!!", self.EsmaCom_A0.sndEM.msgID)
                return
            elif self.EsmaCom_A0.sndEM.msgID > 1:
                rospyLogCol(
                    'WARNING', "!!WARNING: Handshake failed: msg_id=%d!!", self.EsmaCom_A0.sndEM.msgID)

            # Send HANDSHAKE message to arduino 
            self.EsmaCom_A0.sendEcatMessage(MessageType.HANDSHAKE)

            # Restart check timer
            self.timer_sendHandshake.start(self.dt_ecat_check*1000)

    def timer_callback_updateUI_loop(self):
        """ Timer callback to update UI. """

        # Update graphics
        self.scene.update()
        self._widget.plotMazeView.update()

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

        Wall_Config.Reset()  # reset all values in list
        self.MazePlot.updateWalls()  # update walls

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
            rospyLogCol('INFO', "Selected file:", file_name)

            # Call the function to save wall config data to the CSV file with the wall array values converted to bytes
            self.saveToCSV(file_name, Wall_Config.makeWall2ByteList())

    def qt_callback_plotSendBtn_clicked(self):
        """ Callback function for the "Send" button."""

        # Sort entries
        Wall_Config.sortEntries()

        # Send the wall byte array to the arduino
        self.EsmaCom_A0.sendEcatMessage(
            MessageType.MOVE_WALLS, 9, Wall_Config.getWall2ByteList())

    def qt_callback_sysQuiteBtn_clicked(self):
        """ Callback function for the "Quit" button."""

        # Call function to shut down the ROS session
        self.endRosSession()
        # End the application
        QApplication.quit()

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
            rospyLogCol('INFO', "Data saved to:", file_name)
        except Exception as e:
            rospyLogCol('ERROR', "Error saving data to CSV:", str(e))

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
                Wall_Config.makeByte2WallList(wall_byte_config_list)
                rospyLogCol('INFO', "Data loaded successfully.")
        except Exception as e:
            rospyLogCol('ERROR', "Error loading data from CSV:", str(e))

        # Update plot walls
        self.MazePlot.updateWalls()

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

    def terminate_process_and_children(self, p):
        """ Function to terminate a process and all its children when exiting the application """

        ps_command = subprocess.Popen(
            "ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        p.terminate()
        p.kill()

    def endRosSession(self):
        """ Function to end the ROS session """

        # Send END_SESSION message to arduino
        self.EsmaCom_A0.sendEcatMessage(MessageType.END_SESSION)

        # Kill self.signal_Esmacat_read_maze_ard0_ease.emit() thread
        # TEMP self.thread_Esmacat_read_maze_ard0_ease.terminate()

        # Kill specific nodes
        self.terminate_ros_node("/Esmacat_application_node")
        self.terminate_ros_node("/interface_test_node")

        # Wait for nodes to shutdown
        time.sleep(1)  # (sec)

        # Kill all nodes (This will also kill this script's node)
        os.system("rosnode kill -a")

        # Process any pending events in the event loop
        QCoreApplication.processEvents()

        # Close the UI window
        self._widget.close()

        # Send a shutdown request to the ROS master
        rospy.signal_shutdown("User requested shutdown")

    """ @todo: Implement this function to handle the window close event """
    def closeEvent(self, event):
        """ Function to handle the window close event """
        
        rospyLogCol('INFO', "Closing window...")
        # Call function to shut down the ROS session
        self.endRosSession()
        event.accept()  # let the window close

