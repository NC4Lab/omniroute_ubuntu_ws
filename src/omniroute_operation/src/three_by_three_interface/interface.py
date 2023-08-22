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
        """
        Used to convert imported CSV with wall byte mask values to a list with wall numbers

        Args:
            wall_byte_config_list (list): 2D list: col1 = chamber number, col2 = wall byte mask
        
        Returns:
            2D list: col1 = chamber number, col2 = nested wall numbers
        """

        # Clear the existing wall_config_list
        cls.wallConfigList = []

        # Convert the byte values to arrays and update the wall_config_list
        for row in wall_byte_config_list:
            chamber_num = row[0]
            byte_value = row[1]

            # Convert the byte_value back to an array of wall numbers
            wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

            cls.wallConfigList.append([chamber_num, wall_numbers])

            return cls.wallConfigList

    @classmethod
    def makeWall2ByteList(cls):
        """
        Used to covert wall number arrays to byte values for saving to CSV
        
        Returns:
            2D list: col1 = chamber number, col2 = wall byte mask"""  

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
    def getWallByteOnlyList(cls):
        """
        Used to generate a 1D list with only byte values for each chamber corespoinding to the wall configuration
        
        Returns: 
            1D list with byte values for all chambers"""

        wall_byte_config_list = cls.makeWall2ByteList()

        # Update U_arr with corresponding chamber and wall byte
        wall_byte_arr = [0] * N_CHAMBERS
        #wall_arr = [0] * len(cls.wallConfigList)
        for cw in wall_byte_config_list:
            wall_byte_arr[cw[0]] = cw[1]

        return wall_byte_arr

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

class Esmacat_Com:
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
        INITIALIZE_SYSTEM = 2
        REINITIALIZE_SYSTEM = 3
        MOVE_WALLS = 4

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
            self.RegU = Esmacat_Com.RegUnion()    # Union for storing ethercat 8 16-bit reg entries
            self.getUI = Esmacat_Com.UnionIndStruct() # Union index handler for getting union data
            self.setUI = Esmacat_Com.UnionIndStruct() # Union index handler for getting union data

            self.msgID = 0
            self.msgID_last = 0                     # Last message ID   
            self.msgTp = Esmacat_Com.MessageType.MSG_NULL # Message type
            self.msgFoot = [0, 0]                  # Message footer

            self.argLen = 0                       # Message argument length
            self.ArgU = Esmacat_Com.RegUnion()    # Union for storing message arguments
            self.argUI = Esmacat_Com.UnionIndStruct() # Union index handler for argument union data

            self.isNew = False                         # New message flag
            self.isErr = False                         # Message error flag
            self.errTp = Esmacat_Com.ErrorType.ERR_NULL # Message error type
    
    def __init__(self):

        # Initialize ethercat message handler instances
        self.sndEM = self.EcatMessageStruct()  # Initialize message handler instance for sending messages
        self.rcvEM = self.EcatMessageStruct()  # Initialize message handler instance for receiving messages

        # ROS Publisher: Initialize ethercat message handler instances
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)  # Esmacat write maze ard0 ease publisher

    #------------------------ PRIVATE METHODS ------------------------
    
    def _uSetCheckReg(self, r_EM, reg_arr_si16):
        """Set register values in union and check for -1 values indicating no or incomplete messages"""

        # Bail if any register values are equal to -1
        if -1 in reg_arr_si16:
            return False

        # Set register values in union uint16 type
        for i in range(8):
            r_EM.RegU.si16[i] = reg_arr_si16[i]

        return True

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

        r_EM.msgID_last = r_EM.msgID # store last message ID if
        r_EM.msgID = r_EM.RegU.ui16[r_EM.getUI.upd16(0)]

        # Check/log error skipped or out of sequence messages
        if r_EM.msgID - r_EM.msgID_last != 1 and \
            r_EM.msgID != r_EM.msgID_last: # don't log errors for repeat message reads
            self._trackErrType(r_EM, Esmacat_Com.ErrorType.ECAT_ID_DISORDERED)

    def _uSetMsgType(self, r_EM, msg_type_enum):
        """Set message type entry in union and update associated variable"""

        # Set message type entry in union
        r_EM.RegU.ui8[r_EM.setUI.upd8(2)] = msg_type_enum.value
        self._uGetMsgType(r_EM)  # copy from union to associated struct variable

    def _uGetMsgType(self, r_EM):
        """Get message type from union and check if valid"""

        # Get message type value
        msg_type_val = r_EM.RegU.ui8[r_EM.getUI.upd8(2)]

        # Check if 'msg_type_val' corresponds to any value in the 'Esmacat_Com.MessageType' Enum.
        is_found = msg_type_val in [e.value for e in Esmacat_Com.MessageType]
        r_EM.isErr = not is_found # Update struct error flag

        # Log error and set message type to none if not found
        if r_EM.isErr:
            msg_type_val = Esmacat_Com.MessageType.MSG_NULL
            self._trackErrType(r_EM, Esmacat_Com.ErrorType.ECAT_NO_MSG_TYPE_MATCH) 
        else:
            r_EM.msgTp = Esmacat_Com.MessageType(msg_type_val)

    def _uSetErrType(self, r_EM, err_type_enum):
        """Set message type entry in union and update associated variable"""

        # Set message type entry in union
        r_EM.RegU.ui8[r_EM.setUI.upd8(3)] = err_type_enum.value
        self._uGetErrType(r_EM)  # copy from union to associated struct variable

    def _uGetErrType(self, r_EM):
        """Get message type from union and check if valid"""

        # Get message type value
        err_type_val = r_EM.RegU.ui8[r_EM.getUI.upd8(3)]

        # Check if 'err_type_val' corresponds to any value in the 'Esmacat_Com.ErrorType' Enum.
        is_found = err_type_val in [e.value for e in Esmacat_Com.ErrorType]
        r_EM.isErr = not is_found # Update struct error flag

        # Log error and set error type to none if not found
        if r_EM.isErr:
            err_type_val = Esmacat_Com.ErrorType.ERR_NULL
            self._trackErrType(r_EM, Esmacat_Com.ErrorType.ECAT_NO_ERR_TYPE_MATCH) 
        else:
            r_EM.errTp = Esmacat_Com.ErrorType(err_type_val)

    def _uSetArgLength(self, r_EM, msg_arg_len):
        """Set message argument length entry in union and update associated variable"""
        
        # Set argument length union entry
        r_EM.RegU.ui8[r_EM.setUI.upd8(4)] = msg_arg_len
        self._uGetArgLength(r_EM) # copy from union to associated struct variable

    def _uGetArgLength(self, r_EM):
        """Get message argument length"""

        # Get argument length union entry
        r_EM.argLen = r_EM.RegU.ui8[r_EM.getUI.upd8(4)]

    def _uSetArgData8(self, r_EM, msg_arg_data8):
        """Set message 8-bit argument data entry in union"""
        
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
        """Set message 16-bit argument data entry in union"""

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

        # Log missing footer error
        if r_EM.msgFoot[0] != 254 or r_EM.msgFoot[1] != 254: # check if footer is valid
            self._trackErrType(r_EM, Esmacat_Com.ErrorType.ECAT_MISSING_FOOTER) 

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
        """Check for and log any Ecat or runtime errors"""

        # Check for error
        if not do_reset:

            # Run only once
            if r_EM.errTp != err_tp:  
                
                # Set error type and flag
                r_EM.errTp = err_tp
                r_EM.isErr = True

                # Print error message
                rospyLogCol(
                    'ERROR', "!!ERROR: Ecat: %s: id[new,last]=[%d,%d] type=%s[%d]!!", r_EM.errTp.name, r_EM.msgID, r_EM.msgID_last, r_EM.msgTp.name, r_EM.msgTp.value)
                self._printEcatReg('ERROR', 0, r_EM.RegU)  # TEMP
        
        # Unset error type
        elif r_EM.errTp == err_tp:
                r_EM.errTp = Esmacat_Com.ErrorType.ERR_NULL  
                r_EM.isErr = False

    def _printEcatReg(self, level, d_type, reg_u):
        """Print EtherCAT register data"""

        # Print heading with type
        rospyLogCol(level, "\t Ecat Register")

        # Print message data
        for i in range(8):
            if d_type == 2:
                rospyLogCol(level, "\t\t si16[%d] %d", i, reg_u.si16[i]) 
            if d_type == 1:
                rospyLogCol(level, "\t\t ui16[%d] %d", i, reg_u.ui16[i])
            if d_type == 0:
                rospyLogCol(level, "\t\t\t ui8[%d][%d]  %d %d", 2 * i, 2 * i + 1, reg_u.ui8[2 * i], reg_u.ui8[2 * i + 1])

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

    def parseEcatMessage(self, reg_arr_si16):
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

        # Get message id
        self._uGetMsgID(self.rcvEM)

        # Skip redundant messages
        if self.rcvEM.msgID == self.rcvEM.msgID_last:
            return False

        # Get message type and flag if not valid
        self._uGetMsgType(self.rcvEM)

        # Get error type and flag if not valid
        self._uGetErrType(self.rcvEM)

        # Get argument length and arguments
        self._uGetArgData8(self.rcvEM) 

        # Get footer and flag if not found
        self._uGetFooter(self.rcvEM)

        # @todo: figure out what to do if message corrupted

        # Set new message flag
        self.rcvEM.isNew = True

        # Print message
        rospyLogCol('INFO', "(%d)ECAT ACK RECEIVED: %s", self.rcvEM.msgID, self.rcvEM.msgTp.name)
        self._printEcatReg('INFO', 0, self.rcvEM.RegU)  # TEMP

        return True

    def writeEcatMessage(self, msg_type_enum, msg_arg_data=None):
        """
        Used to send outgoing ROS ethercat msg data.

        Args:
            msg_type_enum (Esmacat_Com.MessageType): Message type enum.
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
        rospyLogCol('INFO', "(%d)ECAT SENT: %s", self.sndEM.msgID, self.sndEM.msgTp.name)
        self._printEcatReg('INFO', 0, self.sndEM.RegU)  # TEMP

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

        # Specify delay to start and check reading/writing Esmacat data 
        self.dt_ecat_start = 1 # (sec)
        self.dt_ecat_check = 0.5 # (sec)

        # Couter to incrementally shut down opperations
        self.cnt_shutdown_step = 0
        self.dt_shutdown_step = 0.5 # (sec)

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


    #------------------------ FUNCTIONS: Ecat Communicaton ------------------------

    def procEcatMessage(self):
        """
        Used to parse new incoming ROS ethercat msg data.
        
        Returns:
            int: Success/error codes [0:no message, 1:new message, 2:error]
        """

        # Print confirmation message
        rospyLogCol('INFO', "(%d)PROCESSING ECAT ACK: %s", 
                    self.EsmaCom_A0.rcvEM.msgID, self.EsmaCom_A0.rcvEM.msgTp.name)
        
        #................ Process Ack Error ................ 

        if self.EsmaCom_A0.rcvEM.errTp != Esmacat_Com.ErrorType.ERR_NULL:

            rospyLogCol(
                'ERROR', "!!ERROR: ECAT ERROR: %s!!", self.EsmaCom_A0.rcvEM.errTp.name)
        
        #................ Process Ack Success ................ 

        # HANDSHAKE
        if self.EsmaCom_A0.rcvEM.msgTp == Esmacat_Com.MessageType.HANDSHAKE:

            # Set the handshake flag
            self.EsmaCom_A0.isEcatConnected = True
            rospyLogCol('INFO', "=== ECAT COMMS CONNECTED ===");

            # Send INITIALIZE_SYSTEM message and specify number of chambers
            self.EsmaCom_A0.writeEcatMessage(Esmacat_Com.MessageType.INITIALIZE_SYSTEM, N_CHAMBERS)

        # REINITIALIZE_SYSTEM
        if self.EsmaCom_A0.rcvEM.msgTp == Esmacat_Com.MessageType.REINITIALIZE_SYSTEM:
            # Set the handshake flag
            self.EsmaCom_A0.isEcatConnected = False
            rospyLogCol('INFO', "=== ECAT COMMS DISCONNECTED ===");

        # Reset new message flag
        self.EsmaCom_A0.rcvEM.isNew = False
    
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
        if self.EsmaCom_A0.parseEcatMessage(reg_arr_si16):
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

            # Give up is more that 3 messages have been sent
            if self.EsmaCom_A0.sndEM.msgID > 3:
                rospyLogCol(
                    'ERROR', "!!ERROR: Handshake failed: msg_id=%d!!", self.EsmaCom_A0.sndEM.msgID)
                return
            elif self.EsmaCom_A0.sndEM.msgID > 1:
                rospyLogCol(
                    'WARNING', "!!WARNING: Handshake failed: msg_id=%d!!", self.EsmaCom_A0.sndEM.msgID)

            # Send HANDSHAKE message to arduino 
            self.EsmaCom_A0.writeEcatMessage(Esmacat_Com.MessageType.HANDSHAKE)

            # Restart check timer
            self.timer_sendHandshake.start(self.dt_ecat_check*1000)

    def timer_callback_endSession_once(self):
        """ Timer callback to incrementally shutdown session. """
        
        if self.cnt_shutdown_step == 0:
            # Send REINITIALIZE_SYSTEM message to arduino
            self.EsmaCom_A0.writeEcatMessage(Esmacat_Com.MessageType.REINITIALIZE_SYSTEM)

        elif self.EsmaCom_A0.isEcatConnected == True:
            # Wait for REINITIALIZE_SYSTEM message confirmation and restart timer
            self.timer_endSession.start(self.dt_shutdown_step*1000)
            return
            
        elif self.cnt_shutdown_step == 1:
            # Kill self.signal_Esmacat_read_maze_ard0_ease thread
            self.signal_Esmacat_read_maze_ard0_ease.disconnect()
            rospyLogCol('INFO', "Disconnected from Esmacat read timer thread");

        elif self.cnt_shutdown_step == 2:
            # Kill specific nodes
            self.terminate_ros_node("/Esmacat_application_node")
            self.terminate_ros_node("/interface_test_node")
            rospyLogCol('INFO', "Killed specific nodes");

        elif self.cnt_shutdown_step == 3:
            # Kill all nodes (This will also kill this script's node)
            os.system("rosnode kill -a")
            rospyLogCol('INFO', "Killed all nodes");

        elif self.cnt_shutdown_step == 4:
            # Process any pending events in the event loop
            QCoreApplication.processEvents()
            rospyLogCol('INFO', "Processed all events");

        elif self.cnt_shutdown_step == 5:
            # Close the UI window
            self._widget.close()
            rospyLogCol('INFO', "Closed UI window");

        elif self.cnt_shutdown_step == 6:
            # Send a shutdown request to the ROS master
            rospy.signal_shutdown("User requested shutdown")
            rospyLogCol('INFO', "Sent shutdown request to ROS master");

        elif self.cnt_shutdown_step == 7:
            # End the application
            QApplication.quit()
            rospyLogCol('INFO', "Ended application");
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
        self.EsmaCom_A0.writeEcatMessage(Esmacat_Com.MessageType.MOVE_WALLS, Wall_Config.getWallByteOnlyList())

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

    """ @todo: Implement this function to handle the window close event """
    def closeEvent(self, event):
        """ Function to handle the window close event """
        
        rospyLogCol('INFO', "Closing window...")
        # Call function to shut down the ROS session
        self.end_ros_session()
        event.accept()  # let the window close

