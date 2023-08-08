#!/usr/bin/env python
import os
import rospy
from omniroute_esmacat_ros.msg import *
import signal
import subprocess
import psutil
import time
import ctypes

from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL
from python_qt_binding.QtCore import QObject

from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsItemGroup, QGraphicsLineItem, QGraphicsTextItem
from PyQt5.QtCore import Qt, QRectF, QCoreApplication
from PyQt5.QtGui import QPen, QColor, QFont

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin
import numpy as np
from scipy.io import loadmat
import math
from colorama import Fore, Style
import ctypes
from enum import Enum
import csv

from std_msgs.msg import *
from omniroute_operation.msg import *

"""
The outgoing register is structured arr[8] of 16 bit integers
with all but first 16 bit value seperated into bytes 
[0]: message number
    ui16 [0-65535] 
[1]: message INFO
    b0 message type [0-255] [see: MsgTypeID]
    b1 arg length [0-255] [number of message args in bytes]           
[none or 2:2-6] data
    wall state bytes
        b0 = wall x byte
        b1 = wall x+1 byte  
[x-7]: footer  
    b1=254
    b0=254      
"""

# GLOBAL VARS
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

# CLASS: Maze_Plot to plot the maze
class Maze_Plot(QGraphicsView):

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
                # rospy_log_col('INFO', "\tchamber %d wall %d clicked in UI" % (self.chamber_num, self.wall_num))
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
                # rospy_log_col('INFO', "\tchamber %d clicked in UI" % self.chamber_num)

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

    # FUNCTION: Center plotted text
    def centerText(text_item, center_x, center_y):
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

# CLASS: The shared Wall_Config storage class to store the wall configuration
class Wall_Config:
    wallConfigList = []

    # Reset the wall configuration list
    @classmethod
    def Reset(cls):
        cls.wallConfigList = []

    # Get the number of entries in the wall configuration list
    @classmethod
    def getLen(cls):
        return len(cls.wallConfigList)

    # Add or remove a wall from the wall configuration list
    @classmethod
    def addWall(cls, chamber_num, wall_num):
        for item in cls.wallConfigList:
            if item[0] == chamber_num:
                item[1].append(wall_num)
                return
        cls.wallConfigList.append([chamber_num, [wall_num]])

    # Remove a wall from the wall configuration list
    @classmethod
    def removeWall(cls, chamber_num, wall_num):
        for item in cls.wallConfigList:
            if item[0] == chamber_num:
                item[1].remove(wall_num)
                if not item[1]:  # If the second column is empty, remove the entire row
                    cls.wallConfigList.remove(item)
                return

    # Convert a list of byte values to a wall configuration list entries
    @classmethod
    def makeByte2WallList(cls, wall_byte_config_list):
        # Clear the existing wall_config_list
        cls.wallConfigList = []

        # Convert the byte values to arrays and update the wall_config_list
        for row in wall_byte_config_list:
            chamber_num = row[0]
            byte_value = row[1]

            # Convert the byte_value back to an array of wall numbers
            wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

            cls.wallConfigList.append([chamber_num, wall_numbers])

    # Get a list with byte values for each chamber corespoinding to the wall configuration
    @classmethod
    def makeWall2ByteList(cls):
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

    # Use makeWall2ByteList to convert the wall_config_list to a list of byte values where each byte value corresponds entry corresponds to a chamber
    @classmethod
    def getWall2ByteList(cls):
        wall_byte_config_list = cls.makeWall2ByteList()

        # Update U_arr with corresponding chamber and wall byte
        wall_arr = [0] * len(cls.wallConfigList)
        for cw in wall_byte_config_list:
            wall_arr[cw[0]] = cw[1]

        return wall_arr

    # Sort list by chamber number and wall numbers
    @classmethod
    def sortEntries(cls):
        # Sort the rows by the entries in the based on the first chamber number
        cls.wallConfigList.sort(key=lambda row: row[0])

        # Sort the arrays in the second column
        for row in cls.wallConfigList:
            row[1].sort()

    # Used to access the wall_config_list entries
    @classmethod
    def __iter__(cls):
        return iter(cls.wallConfigList)

    # Used to print the wall_config_list
    @classmethod
    def __str__(cls):
        return str(cls.wallConfigList)

# ENUM: Enum for ethercat python to arduino message type ID
class MessageType(Enum):
    MSG_NONE = 0
    CONFIRM_DONE = 1
    HANDSHAKE = 2
    MOVE_WALLS = 3
    START_SESSION = 4
    END_SESSION = 5
    ERROR = 6

# ENUM: Enum for tracking errors
class ErrorType(Enum):
    ERROR_NONE = 0
    MESSAGE_ID_DISORDERED = 1
    NO_MESSAGE_TYPE_MATCH = 2
    REGISTER_LEFTOVERS = 3
    MISSING_FOOTER = 4

# CLASS: Ethercat message handler
class Ecat_Handler:

    # CLASS: Ethercat message struct
    class EcatMessageStruct:

        # CLASS: emulates c++ union type for storing ethercat data shareable accross 8 and 16 bit data types
        class RegUnion(ctypes.Union):
            _fields_ = [("ui8", ctypes.c_uint8 * 16),
                        ("ui16", ctypes.c_uint16 * 8),
                        ("ui64", ctypes.c_uint64 * 2)]

        def __init__(self):
            self.msgDt = 10
            self.msgID = 0
            self.msgTp = MessageType.MSG_NONE
            self.errTp = ErrorType.ERROR_NONE
            self.msg_arg_lng = 0
            self.Arg8 = [0] * 10
            self.ArgU = self.RegUnion()
            self.u8i = 0
            self.u16i = 0
            self.isDone = False
            self.RegU = self.RegUnion()

    # Handshake finished flag
    isHandshakeDone = False

    # Create message structs
    sndEM = EcatMessageStruct()  # message to be sent
    rcvEM = EcatMessageStruct()  # message received
    tmpEM = EcatMessageStruct()  # temporary message
    cnfEM = EcatMessageStruct()  # message being confirmed

    @classmethod
    def _resetU(cls, r_EM):
        r_EM.RegU.ui64[0] = 0
        r_EM.RegU.ui64[1] = 0
        r_EM.u8i = 0
        r_EM.u16i = 0

    @classmethod
    def _seti8(cls, r_EM, dat_8):
        # Store data
        r_EM.RegU.ui8[r_EM.u8i] = dat_8
        rospyLogCol('HIGHLIGHT', "\t_storei8: u8i[%d] u16i[%d] dat_8=%d", r_EM.u8i, r_EM.u16i, dat_8)
        # Update union indices
        r_EM.u8i += 1
        r_EM.u16i = r_EM.u8i // 2 if r_EM.u8i % 2 == 0 else r_EM.u8i // 2 + 1

    @classmethod
    def _seti16(cls, r_EM, dat_16):
        # Store data
        r_EM.RegU.ui16[r_EM.u16i] = dat_16
        rospyLogCol('HIGHLIGHT', "\t_storei16: u8i[%d] u16i[%d] dat_16=%d", r_EM.u8i, r_EM.u16i, dat_16)
        # Update union indices
        r_EM.u16i += 1
        r_EM.u8i = r_EM.u16i * 2

    @classmethod
    def _geti8(cls, r_EM):
        # Get data
        dat_8 = r_EM.RegU.ui8[r_EM.u8i]
        # Update union indices and return data
        r_EM.u8i += 1
        r_EM.u16i = r_EM.u8i // 2 if r_EM.u8i % 2 == 0 else r_EM.u8i // 2 + 1
        return dat_8

    @classmethod
    def _geti16(cls, r_EM):
        # Store data
        dat_16 = r_EM.RegU.ui16[r_EM.u16i]
        # Update union indices and return data
        r_EM.u16i += 1
        r_EM.u8i = r_EM.u16i * 2
        return dat_16

    @classmethod
    def _printEcatReg(cls, level, d_type, reg_u, msg_type_val=MessageType.MSG_NONE.value):
       
        # Get message type string if exists otherwise set string to "NA"
        try:
            msg_type_str = MessageType(msg_type_val).name
        except ValueError:
            msg_type_str = "NA"

        # Print heading with type
        rospyLogCol(level, "\tEcat Register")

        # Print message data
        for i in range(8):
            if d_type == 1 or i == 0:
                rospyLogCol(level, "\t\tui16[%d] %d", i, reg_u.ui16[i])
            if d_type == 0:
                rospyLogCol(level, "\t\tui8[%d]  %d %d", i, reg_u.ui8[2 * i], reg_u.ui8[2 * i + 1])


    @classmethod
    def _setupMsgStruct(cls, r_EM, msg_id, msg_type_val=0, msg_type_enum=MessageType.MSG_NONE):
       
        # Handle different msg type input
        if msg_type_enum != MessageType.MSG_NONE:
            msg_type_val = msg_type_enum.value

        # Check for valid message type
        is_found = msg_type_val in MessageType.__members__.values()
        is_err = not is_found

        # Set type to none if not found
        if is_err:
            msg_type_val = MessageType.MSG_NONE.value

        # Store message id
        r_EM.msgID = msg_id

        # Store message type enum
        r_EM.msgTp = MessageType(msg_type_val)

        return is_err

    @classmethod
    def _checkErr(cls, r_EM, err_tp, is_err):

        # Handle error
        if is_err:
            # Store error type
            r_EM.errTp = err_tp
            if r_EM.errTp != err_tp:  # only run once
                # Set error type
                r_EM.errTp = err_tp
                rospyLogCol(
                    'ERROR', "!!ERROR: Ecat: %s: id=%d type=%s[%d]!!", r_EM.errTp.name, r_EM.msgID, r_EM.msgTp.name, r_EM.msgTp.value)
                cls._printEcatReg(0, r_EM.RegU, level='ERROR')  # TEMP

        else:
            if r_EM.errTp == err_tp:
                r_EM.errTp = ErrorType.ERROR_NONE  # unset error type

    @classmethod
    def formatEthercatMessage(cls, msg_type_enum, msg_arg_data=None, msg_arg_lng=255):
        # Reset union variables
        cls._resetU(cls.sndEM)

        # Update message id: iterate id and roll over to 1 if max 16 bit value is reached
        msg_id = cls.sndEM.msgID + 1 if cls.sndEM.msgID < 65535 else 1

        # Store message type info
        cls._setupMsgStruct(cls.sndEM, msg_id, msg_type_enum=msg_type_enum)

        # Store message id
        cls._seti16(cls.sndEM, cls.sndEM.msgID)  # message id

        # Store message type value
        cls._seti8(cls.sndEM, cls.sndEM.msgTp.value)

        # 	------------- STORE ARGUMENTS -------------

        # CONFIRM_DONE
        if cls.sndEM.msgTp == MessageType.CONFIRM_DONE:
            cls._seti8(cls.sndEM, 4)  # message argument length
            cls._seti16(cls.sndEM, cls.rcvEM.msgID)  # received message id
            # received message type value
            cls._seti8(cls.sndEM, cls.rcvEM.msgTp.value)

        # HANDSHAKE
        elif cls.sndEM.msgTp == MessageType.HANDSHAKE:
            cls._seti8(cls.sndEM, 0)  # message argument length

        # MOVE_WALLS
        elif cls.sndEM.msgTp == MessageType.MOVE_WALLS:
            cls._seti8(cls.sndEM, msg_arg_lng)  # store message argument length
            for i in range(msg_arg_lng):
                cls._seti8(cls.sndEM, msg_arg_data[i])

        # OTHER
        else:
            # Store message argument length if provided
            cls._seti8(cls.sndEM, msg_arg_lng)

            # Store message arguments if provided
            if msg_arg_data is not None:
                for i in range(msg_arg_lng):
                    cls._seti8(cls.sndEM, msg_arg_data[i])

        # 	------------- FINISH SETUP AND WRITE -------------

        # Store footer
        cls._seti8(cls.sndEM, 254)
        cls._seti8(cls.sndEM, 254)

        # Set flag
        cls.sndEM.isDone = False

        # Print message
        rospyLogCol('INFO', "SENT Ecat Message: id=%d type=%s",
                    cls.sndEM.msgID, cls.sndEM.msgTp.name)
        cls._printEcatReg('INFO', 0, cls.sndEM.RegU)  # TEMP

        # Store and return ethercat message
        reg_arr = [0] * 8
        for i in range(8):
            reg_arr[i] = cls.sndEM.RegU.ui16[i]
        return reg_arr

    @classmethod
    def parseEthercatMessage(cls, reg_dat):
        is_err = False  # error flag

        # Copy data into union
        cls._resetU(cls.tmpEM)  # reset union variables
        for i in range(8):
            cls.tmpEM.RegU.ui16[i] = reg_dat[i]

        # Skip ethercat setup junk (255)
        if cls.tmpEM.RegU.ui8[0] == 255 or cls.tmpEM.RegU.ui8[1] == 255:
            return 0
        
        # Get message id and message type value
        msg_id = cls._geti16(cls.tmpEM)
        msg_type_val = cls._geti8(cls.tmpEM)

        # Skip redundant messages
        if cls.tmpEM.msgID == cls.rcvEM.msgID:
            return 0
        
        # Setup message struct and check for valid message type
        is_err = cls._setupMsgStruct(cls.tmpEM, msg_id, msg_type_val=msg_type_val)

        # Run check error for valid message type
        cls._checkErr(cls.tmpEM, ErrorType.NO_MESSAGE_TYPE_MATCH, is_err)
        if is_err:
            return 2  # return error flag

        # Check if message is preceding handshake
        is_err = not cls.isHandshakeDone and cls.tmpEM.msgTp != MessageType.HANDSHAKE
        cls._checkErr(cls.tmpEM, ErrorType.REGISTER_LEFTOVERS, is_err)
        if is_err:
            return 2  # return error flag

        # Check for skipped or out of sequence messages
        is_err = cls.tmpEM.msgID - cls.rcvEM.msgID != 1
        cls._checkErr(cls.tmpEM, ErrorType.MESSAGE_ID_DISORDERED, is_err)
        if is_err:
            return 2  # return error flag

        # Get argument length
        cls.tmpEM.msg_arg_lng = cls._geti8(cls.tmpEM)

        # Parse 8 bit message arguments
        if cls.tmpEM.msg_arg_lng > 0:
            for i in range(cls.tmpEM.msg_arg_lng):
                cls.tmpEM.ArgU.ui8[i] = cls._geti8(cls.tmpEM)

        # Check for footer
        is_err = cls._geti8(cls.tmpEM) != 254 or cls._geti8(cls.tmpEM) != 254
        cls._checkErr(cls.tmpEM, ErrorType.MISSING_FOOTER, is_err)
        if is_err:
            return 2  # return error flag

        # Set flag
        cls.tmpEM.isDone = False

        # Copy over data
        cls.rcvEM = cls.tmpEM

        # 	------------- PROCESS ARGUMENTS -------------

        # CONFIRM_DONE
        if cls.rcvEM.msgTp == MessageType.CONFIRM_DONE:
            rospyLogCol('INFO', "\tCONFIRM_DONE")

            # Get confirmation message number
            cls.cnfEM.msgID = cls.rcvEM.ArgU.ui16[0]

            # Get message number
            cls.cnfEM.msgTp = MessageType(cls.rcvEM.ArgU.ui8[2])

            # Print confirmation message
            rospyLogCol('INFO', "\t\tConfirmed: type=%s id=%d",
                        cls.cnfEM.msgTp, cls.cnfEM.msgID)

        # ERROR
        if cls.rcvEM.msgTp == MessageType.ERROR:
            pass

        # Print message
        rospyLogCol('INFO', "RECEIVED Ecat Message: id=%d type=%s",
                    cls.rcvEM.msgID, cls.rcvEM.msgTp.name)
        cls._printEcatReg('INFO', 0, cls.rcvEM.RegU)  # TEMP

        # Send confirm done message
        cls.formatEthercatMessage(MessageType.CONFIRM_DONE)

        return 1  # return data received flag

# CLASS: Interface plugin
class Interface(Plugin):

    # Define signals
    signal_Esmacat_read_maze_ard0_ease = Signal()

    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

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

        # Calculate chamber width and wall line width and offset
        maze_view_size = self._widget.plotMazeView.width()
        chamber_width = self._widget.plotMazeView.width()*0.9/NUM_ROWS_COLS
        wall_width = chamber_width*0.1

        # Create Maze_Plot.Maze and populate walls according to WALL_MAP
        self.maze = Maze_Plot.Maze(num_rows=NUM_ROWS_COLS,
                                  num_cols=NUM_ROWS_COLS,
                                  chamber_width=chamber_width,
                                  wall_width=wall_width)

        # Add chambers and disable walls not connected
        for k, c in enumerate(self.maze.chambers):
            self.scene.addItem(c)
            for j, w in enumerate(c.walls):
                if j not in WALL_MAP[k]:
                    w.setEnabled(False)
                    w.setVisible(False)

        # Add walls - this is a new loop so that they are drawn above the chambers
        for c in self.maze.chambers:
            for w in c.walls:
                self.scene.addItem(w)

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

        # QT timer for sending initial handshake message after a delay
        self.timer_sendHandshake = QTimer()
        self.timer_sendHandshake.timeout.connect(
            self.timer_callback_sendHandshake_once)
        self.timer_sendHandshake.setSingleShot(True)  # Run only once
        self.timer_sendHandshake.start(1000)  # (ms)

        # QT timer for checking initial handshake message after a delay
        self.timer_checkHandshake = QTimer()
        self.timer_checkHandshake.timeout.connect(
            self.timer_callback_checkHandshake_once)
        self.timer_checkHandshake.setSingleShot(True)  # Run only once

        # ROS publisher
        # wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)

        # Esmacat write maze ard0 ease publisher
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)  

        # ROS subscriber
        # Esmacat arduino maze ard0 ease
        rospy.Subscriber(
            'Esmacat_read_maze_ard0_ease', ease_registers, self.ros_callback_Esmacat_read_maze_ard0_ease, tcp_nodelay=True)

        # Signal callback setup
        self.signal_Esmacat_read_maze_ard0_ease.connect(
            self.sig_callback_Esmacat_read_maze_ard0_ease)

    """ CALLBACKS: ROS, QT and Signal """

    def ros_callback_Esmacat_read_maze_ard0_ease(self, msg):
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

        # Convert 16 bit signed ints to 16 bit unsigned ints array
        ui16_arr = [0]*8
        for i in range(8):
            ui16_arr[i] = ctypes.c_uint16(si16_arr[i]).value

        # Parse the message and check if its new
        if Ecat_Handler.parseEthercatMessage(ui16_arr) != 1:
            return

        # Handle confirmation message
        if Ecat_Handler.rcvEM.msgTp == MessageType.CONFIRMATION:
            # Check if confirmation message is handshake
            if Ecat_Handler.cnfEM.msgTp == MessageType.HANDSHAKE:

                # Stop handshake check timer
                self.timer_checkHandshake.stop()

                # Set the handshake flag
                Ecat_Handler.isHandshakeDone = True

                # Send START_SESSION message
                self.maze_ard0_pub.publish(
                    *Ecat_Handler.formatEthercatMessage(MessageType.START_SESSION)
                )
                

        # Emit signal to update UI as UI should not be updated from a non-main thread
        # TEMP self.signal_Esmacat_read_maze_ard0_ease.emit()

    def sig_callback_Esmacat_read_maze_ard0_ease(self):
        pass

    def timer_callback_sendHandshake_once(self):
        # Send HANDSHAKE message to arduino
        self.maze_ard0_pub.publish(
                *Ecat_Handler.formatEthercatMessage(MessageType.HANDSHAKE)
        )

        # Start timer to check for HANDSHAKE message confirm recieved
        self.timer_checkHandshake.start(500)

    def timer_callback_checkHandshake_once(self):
        # Check for HANDSHAKE message confirm recieved flag
        if Ecat_Handler.isHandshakeDone == False:

            # Give up is more that 3 messages have been sent
            if Ecat_Handler.sndEM.msgID > 3:
                rospyLogCol(
                    'ERROR', "!!ERROR: Handshake failed: msg_id=%d!!", Ecat_Handler.sndEM.msgID)
                return

            # Send HANDSHAKE message to arduino again
            self.maze_ard0_pub.publish(
                *Ecat_Handler.formatEthercatMessage(MessageType.HANDSHAKE)
            )

            # Restart check timer
            self.timer_checkHandshake.start(500)
        else:
            # Send START message to arduino
            self.maze_ard0_pub.publish(
                *Ecat_Handler.formatEthercatMessage(MessageType.START_SESSION)
            )

    def timer_callback_updateUI_loop(self):
        # Update graphics
        self.scene.update()
        self._widget.plotMazeView.update()

    def qt_callback_fileBrowseBtn_clicked(self):
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
        # Get the index of the clicked item and set it as the current file index
        self.current_file_index = self._widget.fileListWidget.currentRow()

        # Update file index and load csv
        self.loadFromCSV(0)

    def qt_callback_fileNextBtn_clicked(self):
        # Update file index and load csv
        self.loadFromCSV(1)

    def qt_callback_filePreviousBtn_clicked(self):
        # Update file index and load csv
        self.loadFromCSV(-1)

    def qt_callback_plotClearBtn_clicked(self):
        Wall_Config.Reset()  # reset all values in list
        self.maze.updateWalls()  # update walls

    def qt_callback_plotSaveBtn_clicked(self):
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
        # Sort entries
        Wall_Config.sortEntries()

        # Send the wall byte array to the arduino
        self.maze_ard0_pub.publish(
            *Ecat_Handler.formatEthercatMessage(
            MessageType.MOVE_WALLS, 9, Wall_Config.getWall2ByteList())
        )

    def qt_callback_sysQuiteBtn_clicked(self):
        # Call function to shut down the ROS session
        self.endRosSession()
        # End the application
        QApplication.quit()

    ''' FUNCTIONS: CSV File Handling '''

    def getPathConfigDir(self, file_name=None):
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
        try:
            with open(file_name, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in wall_config_list:
                    csv_writer.writerow(row)
            rospyLogCol('INFO', "Data saved to:", file_name)
        except Exception as e:
            rospyLogCol('ERROR', "Error saving data to CSV:", str(e))

    def loadFromCSV(self, list_increment):
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
        self.maze.updateWalls()

    ''' FUNCTIONS: System Operations '''

    def terminate_ros_node(self, s):
        list_cmd = subprocess.Popen(
            "rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.decode().split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def terminate_process_and_children(self, p):
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

        # Send END_SESSION message to arduino
        self.maze_ard0_pub.publish(
            *Ecat_Handler.formatEthercatMessage(MessageType.END_SESSION)
        )

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

    # NOT WORKING!!
    def closeEvent(self, event):
        rospyLogCol('INFO', "Closing window...")
        # Call function to shut down the ROS session
        self.endRosSession()
        event.accept()  # let the window close

# FUNCTION: Log to ROS in color
def rospyLogCol(level, msg, *args):
    # Exit if DB_VERBOSE is false
    if not DB_VERBOSE:
        return

    # Define colors relative to level string argument
    if level == 'ERROR':
        color = Fore.RED
    elif level == 'WARNING':
        color = Fore.YELLOW
    elif level == 'INFO':
        color = Fore.BLUE
    elif level == 'HIGHLIGHT':
        color = Fore.GREEN
    else:
        color = Fore.BLACK

    # Format and log message
    colored_message = f"{color}{msg}{Style.RESET_ALL}"
    formatted_message = colored_message % args
    rospy.loginfo(formatted_message)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())
