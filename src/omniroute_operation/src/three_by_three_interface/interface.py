#!/usr/bin/env python
import os
import rospy
from omniroute_esmacat_ros.msg import *

from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin
import numpy as np
from scipy.io import loadmat
import math
from colorama import Fore, Style
import ctypes
from enum import Enum

from std_msgs.msg import *
from omniroute_operation.msg import *

"""
The outgoing register is structured arr[8] of 16 bit integers
with each 16 bit value seperated into bytes 
[0]: message number
    i16 [0-65535] 
[1]: message info
    b0 message type [0-255] [see: MsgTypeID]
    b1 arg length [0-255] [number of message args in bytes]           
[2:5] wall state bytes
    b0 = wall x byte
    b1 = wall x+1 byte  
[x-8]: footer  
    b1=254
    b0=254                            

@return: None
"""

# GLOBAL VARS
norm = np.linalg.norm
wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)
maze_ard0_pub = rospy.Publisher('/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)

NUM_ROWS_COLS = 3
WALL_MAP = {
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

# Enum for message type ID
class MsgTypeID(Enum):
    INITIALIZE = 128
    MOVE_WALLS_UP = 1
    MOVE_WALLS_DOWN = 2

# Initialize colorama
Fore.RED, Fore.GREEN, Fore.BLUE, Fore.YELLOW  # Set the desired colors

# Log to ROS in color
def rospy_log_info(color, message, *args):
    colored_message = f"{color}{message}{Style.RESET_ALL}"
    formatted_message = colored_message % args
    rospy.loginfo(formatted_message)

def in_current_folder(file_name: str):
    # Get the absolute path of the current script file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Create the path to the "config" directory four levels up
    return os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', 'config', 'paths'))
    # return os.path.join(os.path.dirname(os.path.realpath(__file__)), file_name)

def center_text(text_item, center_x, center_y):
    # Set the text item's position relative to its bounding rectangle
    text_item.setTextWidth(0)  # Allow the text item to resize its width automatically
    text_item.setHtml('<div style="text-align: center; vertical-align: middle;">{}</div>'.format(text_item.toPlainText()))

    # Get the bounding rectangle of the text item
    text_rect = text_item.boundingRect()

    # Center the text both vertically and horizontally over the given coordinates
    x_pos = center_x - text_rect.width() / 2
    y_pos = center_y - text_rect.height() / 2
    text_item.setPos(x_pos, y_pos)

# Create a byte with bits set to 1 based on wall_up_arr
def set_wall_byte(wall_arr):
    byte_value = 0  # Initialize the byte value

    # Iterate over the array of values
    for index in wall_arr:
        if 0 <= index <= 7:
            # Set the corresponding bit to 1 using bitwise OR
            byte_value |= (1 << index)

    return byte_value

# Make msg for Ethercat registry
def make_reg_msg(msg_type_id, msg_lng, cw_list=None):
    # Initialize the function attribute if not already present
    if not hasattr(make_reg_msg, "msg_num_id"):
        make_reg_msg.msg_num_id = 0

    # Itterate message count
    make_reg_msg.msg_num_id = make_reg_msg.msg_num_id + \
        1 if make_reg_msg.msg_num_id < 65535 else 1

    # Create a list 'reg' with 8 16-bit Union elements
    U_arr = [Union() for _ in range(8)]
    u_ind_r = 0

    # Set message num and type id
    U_arr[u_ind_r].i16 = make_reg_msg.msg_num_id
    u_ind_r += 1
    U_arr[u_ind_r].b[0] = msg_type_id.value
    U_arr[u_ind_r].b[1] = msg_lng

    # Update walls to move up
    if (msg_type_id == MsgTypeID.MOVE_WALLS_UP or msg_type_id == MsgTypeID.MOVE_WALLS_DOWN) and cw_list is not None:
        # Update U_arr with corresponding chamber and wall byte
        for cw in cw_list:
            chamber = cw[0]
            u_ind_r = 2 + (chamber // 2)
            wall_byte = set_wall_byte(cw[1])
            u_ind_c = 0 if chamber % 2 == 0 else 1
            U_arr[u_ind_r].b[u_ind_c] = wall_byte
            #rospy_log_info(Fore.YELLOW,"chamber=%d u_ind_r=%d u_ind_c=%d", chamber, u_ind_r, u_ind_c)

    # Set footer
    u_ind_r = 2 + math.ceil(msg_lng/2)
    U_arr[u_ind_r].b[0] = 254
    U_arr[u_ind_r].b[1] = 254

    # TEMP
    #rospy_log_info(Fore.RED,"u_ind_r=%d msg_lng=%d msg_lng/2=%d", u_ind_r, msg_lng, math.ceil(msg_lng/2))

    # Store and return 16-bit values cast as signed for use with ease_registers
    reg_arr = [ctypes.c_int16(U.i16).value for U in U_arr]

    # Print reg message
    for index, U in enumerate(U_arr):
        rospy_log_info(Fore.BLUE, "%d %d", U.b[0], U.b[1])

    # # Print the cw_list
    # if cw_list is not None:
    #     rospy_log_info(Fore.BLUE, "Chamber and wall configuration list:")
    #     for chamber, walls in cw_list:
    #         rospy_log_info(
    #             Fore.BLUE, "Chamber %d: Walls %s", chamber, walls)

    return reg_arr

# Union class using ctype union for storing ethercat data shareable accross data types
class Union:
    def __init__(self):
        self.b = bytearray([0, 0])  # 2 bytes initialized with zeros

    @property
    def i16(self):
        return struct.unpack("<H", self.b)[0]

    @i16.setter
    def i16(self, value):
        packed_value = struct.pack("<H", value)
        self.b[0] = packed_value[0]
        self.b[1] = packed_value[1]

# The shared ConfigHolder class to store the wall_config_list
class ConfigHolder:
    def __init__(self):
        self.wall_config_list = []

    def add_wall(self, chamber_num, wall_num):
        for item in self.wall_config_list:
            if item[0] == chamber_num:
                item[1].append(wall_num)
                return
        self.wall_config_list.append([chamber_num, [wall_num]])

    def remove_wall(self, chamber_num, wall_num):
        for item in self.wall_config_list:
            if item[0] == chamber_num:
                item[1].remove(wall_num)
                if not item[1]:  # If the second column is empty, remove the entire row
                    self.wall_config_list.remove(item)
                return

    def sort_entries(self):
        # Sort the rows by the entries in the based on the first chamber number
        self.wall_config_list.sort(key=lambda row: row[0])

        # Sort the arrays in the second column
        for row in self.wall_config_list:
            row[1].sort()

    def reset(self):
        self.wall_config_list = []

    def get_len(self):
        return len(self.wall_config_list)
    
    def __iter__(self):
        return iter(self.wall_config_list)

    def __str__(self):
        return str(self.wall_config_list)


# Create a shared instance of ConfigHolder
CW_LIST = ConfigHolder()

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

        center_text(self.label, label_pos[0], label_pos[1])

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            rospy.loginfo("Chamber %d wall %d clicked!" % (self.chamber_num, self.wall_num))
            self.setState(not self.state)
            wall_clicked_pub.publish(self.chamber_num, self.wall_num, self.state)
            if self.state: # add list entry
                CW_LIST.add_wall(self.chamber_num, self.wall_num)
            else: # remove list entry
                CW_LIST.remove_wall(self.chamber_num, self.wall_num)
            print(CW_LIST)

    def setState(self, state: bool):
        if state:
            self.line.setPen(self.upPen)
        else:
            self.line.setPen(self.downPen)

        self.state = state

class Chamber(QGraphicsItemGroup):
    def __init__(self, center_x, center_y, chamber_width, chamber_num, wall_width, parent=None):
        super().__init__(parent)
        self.center_x = center_x
        self.center_y = center_y
        self.chamber_width = chamber_width
        self.chamber_num = chamber_num

        # Plot backround chamber octogons
        octagon_vertices = self.get_octagon_vertices(center_x, center_y, chamber_width/2, math.radians(22.5))
        octagon_points = [QPointF(i[0], i[1]) for i in octagon_vertices]
        self.octagon = QGraphicsPolygonItem(QPolygonF(octagon_points))
        self.octagon.setBrush(QBrush(QColor(200, 200, 200)))
        self.addToGroup(self.octagon)

        # Plot cahamber numbers
        self.label = QGraphicsTextItem(str(chamber_num))
        font_size = wall_width*2
        self.label.setFont(QFont("Arial", font_size, QFont.Bold))
        self.addToGroup(self.label)

        # Call the center_text method to center the text over the chamber's center
        center_text(self.label, center_x, center_y)

        wall_angular_offset = 2*math.pi/32  # This decides the angular width of the wall
        wall_vertices_0 = self.get_octagon_vertices(
            center_x, center_y, chamber_width/2, -math.pi/8+wall_angular_offset)
        wall_vertices_1 = self.get_octagon_vertices(
            center_x, center_y, chamber_width/2, -math.pi/8-wall_angular_offset)
        wall_label_pos = self.get_octagon_vertices(
            center_x, center_y, chamber_width/3, 0)

        self.walls = [Wall(p0=wall_vertices_0[k], 
                           p1=wall_vertices_1[k+1], 
                           chamber_num=chamber_num,
                           wall_num=k, 
                           wall_width=wall_width, 
                           label_pos=wall_label_pos[k])
                      for k in range(8)]

    def get_octagon_vertices(self, x, y, w, offset):
        vertices_list = [(round(x + w*math.cos(k)), round(y+w*math.sin(k)))
                         for k in np.linspace(math.pi, 3*math.pi, 9) + offset]
        return vertices_list

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            rospy.loginfo("Chamber %d clicked!" % self.chamber_num)



class Maze:
    def __init__(self, num_rows, num_cols, chamber_width, wall_width):

        self.num_rows = num_rows
        self.num_cols = num_cols
        self.chamber_width = chamber_width

        maze_width = self.chamber_width * self.num_cols
        maze_height = self.chamber_width * self.num_rows
        half_width = chamber_width/2
        x_pos = np.linspace(half_width, int(maze_width - half_width),  num_cols)
        y_pos = np.linspace(half_width, int(maze_height - half_width),  num_rows)

        self.chambers = []
        k = 0
        for y in y_pos:
            for x in x_pos:
                self.chambers.append(
                    Chamber(center_x=x, center_y=y, chamber_width=chamber_width, wall_width=wall_width, chamber_num=k))
                k = k+1

    def reset_walls(self):
        for chamber in self.chambers:
            for wall in chamber.walls:
                wall.setState(False)


class Interface(Plugin):
    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

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

        # Load maze config
        # maze_config = loadmat(in_current_folder('maze_config.mat'))['involved_cd']
        # separating first (indicates number of polygons) and second column (indicates up and down walls).
        # self.cell_number = [c[0] for c in maze_config]
        # self.wall_binary_code = [bin(c[1])[2:].zfill(8)[::-1] for c in maze_config]

        # Create QWidget
        self._widget = QWidget()
        # Extend the widget with all attributes and children from UI file
        loadUi(os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'interface.ui'), self._widget)

        rospy.loginfo('Interface started')

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
        self._widget.mazeView.setViewportUpdateMode(
            QGraphicsView.FullViewportUpdate)
        self.scene = QGraphicsScene()
        self._widget.mazeView.setScene(self.scene)

        # Set the fixed size of the main window based on the dimensions from the UI file
        main_window_width = self._widget.geometry().width()
        main_window_height = self._widget.geometry().height()
        self._widget.setFixedSize(main_window_width, main_window_height)

        # Set the size hint of the main window to match the size of the _widget
        self._widget.window().setMinimumSize(main_window_width, main_window_height)
        self._widget.window().setMaximumSize(main_window_width, main_window_height)

        # Set the background color of the scene to white
        self._widget.mazeView.setBackgroundBrush(QColor(255, 255, 255))
        self._widget.mazeView.setViewport(QtOpenGL.QGLWidget())

        # CSV browser
        self._widget.pathBrowseBtn.clicked.connect(
            self._handle_pathBrowseBtn_clicked)
        self._widget.pathPreviousBtn.clicked.connect(
            self._handle_pathPreviousBtn_clicked)
        self._widget.pathNextBtn.clicked.connect(
            self._handle_pathNextBtn_clicked)
        self._widget.pathDirEdit.setText(in_current_folder('.'))

        # Other buttons
        self._widget.plotClearBtn.clicked.connect(
            self._handle_plotClearBtn_clicked)
        self._widget.plotSaveBtn.clicked.connect(
            self._handle_plotSaveBtn_clicked)
        self._widget.plotSendBtn.clicked.connect(
            self._handle_plotSendBtn_clicked)

         # Calculate chamber width and wall line width and offset
        maze_view_size = self._widget.mazeView.width()
        chamber_width = self._widget.mazeView.width()*0.9/NUM_ROWS_COLS
        wall_width = chamber_width*0.1

        # Create Maze and populate walls according to WALL_MAP
        self.maze = Maze(num_rows=NUM_ROWS_COLS, 
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

        # Start timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateScene)
        self.timer.start(20)

        # Send initialization message
        rospy_log_info(Fore.GREEN, "INITIALIZE")
        reg_arr = make_reg_msg(MsgTypeID.INITIALIZE, 0)
        maze_ard0_pub.publish(*reg_arr)
        

    def updateScene(self):
        self.scene.update()
        self._widget.mazeView.update()

    def _handle_pathBrowseBtn_clicked(self):
        # Change this to the file type you want to allow
        filter = "CSV Files (*.csv)"
        files, _ = QFileDialog.getOpenFileNames(
            None, "Select files to add", in_current_folder('.'), filter)

        if files:
            # Clear the list widget to remove any previous selections
            self._widget.pathListWidget.clear()

            # Extract the file names from the full paths and store them in self.files
            self.files = [os.path.basename(file) for file in files]

            # Add the selected file names to the list widget
            self._widget.pathListWidget.addItems(self.files)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(self.files) > 1:
                self._widget.pathNextBtn.setEnabled(True)
                self._widget.pathPreviousBtn.setEnabled(True)
            else:
                self._widget.pathNextBtn.setEnabled(False)
                self._widget.pathPreviousBtn.setEnabled(False)

            # Save the list of selected files as an attribute of the class
            self.current_file_index = 0

            # Connect the "Next" and "Previous" buttons to their respective callback functions
            self._widget.pathPreviousBtn.clicked.connect(
                self._handle_pathPreviousBtn_clicked)
            self._widget.pathNextBtn.clicked.connect(
                self._handle_pathNextBtn_clicked)
            
    def _handle_pathNextBtn_clicked(self):
        # Decrement the current file index
        self.current_file_index -= 1

        # If we've reached the beginning of the list, loop back to the end
        if self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1

        # Set the current file in the list widget
        self._widget.pathListWidget.setCurrentRow(self.current_file_index)
    
    def _handle_pathPreviousBtn_clicked(self):
        # Increment the current file index
        self.current_file_index += 1

        # If we've reached the end of the list, loop back to the beginning
        if self.current_file_index >= len(self.files):
            self.current_file_index = 0

        # Set the current file in the list widget
        self._widget.pathListWidget.setCurrentRow(self.current_file_index)

    def _handle_plotClearBtn_clicked(self):
        self.maze.reset_walls()
        CW_LIST.reset()

    def _handle_plotSaveBtn_clicked(self):
        self.maze.reset_walls()

    def _handle_plotSendBtn_clicked(self):
        # Sort entries
        CW_LIST.sort_entries()
        print(CW_LIST)
        reg_arr = make_reg_msg(MsgTypeID.MOVE_WALLS_UP, 9, CW_LIST)
        maze_ard0_pub.publish(*reg_arr)  # Publish list to topic