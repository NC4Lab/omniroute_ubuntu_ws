#!/usr/bin/env python
import os
import rospy
from omniroute_esmacat_ros.msg import *

from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL

from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsItemGroup, QGraphicsLineItem, QGraphicsTextItem
from PyQt5.QtCore import Qt, QRectF
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

#ENUM: Enum for ethercat python to arduino message type ID
class P2A_Type_ID(Enum):
    P2A_NONE = 0
    CLEAR_REG = 127
    START_SESSION = 128
    END_SESSION = 129
    MOVE_WALLS = 1

#ENUM: Enum for ethercat arduino to python message type ID
class A2P_Type_ID(Enum):
    A2P_NONE = 0
    CONFIRM_RECEIVED = 1
    ERROR = 254

#ENUM: Enum for tracking errors
class Error_Type(Enum):
    ERROR_NONE = 0
    MESSAGE_ID_DISORDERED = 1
    MISSING_FOOTER = 2

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

            Maze_Plot.center_text(self.label, label_pos[0], label_pos[1])

        def mousePressEvent(self, event):
            if event.button() == Qt.LeftButton:
                #rospy_log_col('INFO', "\tchamber %d wall %d clicked in UI" % (self.chamber_num, self.wall_num))
                self.setState(not self.state)
                #wall_clicked_pub.publish(self.chamber_num, self.wall_num, self.state)
                if self.state: # add list entry
                    Maze_Plot.CW_LIST.add_wall(self.chamber_num, self.wall_num)
                else: # remove list entry
                    Maze_Plot.CW_LIST.remove_wall(self.chamber_num, self.wall_num)

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

            # Center the text over the chamber's center
            Maze_Plot.center_text(self.label, center_x, center_y)

            wall_angular_offset = 2*math.pi/32  # This decides the angular width of the wall
            wall_vertices_0 = self.get_octagon_vertices(
                center_x, center_y, chamber_width/2, -math.pi/8+wall_angular_offset)
            wall_vertices_1 = self.get_octagon_vertices(
                center_x, center_y, chamber_width/2, -math.pi/8-wall_angular_offset)
            wall_label_pos = self.get_octagon_vertices(
                center_x, center_y, chamber_width/3, 0)

            self.walls = [Maze_Plot.Wall(p0=wall_vertices_0[k], 
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
                pass
                #rospy_log_col('INFO', "\tchamber %d clicked in UI" % self.chamber_num)

    # CLASS: Maze
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
                        Maze_Plot.Chamber(center_x=x, center_y=y, chamber_width=chamber_width, wall_width=wall_width, chamber_num=k))
                    k = k+1

        def update_walls(self):
            cw_list = Maze_Plot.CW_LIST.wall_config_list 

            for chamber in self.chambers:
                for wall in chamber.walls:
                    # Check if there is an entry for the current chamber and wall in Maze_Plot.CW_LIST
                    chamber_num = chamber.chamber_num
                    wall_num = wall.wall_num
                    entry_found = any(chamber_num == entry[0] and wall_num in entry[1] for entry in cw_list)

                    # Set the wall state based on whether the entry is found or not
                    wall.setState(entry_found)

    # CLASS: The shared CW_LIST class to store the wall configuration 
    class CW_LIST:
        wall_config_list = []

        @classmethod
        def add_wall(self, chamber_num, wall_num):
            for item in self.wall_config_list:
                if item[0] == chamber_num:
                    item[1].append(wall_num)
                    return
            self.wall_config_list.append([chamber_num, [wall_num]])

        @classmethod
        def remove_wall(self, chamber_num, wall_num):
            for item in self.wall_config_list:
                if item[0] == chamber_num:
                    item[1].remove(wall_num)
                    if not item[1]:  # If the second column is empty, remove the entire row
                        self.wall_config_list.remove(item)
                    return

        @classmethod
        def get_byte_list(self):
            wall_byte_config_list = []
            for row in self.wall_config_list:
                chamber_num = row[0]
                wall_numbers = row[1]
                byte_value = self._make_wall_byte(wall_numbers)
                wall_byte_config_list.append([chamber_num, byte_value])
            return wall_byte_config_list
        
        @classmethod
        def convert_byte_list(self, wall_byte_config_list):
            # Clear the existing wall_config_list
            self.wall_config_list = []

            # Convert the byte values to arrays and update the wall_config_list
            for row in wall_byte_config_list:
                chamber_num = row[0]
                byte_value = row[1]

                # Convert the byte_value back to an array of wall numbers
                wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

                self.wall_config_list.append([chamber_num, wall_numbers])

        @classmethod
        def sort_entries(self):
            # Sort the rows by the entries in the based on the first chamber number
            self.wall_config_list.sort(key=lambda row: row[0])

            # Sort the arrays in the second column
            for row in self.wall_config_list:
                row[1].sort()

        @classmethod
        def reset(self):
            self.wall_config_list = []

        @classmethod
        def get_len(self):
            return len(self.wall_config_list)
        
        @classmethod
        def _make_wall_byte(self, wall_arr):
            byte_value = 0  # Initialize the byte value

            # Iterate over the array of values
            for index in wall_arr:
                if 0 <= index <= 7:
                    # Set the corresponding bit to 1 using bitwise OR
                    byte_value |= (1 << index)

            return byte_value

        @classmethod
        def __iter__(self):
            return iter(self.wall_config_list)

        @classmethod
        def __str__(self):
            return str(self.wall_config_list)

    # FUNCTION: Center plotted text 
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

# CLASS: Interface plugin
class Interface(Plugin):

    # Define signals
    signal_Esmacat_read_maze_ard0_ease = Signal()

    # Define class variables
    p2aEtherMsgID = 0 # initialize python to arduino message number
    a2pEtherMsgID = 0 # initialize arduino to python message number

    # Create enum instances
    a2pEtherMsgType = A2P_Type_ID.A2P_NONE
    p2aEtherMsgType = P2A_Type_ID.P2A_NONE
    errorType = Error_Type.ERROR_NONE

    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

        # Give QObjects reasonable names
        self.setObjectName('Interface')
        rospy_log_col('INFO', "Running Interface setup")

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
        self.current_file_index = 0 # set to zero
        self._widget.fileDirEdit.setText(self.get_path_config_dir()) # set to default path

        # Initialize ardListWidget with arduino names.
        # Add 8 arduinos to the list labeled Arduino 0-9
        for i in range(8):
            self._widget.ardListWidget.addItem("Arduino " + str(i))
        # Hide the blue selection bar
        self._widget.ardListWidget.setStyleSheet("QListWidget::item { border-bottom: 1px solid black; }")
        # Change the text color for all items in the list to light gray
        self._widget.ardListWidget.setStyleSheet("QListWidget { color: lightgray; }")

        # QT UI object callback setup
        self._widget.fileListWidget.itemClicked.connect(self.qt_callback_fileListWidget_clicked)
        self._widget.fileBrowseBtn.clicked.connect(self.qt_callback_fileBrowseBtn_clicked)
        self._widget.filePreviousBtn.clicked.connect(self.qt_callback_filePreviousBtn_clicked)
        self._widget.fileNextBtn.clicked.connect(self.qt_callback_fileNextBtn_clicked)
        self._widget.plotClearBtn.clicked.connect(self.qt_callback_plotClearBtn_clicked)
        self._widget.plotSaveBtn.clicked.connect(self.qt_callback_plotSaveBtn_clicked)
        self._widget.plotSendBtn.clicked.connect(self.qt_callback_plotSendBtn_clicked)
        
        # QT timer setup for UI updating
        self.timer_ui_update = QTimer()
        self.timer_ui_update.timeout.connect(self.timer_callback_ui_update)
        self.timer_ui_update.start(20) # set incriment (ms)

        # QT timer setup for sending initialization message after a delay
        self.timer_send_start = QTimer()
        self.timer_send_start.timeout.connect(self.timer_callback_send_start)
        self.timer_send_start.setSingleShot(True)  # Run only once
        self.timer_send_start.start(500) # (ms)

        # QT timer setup for sending dummy message to clear ethercat registers
        self.timer_send_dummy = QTimer()
        self.timer_send_dummy.timeout.connect(self.timer_callback_send_dummy)
        self.timer_send_dummy.setSingleShot(True)  # Run only once


        # ROS publisher stuff
        #wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1) 
        self.maze_ard0_pub = rospy.Publisher('/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1) # Esmacat write maze ard0 ease

        # ROS subscriber stuff
        rospy.Subscriber('Esmacat_read_maze_ard0_ease', ease_registers, self.ros_callback_Esmacat_read_maze_ard0_ease, tcp_nodelay=True)

        # Signal callback setup
        self.signal_Esmacat_read_maze_ard0_ease.connect(self.sig_callback_Esmacat_read_maze_ard0_ease)

        # Add a shutdown callback to unregister the subscriber when the ROS session is ending
        rospy.on_shutdown(self.ros_callback_shutdown)

    # CLASS: emulates c++ union type for storing ethercat data shareable accross 8 and 16 bit data types
    class Union:
        def __init__(self):
            self.ui8 = bytearray([0, 0])  # 2 bytes initialized with zeros

        @property
        def ui16(self):
            return struct.unpack("<H", self.ui8)[0]

        @ui16.setter
        def ui16(self, value):
            packed_value = struct.pack("<H", value)
            self.ui8[0] = packed_value[0]
            self.ui8[1] = packed_value[1]

    def ros_callback_shutdown(self):
        # Send END_SESSION message to arduino
        self.send_ethercat_message(P2A_Type_ID.END_SESSION)
        # TEMP self.timer_send_dummy.start(500) # (ms)

        # Unregister the subscriber
        if self._joint_sub:
            self._joint_sub.unregister()
    
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
            # TEMP rospy_log_col('HIGHLIGHT', "si16_arr[%d]=%d ui16_arr[%d]=%d", i, si16_arr[i], i, ui16_arr[i])

        # Parse the message and check if its new
        if self.get_ethercat_message(ui16_arr) != 0:
            return

        # Emit signal to update UI as UI should not be updated from a non-main thread
        self.signal_Esmacat_read_maze_ard0_ease.emit() 

    def sig_callback_Esmacat_read_maze_ard0_ease(self):
        pass

    def timer_callback_ui_update(self):
        # Update graphics
        self.scene.update()
        self._widget.plotMazeView.update()

    def timer_callback_send_start(self):
        # Send START_SESSION message to arduino
        self.send_ethercat_message(P2A_Type_ID.START_SESSION)
        self.timer_send_dummy.start(500) # (ms)

    def timer_callback_send_dummy(self):
        # Send CLEAR_REG message to arduino to clear the START_SESSION message from the registry
        self.send_ethercat_message(P2A_Type_ID.CLEAR_REG) 

    def qt_callback_fileBrowseBtn_clicked(self):
        # Filter only CSV files
        filter = "CSV Files (*.csv)"
        files, _ = QFileDialog.getOpenFileNames(
            None, "Select files to add", self.get_path_config_dir(), filter)

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
            self.load_csv_data(0)

    def qt_callback_fileListWidget_clicked(self, item):
        # Get the index of the clicked item and set it as the current file index
        self.current_file_index = self._widget.fileListWidget.currentRow()
        
        # Update file index and load csv
        self.load_csv_data(0)
            
    def qt_callback_fileNextBtn_clicked(self):
        # Update file index and load csv
        self.load_csv_data(1)
    
    def qt_callback_filePreviousBtn_clicked(self):
        # Update file index and load csv
        self.load_csv_data(-1)

    def qt_callback_plotClearBtn_clicked(self):
        Maze_Plot.CW_LIST.reset() # reset all values in list
        self.maze.update_walls() # update walls

    def qt_callback_plotSaveBtn_clicked(self):
        # Open the folder specified by self.get_path_config_dir() in an explorer window
        folder_path = self.get_path_config_dir()
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getSaveFileName(
            None, "Save CSV File", folder_path, "CSV Files (*.csv);;All Files (*)", options=options)

        if file_name:
            # Ensure the file name ends with ".csv"
            if not file_name.endswith(".csv"):
                file_name += ".csv"

            # The user has specified a file name, you can perform additional actions here
            rospy_log_col('INFO', "Selected file:", file_name)

            # Call the function to save wall config data to the CSV file with the wall array values converted to bytes
            self.save_data_to_csv(file_name, Maze_Plot.CW_LIST.get_byte_list())

    def qt_callback_plotSendBtn_clicked(self):
        # Sort entries
        Maze_Plot.CW_LIST.sort_entries()
        #rospy_log_col('INFO', Maze_Plot.CW_LIST)
        self.send_ethercat_message(P2A_Type_ID.MOVE_WALLS, 9, Maze_Plot.CW_LIST.get_byte_list())

    def get_path_config_dir(self, file_name=None):
        # Get the absolute path of the current script file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Create the path to the "config" directory four levels up
        dir_path =  os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', 'config', 'paths'))
        # Return file or dir path
        if file_name is not None:
            return os.path.join(dir_path, file_name)
        else:
            return dir_path
    
    def send_ethercat_message(self, p2a_msg_type, msg_arg_lng = 0, cw_list=None):
        
        # Itterate message id and roll over to 1 if max 16 bit value is reached
        self.p2aEtherMsgID = self.p2aEtherMsgID + \
            1 if self.p2aEtherMsgID < 65535 else 1

        # Create a list with 8 16-bit Union elements
        U = [Interface.Union() for _ in range(8)]
        ui16_i = 0

        # Set message id and type
        U[ui16_i].ui16 = self.p2aEtherMsgID
        ui16_i += 1
        U[ui16_i].ui8[0] = p2a_msg_type.value
        U[ui16_i].ui8[1] = msg_arg_lng

        # Update walls to move up
        if (p2a_msg_type == P2A_Type_ID.MOVE_WALLS) and cw_list is not None:
            # Update U_arr with corresponding chamber and wall byte
            for cw in cw_list:
                chamber = cw[0]
                ui16_i = 2 + (chamber // 2)
                wall_byte = cw[1]
                u_ind_c = 0 if chamber % 2 == 0 else 1
                U[ui16_i].ui8[u_ind_c] = wall_byte

        # Add footer
        ui16_i = 2 + math.ceil(msg_arg_lng/2)
        U[ui16_i].ui8[0] = 254
        U[ui16_i].ui8[1] = 254

        # Store and return 16-bit values cast as signed for use with ease_registers
        reg_arr = [ctypes.c_int16(U.ui16).value for U in U]

        # Print reg message
        rospy_log_col('INFO', "New ethercat message sent: %s %d", p2a_msg_type.name, self.p2aEtherMsgID)
        rospy_log_col('INFO', "\tui16[0] %d", U[0].ui16)
        for i in range(1, len(U)):
            rospy_log_col('INFO', "\tui8[%d]  %d %d", i, U[i].ui8[0], U[i].ui8[1])
 

        # Publish list to topic
        self.maze_ard0_pub.publish(*reg_arr)  

        # TEMP Print the cw_list
        # if cw_list is not None:
        #     rospy_log_col('INFO', "Chamber and wall configuration list:")
        #     for chamber, walls in cw_list:
        #         rospy_log_col('INFO', "Chamber %d: Walls %s", chamber, walls)

    def get_ethercat_message(self, reg_dat):
        a2p_msg_id = 0
        a2p_msg_type = 0
        msg_arg_lng = 0
        msg_arg_data = [0] * 10
        U = Interface.Union()

        # Check first register entry for msg id
        ui16_i = 0
        a2p_msg_id = reg_dat[ui16_i]
        ui16_i += 1
        U.ui16 = reg_dat[ui16_i]
        ui16_i += 1
        a2p_msg_type = U.ui8[0]
        msg_arg_lng = U.ui8[1]

        # Skip ethercat setup junk (65535)
        if a2p_msg_id == 65535 or self.a2pEtherMsgID == 65535:
            return 0

        # skip redundant messages and reset message type to NONE
        if a2p_msg_id == self.a2pEtherMsgID:
            self.a2pEtherMsgType = A2P_Type_ID.A2P_NONE
            return 0

        # Check for skipped or out of sequence messages
        if a2p_msg_id - self.a2pEtherMsgID != 1:
            if self.errorType != Error_Type.MESSAGE_ID_DISORDERED: # only run once
                # Set id last to new value on first error and set error type
                self.a2pEtherMsgID = a2p_msg_id 
                self.errorType = Error_Type.MESSAGE_ID_DISORDERED
                rospy_log_col('ERROR', "!!Ethercat message id mismatch: last=%d new=%d!!", self.a2pEtherMsgID, a2p_msg_id)
            return 1

        # Update dynamic enum instance
        self.a2pEtherMsgType.value = a2p_msg_type
        rospy_log_col('INFO', "\tNew ethercat message received: %s %d", self.a2pEtherMsgType.name, a2p_msg_id)

        # Update message id
        self.a2pEtherMsgID = a2p_msg_id


        # Parse 8 bit message arguments
        if msg_arg_lng > 0:
            msg_arg_lng_ui16_round = math.ceil(msg_arg_lng / 2) # divide message length by 2 and round up
            i8_i = 0
            for ui16_i in range(msg_arg_lng_ui16_round):
                # Get next entry
                U.ui16 = reg_dat[ui16_i]
                ui16_i += 1
                # Loop through bytes in given 16 bit entry and store
                for b_ii in range(2):
                    msg_arg_data[i8_i] = U.ui8[b_ii]
                    i8_i += 1

        # Check for footer
        U.ui16 = reg_dat[ui16_i]
        ui16_i += 1
        if U.ui8[0] != 254 and U.ui8[1] != 254:
            rospy_log_col('ERROR', "\t!!Missing message footer!!")
            self.errorType = Error_Type.MISSING_FOOTER
            return 1

        # HANDLE MESSAGE TYPE

        # CONFIRM_RECEIVED
        if self.a2pEtherMsgType == A2P_Type_ID.CONFIRM_RECEIVED:
            rospy_log_col('INFO', "\tCONFIRM_RECEIVED")

        # A2P_NONE
        elif self.a2pEtherMsgType == A2P_Type_ID.ERROR:
            rospy_log_col('INFO', "\tA2P_NONE")

        # Return new message flag
        return 0

    def save_data_to_csv(self, file_name, data):
        try:
            with open(file_name, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in data:
                    csv_writer.writerow(row)
            rospy_log_col('INFO', "Data saved to:", file_name)
        except Exception as e:
            rospy_log_col('ERROR', "Error saving data to CSV:", str(e))

    def load_csv_data(self, list_increment):
        # Update the current file index
        self.current_file_index += list_increment

        # Loop back to the end or start if start or end reached, respectively
        if list_increment < 0 and self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1 # set to end
        elif list_increment > 0 and self.current_file_index >= len(self.files):
            self.current_file_index = 0 # set to start

        # Set the current file in the list widget
        self._widget.fileListWidget.setCurrentRow(self.current_file_index)

        # Get the currently selected file path
        file_name = self.files[self.current_file_index]
        folder_path = self.get_path_config_dir()
        file_path = os.path.join(folder_path, file_name)

        # Load and store CSV data
        try:
            with open(file_path, 'r') as csv_file:
                csv_reader = csv.reader(csv_file)
                wall_byte_config_list = [[int(row[0]), int(row[1])] for row in csv_reader]
                Maze_Plot.CW_LIST.convert_byte_list(wall_byte_config_list)
                rospy_log_col('INFO', "Data loaded successfully.")
        except Exception as e:
            rospy_log_col('ERROR', "Error loading data from CSV:", str(e))

        # Update plot walls
        self.maze.update_walls()

# FUNCTION: Log to ROS in color
def rospy_log_col(level, message, *args):
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
    colored_message = f"{color}{message}{Style.RESET_ALL}"
    formatted_message = colored_message % args
    rospy.loginfo(formatted_message)
 