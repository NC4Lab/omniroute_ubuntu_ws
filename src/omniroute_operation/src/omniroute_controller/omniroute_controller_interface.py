#!/usr/bin/env python

# ======================== PACKAGES ========================

# Custom Imports
from argparse import ArgumentParser
from shared_utils.maze_debug import MazeDB
from shared_utils.projection_operation import ProjectionOperation
from shared_utils.esmacat_com import EsmacatCom
from shared_utils.ui_utilities import UIUtilities


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
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# PyQt and PySide Imports
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsItemGroup, QGraphicsLineItem, QGraphicsTextItem, QGraphicsPixmapItem
from PyQt5.QtCore import Qt, QRectF, QCoreApplication
from PyQt5.QtGui import QPen, QColor, QFont, QPixmap
from python_qt_binding import loadUi, QtOpenGL
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from qt_gui.plugin import Plugin
from PyQt5 import QtWidgets

# ======================== GLOBAL VARS ========================

NUM_ROWS_COLS = 3  # number of rows and columns in maze
WALL_MAP = {  # wall map for 3x3 maze [chamber_num][wall_num]
    0: [0, 1, 2, 3, 4, 5, 6, 7],
    1: [1, 2, 3, 5, 7],
    2: [0, 1, 2, 3, 4, 5, 6, 7],
    3: [0, 1, 3, 5, 7],
    4: [0, 1, 2, 3, 4, 5, 6, 7],
    5: [1, 3, 4, 5, 7],
    6: [0, 1, 2, 3, 4, 5, 6, 7],
    7: [1, 3, 5, 6, 7],
    8: [0, 1, 2, 3, 4, 5, 6, 7]
}

# ======================== GLOBAL CLASSES ========================


class MazeDimensions:
    # Class for storing the dimensions of the maze
    def __init__(self):
        self.chamber_wd = 0.3  # Chamber width (m)
        self.n_chamber_side = 3
        self.chamber_centers = []  # List of chamber centers

        # Compute the chamber centers
        for i in range(0, self.n_chamber_side**2):
            row = i//self.n_chamber_side
            col = i % self.n_chamber_side
            chamber_center = np.array([self.chamber_wd/2 + col*self.chamber_wd,
                                      self.chamber_wd/2 + (self.n_chamber_side-1-row)*self.chamber_wd])
            self.chamber_centers.append(chamber_center)


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

        Arguments:
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
                MazeDB.printMsg('DEBUG', "[%d][%d]", row[0], row[1])

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

            Arguments:
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
            self.test_num = _chamber_num
            self.center_x = _center_x
            self.center_y = _center_y
            self.chamber_width = _chamber_width

            # Create ROS publisher for gantry commands for mouse press events
            self.gantry_pub = rospy.Publisher(
                '/gantry_cmd', GantryCmd, queue_size=1)

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
            # Center the text over the chamber's center
            MazePlot._centerText(self.label, _center_x, _center_y)
            self.addToGroup(self.label)

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

            Arguments:
                status_enum (MazePlot.Status): Status enum
                do_force (bool): Force status update if true (Optional)
            """

            # Check and set the new status
            if not MazePlot.checkStatus(self.status, status_enum, do_force):
                return
            self.status = status_enum

            # Set text color
            self.label.setDefaultTextColor(self.status.color)

            # Print chamber status change
            MazeDB.printMsg('DEBUG', "Chamber %d: %s", self.chamber_num,
                            self.status.name)

            # Set chamber color
            # self.octagon.setBrush(QBrush(self.status.color))

        def mousePressEvent(self, event):
            """Handles mouse press events and sets the chamber status"""
            # [TODO]: Add functionality for setting all walls at once

            # Send command to move gantry to selected chamber
            self.gantry_pub.publish("move_to_chamber", [self.chamber_num])

            # # Bail if chamber is not enabled
            # if not MazePlot.isEnabled(self.status):
            #     return

            # if event.button() == Qt.LeftButton:

            #     # Change state setting of all walls
            #     for _, wall in enumerate(self.Walls):
            #         wall.togglePosStatus()

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

            # Create maze status var and set status
            self.status = MazePlot.Status.UNINITIALIZED
            self.setStatus(MazePlot.Status.UNINITIALIZED)

        def setStatus(self, status_enum, do_force=False):
            """
            Set/update maze status and set UI object colors

            Arguments:
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

        Arguments:
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
    signal_Esmacat_read_maze_ease = Signal()
    signal_rat_pos = Signal(int, int, int)

    def __init__(self, context):
        super(Interface, self).__init__(context)
        MazeDB.printMsg('ATTN', "Omniroute Conroller Interface Started")

        # ................ QT UI Setup ................


        # Set a name for this QObject instance. Helpful for debugging and identifying objects in complex UIs.
        self.setObjectName('Interface')

        # Import the ArgumentParser class from the argparse module.
        parser = ArgumentParser()

        # Define command-line arguments for this plugin, specifically adding a "quiet" mode.
        # "-q" or "--quiet" can be used as flags when starting the plugin to suppress output messages.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        
        # Parse the arguments passed to this plugin from the command line.
        # 'context.argv()' contains the command-line arguments for this specific plugin context.
        args, unknowns = parser.parse_known_args(context.argv())

        # If "quiet" mode is not enabled, print the arguments and any unrecognized options.
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create an empty QWidget that will serve as the base for the plugin UI.
        self._widget = QWidget()

        # Load and extend the QWidget with UI elements from the .ui file.
        loadUi(os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'omniroute_controller_interface.ui'), self._widget)

        # Assign a unique object name to the loaded UI, useful for debugging purposes.
        self._widget.setObjectName('InterfacePluginUi')

        # Set the title of the widget. If multiple instances of this plugin are opened,
        # append a unique serial number to the window title. This helps to distinguish
        # between different instances, especially when multiple instances are opened in the same session.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.plotMazeView.setViewportUpdateMode(
            QGraphicsView.FullViewportUpdate)
        self.scene = QGraphicsScene()
        self._widget.plotMazeView.setScene(self.scene)

        # Set the window to a fixed size
        UIUtilities.set_fixed_size(self._widget)

        # Move the window
        UIUtilities.move_ui_window(
            self._widget, horizontal_alignment='left', vertical_alignment='top')
        
        # Set the background color of the scene to white
        self._widget.plotMazeView.setBackgroundBrush(QColor(255, 255, 255))
        self._widget.plotMazeView.setViewport(QtOpenGL.QGLWidget())

        # Get the absolute path of the current script file
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Specify the defualt wall path directory
        path_dir_default = os.path.abspath(os.path.join(
            script_dir, '..', '..', '..', '..', 'data', 'paths'))

        # Set to default data file path
        self._widget.fileDirEdit.setText(path_dir_default)

        # Specify the defualt wall projection config directory
        self.proj_cfg_dir_default = os.path.abspath(os.path.join(
            script_dir, '..', '..', '..', '..', 'data', 'projection', 'image_config'))

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

        # Disable all but start and quit buttons
        self._widget.sysReinitBtn.setEnabled(False)
        self._widget.filePreviousBtn.setEnabled(False)
        self._widget.fileNextBtn.setEnabled(False)
        self._widget.plotClearBtn.setEnabled(False)
        self._widget.plotSaveBtn.setEnabled(False)
        self._widget.plotSendBtn.setEnabled(False)

        # ................ Maze Setup ................

        self.rat_pos_x = 350
        self.rat_pos_y = 150

        # Default system settings [default][min][max]
        self.sysDefaults = [
            [9, 1, 9],          # Num chamb init
            [3, 1, 9],          # Max chamb to move per block
            [2, 1, 3],          # Max move attempt
            [210, 0, 255],      # PWM duty
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

        # Add the rat image to the scene
        self.image = QPixmap(os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'rat.png'))
        self.rat_image = QGraphicsPixmapItem(self.image)
        self.rat_image_dim = round(self._widget.plotMazeView.height()*0.2)
        self.rat_image.setPixmap(self.image.scaled(
            self.rat_image_dim, self.rat_image_dim))
        self.rat_image.setPos(self.rat_pos_x, self.rat_pos_y)
        self.rat_image.setTransformOriginPoint(
            self.rat_image_dim/2, self.rat_image_dim/2)
        self.scene.addItem(self.rat_image)

        # ................ Initialize Other Class Variables ................

        # Track arduino connection states in a dictionary
        self.is_arduino_connected_list = {
            "maze_ease_connected": False,
            "gantry_ease_connected": False,
            "sync_ease_connected": False,
        }

        # Counter to incrementally shut down opperations
        self.cnt_shutdown_step = 0  # tracks the current step
        self.cnt_shutdown_ack_check = 0  # tracks number of times ack has been checked
        self.dt_shutdown_step = 0.25  # (sec)

        # ................ Maze Ecat Setup ................

        # Create EsmacatCom object for maze_ease
        self.EsmaCom = EsmacatCom('maze_ease')

        # Specify handshake ack timeout
        self.dt_check_handshake = 1000  # (ms)

        # ................ Projection Setup ................

        # Projection command publisher
        self.ProjOpp = ProjectionOperation()

        # ................ Gantry Setup ................

        self.MazeDim = MazeDimensions()

        # Marker to gantry center offset
        self.gantry_marker_to_gantry_center = np.array(
            [-0.317, -0.185])

        # To scale from maze to GUI coordinates
        optitrack_chamber_dist = math.sqrt((self.MazeDim.chamber_centers[0][0] - self.MazeDim.chamber_centers[1][0])**2 + (
            self.MazeDim.chamber_centers[0][1] - self.MazeDim.chamber_centers[1][1])**2)
        gui_chamber_dist = math.sqrt((self.MP.Chambers[0].center_x - self.MP.Chambers[1].center_x)**2 + (
            self.MP.Chambers[0].center_y - self.MP.Chambers[1].center_y)**2)
        self.optitrack_to_gui_scale = gui_chamber_dist/optitrack_chamber_dist

        # ................ ROS Setup ................

        # ROS wall state subscriber used to set walls from other interfaces
        self.wall_config = rospy.Subscriber(
            '/wall_state_cmd', WallState, self.ros_callback_wall_config, queue_size=100, tcp_nodelay=True)

        # ROS gantry command publisher
        self.gantry_pub = rospy.Publisher(
            '/gantry_cmd', GantryCmd, queue_size=1)

        # ROS sync command publisher
        self.sync_pub = rospy.Publisher(
            '/sync_cmd', SyncCmd, queue_size=1)

        # ROS event publisher and subscriber
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)
        self.event_sub = rospy.Subscriber(
            '/event', Event, self.ros_callback_event, queue_size=1, tcp_nodelay=True)

        # Harness position subscriber
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped,
                         self.ros_callback_harness_pose, queue_size=1, tcp_nodelay=True)

        self.signal_rat_pos.connect(self.sig_callback_update_rat_pos_in_gui)

        # ................ Callback Setup ................

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

        # Gantry ui callbacks
        self._widget.runGantryBtn.clicked.connect(
            self.qt_callback_runGantryBtn_clicked)
        self._widget.homeGantryBtn.clicked.connect(
            self.qt_callback_homeGantryBtn_clicked)
        self._widget.trackHarnessTogBtn.clicked.connect(
            self.qt_callback_trackHarnessTogBtn_clicked)
        self._widget.lowerFeederTogBtn.clicked.connect(
            self.qt_callback_lowerFeederTogBtn_clicked)
        self._widget.runPumpTogBtn.clicked.connect(
            self.qt_callback_runPumpTogBtn_clicked)
        self._widget.rewardBtn.clicked.connect(
            self.qt_callback_rewardBtn_clicked)

        # Projector mode ui callbacks
        self._widget.projWinTogBtn.clicked.connect(
            self.qt_callback_projWinTogBtn_clicked)
        self._widget.projWinTogFullScrBtn.clicked.connect(
            self.qt_callback_projWinTogFullScrBtn_clicked)
        self._widget.projWinForceFucusBtn.clicked.connect(
            self.qt_callback_projWinForceFucusBtn_clicked)

        # QT timer setup for UI updating
        self.timer_updateUI = QTimer()
        self.timer_updateUI.timeout.connect(self.timer_callback_updateUI_loop)
        self.timer_updateUI.start(20)  # set incriment (ms)

        # QT timer setup for checking ROS ECAT messages
        self.timer_updateUI = QTimer()
        self.timer_updateUI.timeout.connect(self.timer_callback_checkECAT_loop)
        self.timer_updateUI.start(20)  # set incriment (ms)

        # QT timer to send and check handshake confirmation after a delay
        self.timer_sendCheckHandshake = QTimer()
        self.timer_sendCheckHandshake.timeout.connect(
            self.timer_callback_handshakeHandler_once)
        self.timer_sendCheckHandshake.setSingleShot(True)  # Run only once

        # QT timer to handle incremental shutdown
        self.timer_endSession = QTimer()
        self.timer_endSession.timeout.connect(
            self.timer_callback_endSession_once)
        self.timer_endSession.setSingleShot(True)  # Run only once

        # Projected wall image ui callback
        self.proj_wall_img_cfg_btn_vec = []  # Initalize vector for buttons
        for i in range(9):
            button_name = f'projWallImgCfgBtn_{i}'
            button = getattr(self._widget, button_name)
            button.clicked.connect(  # Use lambda pass button index tor callback
                lambda _, b=i: self.qt_callback_projWallImgCfgBtn_clicked(b))
            self.proj_wall_img_cfg_btn_vec.append(button)  # Store the button

        # Projected floor image ui callback
        self.proj_floor_img_cfg_btn_vec = []  # Initalize vector for buttons
        for i in range(4):
            button_name = f'projFloorImgCfgBtn_{i}'
            button = getattr(self._widget, button_name)
            button.clicked.connect(  # Use lambda pass button index tor callback
                lambda _, b=i: self.qt_callback_projFloorImgCfgBtn_clicked(b))
            self.proj_floor_img_cfg_btn_vec.append(button)  # Store the button
    
    # ------------------------ FUNCTIONS: Ecat Communicaton ------------------------

    def procEcatMessage(self):
        """ Used to parse new incoming ROS ethercat msg data. """

        # Print confirmation message
        MazeDB.printMsg('INFO', "Processing Ecat Message [id=%d]: Msg[%s]",
                        self.EsmaCom.rcvEM.msgID, self.EsmaCom.rcvEM.msgTp.name)

        # ................ Process Ack Error First ................

        if self.EsmaCom.rcvEM.errTp != EsmacatCom.ErrorType.ERR_NONE:

            MazeDB.printMsg('WARNING', "Ecat Message [id=%d]: Err[%s]",
                            self.EsmaCom.rcvEM.msgID, self.EsmaCom.rcvEM.errTp.name)

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
                            'WARNING', "\t chamber[%d] i2c_status[%d]", cham_i, i2c_status)

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
                            'WARNING', "\t chamber[%d] walls%s", cham_i, MazeDB.arrStr(wall_numbers))

        # ................ Process Ack Message ................

        # HANDSHAKE
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.HANDSHAKE:
            MazeDB.printMsg('ATTN', "Maze Arduino Handshake Confirmed")

            # Publish to ROS event
            self.event_pub.publish("maze_ease_connected", rospy.Time.now())

            # Set the handshake flag
            self.EsmaCom.isEcatConnected = True

            # Set maze hardware status to initialized
            self.MP.setStatus(MazePlot.Status.INITIALIZED)

            # Send INITIALIZE_CYPRESS message
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.INITIALIZE_CYPRESS)

            # Enable buttons
            self._widget.sysReinitBtn.setEnabled(True)
            self._widget.filePreviousBtn.setEnabled(True)
            self._widget.fileNextBtn.setEnabled(True)
            self._widget.plotClearBtn.setEnabled(True)
            self._widget.plotSaveBtn.setEnabled(True)
            self._widget.plotSendBtn.setEnabled(True)

        # INITIALIZE_CYPRESS
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.INITIALIZE_CYPRESS:
            MazeDB.printMsg('ATTN', "Cypress I2C Initialization Confirmed")

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
            MazeDB.printMsg('ATTN', "Wall Initialization Confirmed")

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
            MazeDB.printMsg('ATTN', "System Reinitialization Confirmed")

            # Send INITIALIZE_WALLS message again
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.INITIALIZE_WALLS)

        # RESET_SYSTEM
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.RESET_SYSTEM:
            MazeDB.printMsg('ATTN', "Ecat Disconnection Confirmed")

            # Reset the handshake flag
            self.EsmaCom.isEcatConnected = False

        # Reset new message flag
        self.EsmaCom.rcvEM.isNew = False

    # ------------------------ CALLBACKS: Timers ------------------------

    def timer_callback_updateUI_loop(self):
        """ Timer callback to update UI. """

        # Update graphics
        self.scene.update()
        self._widget.plotMazeView.update()

    def timer_callback_checkECAT_loop(self):
        """ Timer callback to check for new ECAT messages. """

        # Check for new message
        if self.EsmaCom.rcvEM.isNew:
            self.procEcatMessage()

    def timer_callback_handshakeHandler_once(self):
        """ Timer callback to check for handshake confirmation. """

        # Check for HANDSHAKE message confirm recieved flag
        if self.EsmaCom.isEcatConnected == False:

            # Log error
            MazeDB.printMsg(
                'ERROR', "Maze Arduino Handshake Failed")

            # Set maze hardware status to error
            self.MP.setStatus(MazePlot.Status.ERROR)

            # Set arduino list widget to error color
            self.MP.setStatus(MazePlot.Status.ERROR)

    def timer_callback_endSession_once(self):
        """ Timer callback to incrementally shutdown session. """

        if self.cnt_shutdown_step == 0:
            # Send RESET_SYSTEM message if connected
            if self.EsmaCom.isEcatConnected:
                self.EsmaCom.writeEcatMessage(
                    EsmacatCom.MessageType.RESET_SYSTEM)

        elif self.EsmaCom.isEcatConnected == True:
            if self.cnt_shutdown_ack_check > int(30/self.dt_shutdown_step):
                MazeDB.printMsg(
                    'ERROR', "FAILED: SHUTDOWN: RESET_SYSTEM CONFIRMATION AFTER 30 SECONDS")
                self.EsmaCom.isEcatConnected = False
            else:
                self.cnt_shutdown_ack_check += 1
                # Wait for RESET_SYSTEM message confirmation and restart timer
                self.timer_endSession.start(self.dt_shutdown_step*1000)
                return

        elif self.cnt_shutdown_step == 1:
            # Kill self.signal_Esmacat_read_maze_ease thread
            self.signal_Esmacat_read_maze_ease.disconnect()
            MazeDB.printMsg(
                'INFO', "SHUTDOWN: Disconnected from Esmacat read timer thread")

        elif self.cnt_shutdown_step == 2:
            # Kill specific nodes
            # self.terminate_ros_node("/gantry_operation_node")
            self.terminate_ros_node("/projection_opperation_node")
            self.terminate_ros_node("/Esmacat_application_node")
            self.terminate_ros_node("/omniroute_controller_node")
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
            MazeDB.printMsg(
                'INFO', "SHUTDOWN: Sent shutdown request to ROS master")

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

    def qt_callback_homeGantryBtn_clicked(self):
        """ Callback function for the "Home Gantry" button."""
        self.gantry_pub.publish("home_gantry", [])
        self._widget.xSpinBox.setValue(0)
        self._widget.ySpinBox.setValue(0)

    def qt_callback_runGantryBtn_clicked(self):
        """ Callback function for the "Run Gantry" button."""
        x = self._widget.xSpinBox.value()
        y = self._widget.ySpinBox.value()
        MazeDB.printMsg(
            'DEBUG', "Publish Move to Coordinates: x[%0.2f] y[%0.2f]", x, y)
        self.gantry_pub.publish("move_to_coordinate", [x, y])

    def qt_callback_trackHarnessTogBtn_clicked(self):
        """ Callback function to start and stop gantry tracking the rat from button press."""
        if self._widget.trackHarnessTogBtn.isChecked():
            self.gantry_pub.publish("start_harness_tracking", [])
        else:
            self.gantry_pub.publish("stop_harness_tracking", [])

    def qt_callback_lowerFeederTogBtn_clicked(self):
        """ Callback function to lower or raise the feeder from button press."""
        if self._widget.lowerFeederTogBtn.isChecked():
            self.gantry_pub.publish("lower_feeder", [])
        else:
            self.gantry_pub.publish("raise_feeder", [])

    def qt_callback_runPumpTogBtn_clicked(self):
        """ Callback function to run or stop the pump from button press."""
        if self._widget.runPumpTogBtn.isChecked():
            self.gantry_pub.publish("start_pump", [])
        else:
            self.gantry_pub.publish("stop_pump", [])

    def qt_callback_rewardBtn_clicked(self):
        """ Callback function to run the full feeder opperation from button press."""
        self.gantry_pub.publish("deliver_reward", [1.0])

    def qt_callback_projWinTogBtn_clicked(self):
        """ Callback function to toggle if projector widnows are on the main monitor or prjectors from button press."""

        # Code -1
        self.ProjOpp.publish_command_message(-1)
        MazeDB.printMsg('DEBUG', "Command for projWinTogBtn sent")

    def qt_callback_projWinTogFullScrBtn_clicked(self):
        """ Callback function to change projector widnows position from button press."""

        # Code -2
        self.ProjOpp.publish_command_message(-2)
        MazeDB.printMsg('DEBUG', "Command for projWinTogFullScrBtn sent")

    def qt_callback_projWinForceFucusBtn_clicked(self):
        """ Callback function to force windows to the top of the display stack from button press."""

        # Code -3
        self.ProjOpp.publish_command_message(-3)
        MazeDB.printMsg('DEBUG', "Command for projWinForceFucusBtn sent")

    def qt_callback_projWallImgCfgBtn_clicked(self, button_number):
        """ Callback function to send projector wall image config from button press."""

        # Get the button that was clicked
        clicked_button = self.proj_wall_img_cfg_btn_vec[button_number]

        # Uncheck all the buttons except the one that was clicked
        for i, button in enumerate(self.proj_wall_img_cfg_btn_vec):
            if i != button_number:
                button.setChecked(False)

        # Load the appropriate file based on the button number
        file_name = None
        if button_number == 0:
            file_name = 'walls_0_blank.csv'
        elif button_number == 1:
            file_name = 'walls_1_east_r.csv'
        elif button_number == 2:
            file_name = 'walls_2_east_l.csv'
        elif button_number == 3:
            file_name = 'walls_3_south_r.csv'
        elif button_number == 4:
            file_name = 'walls_4_south_l.csv'
        elif button_number == 5:
            file_name = 'walls_5_west_r.csv'
        elif button_number == 6:
            file_name = 'walls_6_west_l.csv'
        elif button_number == 7:
            file_name = 'walls_7_north_r.csv'
        elif button_number == 8:
            file_name = 'walls_8_north_l.csv'

        # Format full path
        csv_path = os.path.join(self.proj_cfg_dir_default, file_name)

        # Load and store CSV data
        self.ProjOpp.set_config_from_csv(csv_path, "walls")

        # Send the new image configuration
        self.ProjOpp.publish_image_message()
        MazeDB.printMsg(
            'Sent', "Sent ROS Wall Image Configuration: file[%s]", file_name)

    def qt_callback_projFloorImgCfgBtn_clicked(self, button_number):
        """ Callback function to send projector floor image config from button press."""

        # Get the button that was clicked
        clicked_button = self.proj_floor_img_cfg_btn_vec[button_number]

        # Uncheck all the buttons except the one that was clicked
        for i, button in enumerate(self.proj_floor_img_cfg_btn_vec):
            if i != button_number:
                button.setChecked(False)

        # Load the appropriate file based on the button number
        file_name = None
        if button_number == 0:
            file_name = 'floor_0_blank.csv'
        elif button_number == 1:
            file_name = 'floor_1_green.csv'
        elif button_number == 2:
            file_name = 'floor_2_pat_1.csv'
        elif button_number == 3:
            file_name = 'floor_3_pat_2.csv'

        # Format full path
        csv_path = os.path.join(self.proj_cfg_dir_default, file_name)

        # Load and store CSV data
        self.ProjOpp.set_config_from_csv(csv_path, "floor")

        # Send the new image configuration
        self.ProjOpp.publish_image_message()
        MazeDB.printMsg(
            'Sent', "Sent ROS Floor Image Configuration: file[%s]", file_name)

    def qt_callback_sysStartBtn_clicked(self):
        """ 
        Callback function for the "Start" button.

        This function sends the initial system command to all the arduinos, which, 
        most importantly, triggers the handshake exchange.
        """

        # Send command to initialize the gantry
        self.gantry_pub.publish("initialize_gantry", [])

        # Sebd command to initialize sync sender
        self.sync_pub.publish("initialize_sync_sender", [])

        # Send HANDSHAKE to maze arduino with current system settings
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.HANDSHAKE, self.getParamTxtBox())

        # Start handshake callback timer
        self.timer_sendCheckHandshake.start(self.dt_check_handshake)


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
                        wall.setStatus(
                            MazePlot.Status.UNINITIALIZED, do_force=True)

    def qt_callback_sysQuiteBtn_clicked(self):
        """ Callback function for the "Quit" button."""

        # Call timer callback to incrementally shutdown session
        self.timer_endSession.start(0)

    # ------------------------ CALLBACKS: Other ------------------------

    def ros_callback_wall_config(self, msg):
        """ Callback function for subscribing to experiment controller command."""

        # Validate chamber and wall numbers
        if msg.chamber < -1 or msg.chamber >= self.MP.num_rows_cols**2:
            MazeDB.printMsg(
                'WARNING', "Invalid Chamber Number: %d", msg.chamber)
            return
        for wall in msg.wall:
            if wall < 0 or wall >= 8:
                MazeDB.printMsg('WARNING', "Invalid Wall Number: %d", msg.wall)
                return

        if msg.chamber != -1:
            for wall in msg.wall:
                if msg.state:
                    WallConfig.add_wall(msg.chamber, wall)
                else:
                    WallConfig.remove_wall(msg.chamber, wall)

        if msg.send:
            # Sort entries
            WallConfig._sort_entries()

            # Send MOVE_WALLS message with wall byte array
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.MOVE_WALLS, WallConfig.get_wall_byte_list())

        # Update walls
        self.MP.updatePlotFromWallConfig()

    def ros_callback_harness_pose(self, msg):
        harness_x = msg.pose.position.x
        harness_y = msg.pose.position.y

        # Convert to GUI coordinates
        harness_pose_in_gui = self.optitrack_to_gui(
            [harness_x, harness_y])

        # Convert orientation to euler angles
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Convert to degrees and adjust for optitrack orientation
        yaw = -(int(yaw * 180 / math.pi)-30)

        self.signal_rat_pos.emit(
            harness_pose_in_gui[0], harness_pose_in_gui[1], yaw)

    def ros_callback_event(self, msg):
        """ Callback function to track handshake events for all arduinos. """

        # Check if event is relevant and update connection state if it is
        if msg.event in self.is_arduino_connected_list:
            self.is_arduino_connected_list[msg.event] = True
        else:
            return

        # Check if all connection states are True
        if all(self.is_arduino_connected_list.values()):
            # Disable Start button
            self._widget.sysStartBtn.setEnabled(False)

            # Enable Reinitialize button
            self._widget.sysReinitBtn.setEnabled(True)

    def sig_callback_update_rat_pos_in_gui(self, x, y, yaw):
        """
        Update variables for rat position used by the GUI
        """
        self.rat_pos_x = x
        self.rat_pos_y = y
        self.rat_image.setPos(self.rat_pos_x, self.rat_pos_y)
        self.rat_image.setRotation(yaw)

    # ------------------------ FUNCTIONS: Minor ------------------------

    def optitrack_to_gui(self, optitrack_pos):
        """
        Converts an Optitrack position (coordinates from a tracking system) to a GUI-compatible position.

        Arguments:
        - optitrack_pos (array-like): A 2D position [x, y] in the Optitrack coordinate system.

        Returns:
        - gui_pos (np.array): A 2D integer array [x, y] representing the converted position in the GUI.
        """
        # Initialize the GUI position array with zeros (2D position)
        gui_pos = np.zeros(2)

        # Convert the x-coordinate:
        # - Translate the Optitrack x-coordinate relative to the maze chamber's center.
        # - Scale to match GUI scaling factors.
        # - Offset by the GUI chamber center and adjust to align with the rat image dimensions.
        gui_pos[0] = (optitrack_pos[0] - self.MazeDim.chamber_centers[0][0]) * \
            self.optitrack_to_gui_scale + \
            self.MP.Chambers[0].center_x - self.rat_image_dim / 2

        # Convert the y-coordinate:
        # - Similar to the x-coordinate but with an inversion (negative scale) to align GUI axes.
        gui_pos[1] = -((optitrack_pos[1] - self.MazeDim.chamber_centers[0][1]) *
                       self.optitrack_to_gui_scale - self.MP.Chambers[0].center_y) - self.rat_image_dim / 2

        # Convert the resulting GUI position to integers (required for most GUI positioning)
        gui_pos = gui_pos.astype(int)

        # Get the width and height of the GUI scene to use for bounding the position
        sceneWidth = self._widget.plotMazeView.width()
        sceneHeight = self._widget.plotMazeView.height()

        # Bound the x-coordinate of the GUI position to ensure it stays within the GUI scene width
        if gui_pos[0] < 0:
            gui_pos[0] = 0
        elif gui_pos[0] > sceneWidth:
            gui_pos[0] = sceneWidth

        # Bound the y-coordinate of the GUI position to ensure it stays within the GUI scene height
        if gui_pos[1] < 0:
            gui_pos[1] = 0
        elif gui_pos[1] > sceneHeight:
            gui_pos[1] = sceneHeight

        # Return the bounded, scaled, and translated GUI position
        return gui_pos

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

        # Update the current file index by adding the given increment value
        self.current_file_index += list_increment

        # Loop back to the end or start if the index goes out of range, effectively creating a circular index
        if list_increment < 0 and self.current_file_index < 0:
            # Set to the last index if negative overflow occurs
            self.current_file_index = len(self.csv_files) - 1
        elif list_increment > 0 and self.current_file_index >= len(self.csv_files):
            self.current_file_index = 0  # Set to the first index if positive overflow occurs

        # Set the current file in the list widget for user interface purposes
        self._widget.fileListWidget.setCurrentRow(self.current_file_index)

        # Get the directory path and current file name to construct the full file path
        dir_path = self._widget.fileDirEdit.text()
        file_name = self.csv_files[self.current_file_index]
        file_path = os.path.join(dir_path, file_name)

        # Load and store CSV data into the wall configuration
        try:
            with open(file_path, 'r') as csv_file:
                csv_reader = csv.reader(csv_file)
                
                # Read each row and convert the data to integers, representing chamber and wall configuration
                wall_byte_config_list = [
                    [int(row[0]), int(row[1])] for row in csv_reader]
                
                # Update the wall configuration using the WallConfig class
                WallConfig.make_byte2num_cw_list(wall_byte_config_list)
                MazeDB.printMsg(
                    'INFO', "CSV: Data Loaded from File: %s", file_name)
                
        except Exception as e:
            # Print an error message if loading the CSV file fails
            MazeDB.printMsg('ERROR', "CSV: Loading Data Error: %s", str(e))

        # Update the plotted wall configuration based on the newly loaded data
        self.MP.updatePlotFromWallConfig()

    def saveToCSV(self, save_file_path, wall_config_list):
        """ Function to save the wall config data to a CSV file """

        try:
            # Open the file for writing and use a CSV writer to write each row of wall configuration data
            with open(save_file_path, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in wall_config_list:
                    csv_writer.writerow(row)
           
            # Extract the file name from the full path to log it
            save_file_name = os.path.basename(save_file_path)
            MazeDB.printMsg(
                'INFO', "CSV: Data Saved to File: %s", save_file_name)
            
        except Exception as e:
            # Print an error message if saving the CSV file fails
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

    def terminate_ros_node(self, s):
        """ Function to terminate a ROS node when exiting the application """

        list_cmd = subprocess.Popen(
            "rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.decode().split("\n"):
            if (str.startswith(s)):
                MazeDB.printMsg('INFO', "Killing Node: %s", str)
                os.system("rosnode kill " + str)

    def closeEvent(self, event):
        """ Function to handle the window close event """
        # [TODO]: Implement this function to handle the window close event

        MazeDB.printMsg('INFO', "Closing window...")
        # Call function to shut down the ROS session
        self.end_ros_session()
        event.accept()  # let the window close
