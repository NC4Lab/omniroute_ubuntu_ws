#!/usr/bin/env python
from shared_utils.ui_utilities import UIUtilities
from shared_utils.maze_debug import MazeDB
from shared_utils.wall_utilities import MazeDimensions, WallConfig
from experiment_controller.experiment_controller_interface import Wall
from experiment_controller.experiment_controller_interface import CommonFunctions

import os
import rospy
import numpy as np
import random
from std_msgs.msg import String, Int32, Int8
from geometry_msgs.msg import PoseStamped, PointStamped
from omniroute_operation.msg import *
from omniroute_esmacat_ros.msg import *
from dateutil.parser import parse as parsedate

from enum import Enum, auto
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL
from PyQt5.QtWidgets import QGraphicsScene, QButtonGroup, QFileDialog, QWidget
from PyQt5.QtCore import QTimer

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin

import psytrack as psy
from psytrack.helper.helperFunctions import read_input

class Mode(Enum):
    START_EXPERIMENT = auto()
    START_TRIAL = auto()
    PAUSE_EXPERIMENT = auto()
    RESUME_EXPERIMENT = auto()


class Interface(Plugin):
    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None
        self.setObjectName('Radial Maze Interface')

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(
            __file__)), 'radial_maze_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        rospy.loginfo('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self.mode = Mode.START_EXPERIMENT

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)

        self.current_time = rospy.Time.now()

        self.scene = QGraphicsScene()

        self.projection_floor_pub = rospy.Publisher('projection_image_floor_num', Int32, queue_size=100)

        rospy.Subscriber('/mode', String, self.mode_callback, queue_size=1)

        self.common_functions = CommonFunctions()


        self.projection_floor_pub = rospy.Publisher('projection_image_floor_num', Int32, queue_size=100)

        rospy.Subscriber('/rat_head_chamber', Int8, self.rat_head_chamber_callback, queue_size=1, tcp_nodelay=True)

        rospy.Subscriber('/rat_body_chamber', Int8, self.rat_body_chamber_callback, queue_size=1, tcp_nodelay=True)
        

        self.floor_img_green_num = 1

        self.rat_head_chamber = -1
        self.rat_body_chamber = -1
        self.previous_rat_chamber = -1
        

    def lowerWalls(self):
    # Lower Walls 0,2,4,6 in chamber 4 (central chamber)
        for i in [0, 2, 4, 6]:
            self.common_functions.lower_wall(Wall(4, i), False)
        self.common_functions.activateWalls()

    def raiseWalls(self):
        for i in range(8):
            self.common_functions.raise_wall(Wall(4, i), False)
        self.common_functions.activateWalls()

    def mode_callback(self, msg):
        mode = msg.data
        if mode == "START_EXPERIMENT":
            self.mode = Mode.START_EXPERIMENT
        elif mode == "PAUSE_EXPERIMENT":
            self.mode_before_pause = self.mode
            self.mode = Mode.PAUSE_EXPERIMENT
        elif mode == "RESUME_EXPERIMENT":
            self.mode = Mode.RESUME_EXPERIMENT

    def rat_head_chamber_callback(self, msg):
        self.rat_head_chamber = msg.data

    def rat_body_chamber_callback(self, msg):
        self.rat_body_chamber = msg.data

    def run_experiment(self):
    # This function loops
        rospy.loginfo("EXPERIMENT RUNNING")
        self.current_time = rospy.Time.now()
        current_rat_chamber = self.rat_head_chamber

        if current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1:
            # The rat has moved to a different chamber, update the gantry position
            self.common_functions.move_gantry_to_chamber(current_rat_chamber)

            # Update the previous_rat_chamber for the next iteration
            self.previous_rat_chamber = current_rat_chamber

        if self.mode == Mode.START_EXPERIMENT:
            rospy.loginfo("START OF THE EXPERIMENT")
            self.raiseWalls()
            self.projection_floor_pub.publish(self.floor_img_green_num)
            rospy.sleep(3)
            self.mode = Mode.START_TRIAL

        elif self.mode == Mode.START_TRIAL:
            rospy.loginfo("START OF TRIAL")
            self.lowerWalls()
            rospy.sleep(3)
            self.mode = Mode.START_EXPERIMENT
        

        


if __name__ == '__main__':
    rospy.init_node('radial_maze')
    Interface()
    rospy.spin()