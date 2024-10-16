#!/usr/bin/env python
import os
import time
import rospy
import numpy as np
import math
import subprocess
import random
from std_msgs.msg import String, Int32, Int8
from geometry_msgs.msg import PoseStamped, PointStamped
from omniroute_operation.msg import *
from omniroute_esmacat_ros.msg import *
from dateutil.parser import parse as parsedate

import pandas as pd
from enum import Enum
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL
from PyQt5.QtWidgets import QGraphicsScene, QButtonGroup, QFileDialog, QWidget
from PyQt5.QtCore import QTimer

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin

import json
from omniroute_controller.omniroute_controller_interface import MazeDimensions
from experiment_controller.experiment_controller_interface import Wall
from experiment_controller.experiment_controller_interface import CommonFunctions

class Mode(Enum):
    START_EXPERIMENT = 1
    START_TRIAL = 2
    RAT_IN_START_CHAMBER = 3
    START = 4
    SOUND_CUE = 5
    START_TO_CHOICE = 6
    CHOICE = 7
    CHOICE_TO_GOAL = 8
    SUCCESS = 9
    REWARD_START = 10
    REWARD_END = 11
    POST_REWARD = 12
    REWARD_TO_RETURN = 13
    RETURN_TO_START = 14
    INTER_TRIAL_INTERVAL = 15
    ERROR = 16 
    ERROR_TO_RETURN = 17
    END_TRIAL = 18
    END_EXPERIMENT = 19
    PAUSE_EXPERIMENT = 20
    RESUME_EXPERIMENT = 21

class Interface(Plugin):
    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None
        self.setObjectName('Maze Experiment Interface')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'dynamic_training_controller_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        rospy.loginfo('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')

        # Add widget to the user interface
        context.add_widget(self._widget)

        self.scene = QGraphicsScene()

        self._widget.testingPhaseBtn.clicked.connect(self._handle_testingPhaseBtn_clicked)
        # Button for genetating trials migth be removed later depending on how we design the dynamic_training_controller
        self._widget.trialsGeneratorBtn.clicked.connect(self._handle_trialGeneratorBtn_clicked)
        self._widget.contTMazeBtn.clicked.connect(self._handle_contTMazeBtn_clicked)
        self._widget.lowerAllDoorsBtn.clicked.connect(self._handle_lowerAllDoorsBtn_clicked)
        # The starting chamber is always the same so no need to define it in the interface

        self.is_testing_phase = False
        self.trial_generator = False

        # Define all Publishers
        self.gantry_pub = rospy.Publisher('/gantry_cmd', GantryCmd, queue_size=1)
        self.write_sync_ease_pub = rospy.Publisher('/Esmacat_write_sync_ease', ease_registers, queue_size=1)
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)
        self.button_pub = rospy.Publisher('/button', String, queue_size=1)
        self.experiment_pub = rospy.Publisher('/experiment', String, queue_size=1)
        self.sound_pub = rospy.Publisher('/sound', String, queue_size=1)

        # Define all Subscribers
        rospy.Subscriber('/mode', String, self.mode_callback, queue_size=1)
        rospy.Subscriber('/rat_head_chamber', Int8, self.rat_head_chamber_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/rat_body_chamber', Int8, self.rat_body_chamber_callback, queue_size=1, tcp_nodelay=True)

        self.rat_head_chamber = -1
        self.rat_body_chamber = -1

        # Time for setting up publishers and subscribers
        rospy.sleep(1.0)

        self.experiment_pub.publish("dynamic_training_controller_experiment")

        ## EXPERIMENTAL PARAMETERS

        # File & Training parameters
        self.current_file_index = 0
        self.training_mode = None

        # Trial parameters
        self.currentTrial = []
        self.currentTrialNumber = 0
        self.nTrials = 0
        self.trials = []
        self.current_trial_index = 0

        # State parameters
        self.mode = Mode.START
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # Time parameters
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)

        # Delay parameters (change durations later)
        self.inter_trial_interval = rospy.Duration(1.0) # time between trials
        self.sound_delay = rospy.Duration(0.5) # Delay between sound cue and lowering the doors of the start chamber 
        self.choice_delay = rospy.Duration(1.5) # Duration to wait for rat to move to the choice point
        self.reward_start_delay = rospy.Duration(13) # Duration to wait to for the reward to dispense
        self.reward_end_delay = rospy.Duration(2) # Duration to wait to dispense reward if the rat made the right choice
        self.success_delay = rospy.Duration(1) # Delay after reward ends
        self.error_delay = rospy.Duration(2) # Delay after error

        # Stimulus parameters
        self.play_left_sound_cue = 0
        self.play_right_sound_cue = 0

        # Chamber parameters
        self.success_chamber = 0
        self.error_chamber = 0
        self.start_chamber = 0
        self.central_chamber = 0
        self.choice_chamber = 0
        self.return_chambers_left = 0
        self.return_chambers_right = 0

        # Wall parameters
        self.start_wall = Wall(0, 0)
        self.left_goal_entry_wall = Wall(0, 0)
        self.right_goal_entry_wall = Wall(0,0)
        self.left_goal_exit_wall = Wall(0,0)
        self.right_goal_exit_wall = Wall(0,0)
        self.left_return_wall = Wall(0,0)
        self.right_return_wall + Wall(0,0)

        # Rat parameters
        self.rat_position = 0

        # Maze parameters
        self.maze_dim = MazeDimensions()

        # Common functions
        self.common_functions = CommonFunctions()

        # Trial Types [Sound cue]
        self.trial_types = {
            1: ['1KHz'],
            2: ['8KHz']
        }

        self.trial_count = {key: 0 for key in self.trial_types} 

        # Define actions for clicking each button in the interface

        # Define messages (i.e. data) received from the gantry

        # Define run_experiment









