#!/usr/bin/env python
from shared_utils.ui_utilities import UIUtilities
from shared_utils.maze_debug import MazeDB
from shared_utils.wall_utilities import MazeDimensions, WallConfig
from experiment_controller.experiment_controller_interface import Wall
from experiment_controller.experiment_controller_interface import CommonFunctions

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

import psytrack as psy
from psytrack.helper.helperFunctions import read_input


class Mode(Enum):
    START_INTERFACE = 0
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
    REWARD_RETURN_TO_START = 22
    ERROR_RETURN_TO_START = 23
    ERROR_START = 24
    ERROR_END = 25


class Interface(Plugin):
    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None
        self.setObjectName('Pseudorandom Control Interface')

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
            __file__)), 'pseudorandom_control_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        rospy.loginfo('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self.scene = QGraphicsScene()

        # Define all buttons in the interface
        self._widget.alignMaxTrainingBtn.clicked.connect(
            self._handle_alignMaxTrainingBtn_clicked)
        self._widget.learnMaxTrainingBtn.clicked.connect(
            self._handle_learnMaxTrainingBtn_clicked)
        self._widget.pseudorandomTrainingBtn.clicked.connect(
            self._handle_pseudorandomTrainingBtn_clicked)
        self._widget.preTrainingPhaseOneBtn.clicked.connect(
            self._handle_preTrainingPhaseOneBtn_clicked)
        self._widget.preTrainingPhaseTwoBtn.clicked.connect(
            self._handle_preTrainingPhaseTwoBtn_clicked)
        self._widget.contTMazeBtn.clicked.connect(
            self._handle_contTMazeBtn_clicked)
        self._widget.lowerAllDoorsBtn.clicked.connect(
            self._handle_lowerAllDoorsBtn_clicked)
        self._widget.lowerDoorBtn.clicked.connect(
            self._handle_lowerDoorBtn_clicked)
        # The starting chamber is always the same so no need to define it in the interface

        self.is_testing_phase = False
        self.alignMax_training = False
        self.learnMax_training = False
        self.pseudorandom_training = False
        self.pretraining_phase_one = False
        self.pretraining_phase_two = False

        # Define all Publishers
        self.gantry_pub = rospy.Publisher(
            '/gantry_cmd', GantryCmd, queue_size=1)
        self.write_sync_ease_pub = rospy.Publisher(
            '/Esmacat_write_sync_ease', ease_registers, queue_size=1)
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)
        self.button_pub = rospy.Publisher('/button', String, queue_size=1)
        self.experiment_pub = rospy.Publisher(
            '/experiment', String, queue_size=1)
        self.sound_pub = rospy.Publisher('sound_cmd', String, queue_size=1)
        self.stimulus_pub = rospy.Publisher(
            '/stimulus_cmd', String, queue_size=1)

        # Define all Subscribers
        rospy.Subscriber('/mode', String, self.mode_callback, queue_size=1)
        rospy.Subscriber('/rat_head_chamber', Int8,
                         self.rat_head_chamber_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/rat_body_chamber', Int8,
                         self.rat_body_chamber_callback, queue_size=1, tcp_nodelay=True)

        self.rat_head_chamber = -1
        self.rat_body_chamber = -1

        # Time for setting up publishers and subscribers
        rospy.sleep(1.0)

        self.experiment_pub.publish("pseudorandom_controller_experiment")

        # EXPERIMENTAL PARAMETERS

        # File & Training parameters
        self.current_file_index = 0
        self.training_mode = None

        # Trial parameters
        self.nDay = 0
        self.currentTrial = []
        self.currentTrialNumber = 0
        self.nTrials = 60
        self.trials = []
        self.current_trial_index = 0
        self.nPrevTrials = 0

        # State parameters
        self.mode = Mode.START_INTERFACE
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # Time parameters
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)

        # Delay parameters (change durations later)
        self.inter_trial_interval = rospy.Duration(1.0)  # time between trials
        # Duration of delay in the beginning of the trial
        self.start_first_delay = rospy.Duration(5.0)
        # Delay between sound cue and lowering the doors of the start chamber
        self.sound_delay = rospy.Duration(5)
        # Duration to wait for rat to move to the choice point
        self.choice_delay = rospy.Duration(2)
        # Duration to wait to for the reward to dispense
        self.reward_start_delay = rospy.Duration(10)
        # Duration to wait to dispense reward if the rat made the right choice
        self.reward_end_delay = rospy.Duration(2)
        self.success_delay = rospy.Duration(1)  # Delay after reward ends
        self.error_delay = rospy.Duration(40)  # Delay after error
        self.end_trial_delay = rospy.Duration(
            1)  # Delay after the end of the trial
        self.error_start_delay = rospy.Duration(8)
        self.error_end_delay = rospy.Duration(2)

        # Stimulus parameters
        self.play_left_sound_cue = 0
        self.play_right_sound_cue = 0
        self.sound_cue = "1kHz"

        # Chamber parameters
        self.success_chamber = 0
        self.error_chamber = 0
        self.start_chamber = 0
        self.central_chamber = 0
        self.choice_chamber = 0
        self.left_return_chamber = 0
        self.right_return_chamber = 0
        self.previous_rat_chamber = 0

        # Wall parameters
        self.start_wall = Wall(0, 0)
        self.left_goal_entry_wall = Wall(0, 0)
        self.right_goal_entry_wall = Wall(0, 0)
        self.left_goal_exit_wall = Wall(0, 0)
        self.right_goal_exit_wall = Wall(0, 0)
        self.left_return_wall = Wall(0, 0)
        self.right_return_wall = Wall(0, 0)

        # Rat parameters
        self.rat_position = 0
        current_rat_chamber = self.rat_head_chamber

        # Maze parameters
        self.maze_dim = MazeDimensions()

        # Learning Parameters
        self.stage = "early"
        self.early_learning_threshold = 300
        self.performance_threshold = 0.70

        # Trial Data Parameters
        self.y = y = np.empty(self.nTrials, dtype=int)
        self.inputs = np.empty(self.nTrials, dtype=int)
        self.name = []
        self.answer = np.empty(self.nTrials, dtype=int)
        self.correct = np.empty(self.nTrials, dtype=int)
        self.dayLength = np.empty(self.nDay, dtype=int)
        self.performance = 0

        # Training Algorithm Parameters
        self.selected_stimulus = None
        self.next_trial_data = {}
        self.stimulus = 0
        self.bias = 0

        # Common functions
        self.common_functions = CommonFunctions()

        # Trial Types [Sound cue]
        self.trial_types = {
            1: ['1KHz'],
            2: ['8KHz']
        }

        # Counts
        self.trial_count = {-1: 0, 1: 0}
        self.left_count = 0
        self.right_count = 0

    # Define actions for clicking each button in the interface
    def _handle_alignMaxTrainingBtn_clicked(self):
        self.alignMax_training = True
        rospy.loginfo("AlignMax training selected")

    def _handle_learnMaxTrainingBtn_clicked(self):
        self.learnMax_training = True
        rospy.loginfo("LearnMax training selected")

    def _handle_pseudorandomTrainingBtn_clicked(self):
        self.pseudorandom_training = True
        rospy.loginfo("Pseudorandom training selected")

    def _handle_preTrainingPhaseOneBtn_clicked(self):
        self.pretraining_phase_one = True
        rospy.loginfo("Pretraining Phase One selected")

    def _handle_preTrainingPhaseTwoBtn_clicked(self):
        self.pretraining_phase_two = True
        rospy.loginfo("Pretraining Phase Two selected")

    def _handle_contTMazeBtn_clicked(self):
        rospy.loginfo("Continuous T Maze selected")
        self.setContTMazeConfig()
        self.setchamberFiveStartConfig()

    def setContTMazeConfig(self):
        # Raise all walls
        for i in range(9):
            for j in range(8):
                self.common_functions.raise_wall(Wall(i, j), False)

        # Lower walls between chambers 7 and 8, 1 and 2, 3 and 4
        for i in [2, 4, 8]:
            # Left and right goal walls remain raised until the first trial starts
            self.common_functions.lower_wall(Wall(i, 0), False)
        self.common_functions.activateWalls()

    def _handle_lowerAllDoorsBtn_clicked(self):
        self.setlowerAllConfig()

    def _handle_lowerDoorBtn_clicked(self):
        self.setlowerConfig()

    # Lower Walls of the chambers that the rat is at risk of getting caught in (i.e. those that are lowered/ raised during the experiment)
    def setlowerConfig(self):
        # Lower Wall 4 in chamber 4 (central chamber);
        if self.rat_head_chamber == 5 or 4:
            self.common_functions.lower_wall(Wall(4, 4), False)

        # Lower Wall 2 in chamber 8 (left return chamber);
        elif self.rat_head_chamber == 8 or 5:
            self.common_functions.lower_wall(Wall(8, 0), False)

        # Lower Wall 6 in chamber 2 (right return chamber);
        elif self.rat_head_chamber == 5 or 2:
            self.common_functions.lower_wall(Wall(2, 6), False)

        # Lower Walls 2, 4 in chamber 6 (left goal chamber); and
        elif self.rat_head_chamber == 3 or 6 or 7:
            for i in [2, 4]:
                self.common_functions.lower_wall(Wall(6, i), False)

        # Lower Walls and 6, 4 in chamber 0 (right goal chamber)
        elif self.rat_head_chamber == 0 or 1 or 3:
            for i in [4, 6]:
                self.common_functions.lower_wall(Wall(0, i), False)
        self.common_functions.activateWalls()

    def setlowerAllConfig(self):
        self.common_functions.lower_wall(Wall(4, 4), False)
        self.common_functions.lower_wall(Wall(2, 6), False)
        self.common_functions.lower_wall(Wall(8, 2), False)
        for i in [2, 4]:
            self.common_functions.lower_wall(Wall(6, i), False)

        for i in [4, 6]:
            self.common_functions.lower_wall(Wall(0, i), False)
        self.common_functions.activateWalls()

    # Define chambers

    def setchamberFiveStartConfig(self):
        self.start_chamber = 5
        self.central_chamber = 4
        self.choice_chamber = 3
        self.left_goal_chamber = 6
        self.right_goal_chamber = 0
        self.left_return_chamber = 8
        self.right_return_chamber = 2

        self.start_wall = Wall(4, 4)
        self.left_goal_entry_wall = Wall(6, 2)
        self.right_goal_entry_wall = Wall(0, 6)
        self.left_goal_exit_wall = Wall(6, 4)
        self.right_goal_exit_wall = Wall(0, 4)
        self.left_return_wall = Wall(8, 2)
        self.right_return_wall = Wall(2, 6)

    # Define messages (i.e. data) received from the gantry
    def rat_head_chamber_callback(self, msg):
        self.rat_head_chamber = msg.data

    def rat_body_chamber_callback(self, msg):
        self.rat_body_chamber = msg.data

    def mode_callback(self, msg):
        mode = msg.data
        if mode == "START_EXPERIMENT":
            self.mode = Mode.START_EXPERIMENT
        elif mode == "PAUSE_EXPERIMENT":
            self.mode_before_pause = self.mode
            self.mode = Mode.PAUSE_EXPERIMENT
        elif mode == "RESUME_EXPERIMENT":
            self.mode = Mode.RESUME_EXPERIMENT

    # def trial_callback(self, msg):
    #    # Convert the string back into a list (if necessary)
    #    trial_data = json.loads(msg.data)
    #    self.currentTrial = trial_data['trial']
    #    self.current_trial_index = trial_data['current_trial_index']
    #    self.trials = trial_data['trials']
    #    self.nTrials = trial_data['nTrials']

        # Log the received trial and index
    #    rospy.loginfo(f"Received selected trial: {self.currentTrial}")
    #    rospy.loginfo(
    #        f"Received current_trial_index: {self.current_trial_index}")

    def run_experiment(self):

        self.current_time = rospy.Time.now()
        current_rat_chamber = self.rat_head_chamber

        if current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1:
            # The rat has moved to a different chamber, update the gantry position
            self.common_functions.move_gantry_to_chamber(current_rat_chamber)

            # Update the previous_rat_chamber for the next iteration
            self.previous_rat_chamber = current_rat_chamber

        if self.mode == Mode.START_EXPERIMENT:
            rospy.loginfo("START OF THE EXPERIMENT")
            if self.pseudorandom_training == True:
                self.nDay = self.nDay + 1
                rospy.loginfo(f"Current session: {self.nDay}")
            self.currentTrialNumber = self.current_trial_index-1
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_IN_START_CHAMBER
            rospy.loginfo("RAT IN START CHAMBER")

        elif self.mode == Mode.RAT_IN_START_CHAMBER:
            self.currentTrialNumber = self.currentTrialNumber+1
            rospy.loginfo(f"Current trial number: {self.currentTrialNumber}")

            if self.trials and 0 <= self.currentTrialNumber < len(self.trials):
                self.currentTrial = self.trials[self.currentTrialNumber]
            else:
                # Handle the case where trials is empty or currentTrialNumber is out of range
                self.currentTrial = None

            # End experiment if there are no more trials from the predefined number of total trials
            if self.currentTrial is not None and self.currentTrialNumber >= self.nTrials:
                self.mode = Mode.END_EXPERIMENT

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.START_TRIAL
            rospy.loginfo(f"START OF TRIAL: {self.currentTrial}")

        elif self.mode == Mode.START_TRIAL:
            if self.pretraining_phase_one == True or self.pretraining_phase_two == True:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START
                rospy.loginfo("START")

            if self.pseudorandom_training == True:
                if self.currentTrial is not None and self.currentTrialNumber == 0:
                    # Generate first trial data randomly
                    new_trial = {
                        # Pseudorandomly select stimulus
                        'stimulus': random.choice([-1, 1]),
                        'trial_number': self.currentTrialNumber,
                        'other_parameters': {}                    # Add other trial-specific data as needed
                    }

                    self.trials.append(new_trial)  # Add to the trials list
                    self.currentTrial = new_trial
                    self.stimulus = new_trial['stimulus']
                    self.trial_count[self.stimulus] += 1

                else:
                    if self.trial_count[-1] < 3 and self.trial_count[1] < 3:
                        self.stimulus = random.choice([-1, 1])
                    elif self.trial_count[-1] >= 3:
                        self.stimulus = 1
                    elif self.trial_count[1] >= 3:
                        self.stimulus = -1
                    else:
                        self.stimulus = random.choice([-1, 1])

                    self.currentTrial = {
                        'stimulus': self.stimulus,
                        'trial_number': self.currentTrialNumber,
                        'other parameters': {}
                    }
                    self.trials.append(self.currentTrial)

                self.stimulus_pub.publish(str(self.stimulus))
                rospy.loginfo(f"Selected stimulus is: {self.stimulus}")

                if self.stimulus == -1:
                    self.sound_cue = '1kHz'
                    self.sound_pub.publish(self.sound_cue)
                    self.answer[self.currentTrialNumber] = 2
                    rospy.loginfo(
                        f"Correct stimulus-response choice is: {self.answer[self.currentTrialNumber]}")
                    self.success_chamber = self.left_goal_chamber
                    self.error_chamber = self.right_goal_chamber
                    self.trial_count[-1] += 1
                    self.trial_count[1] = 0

                elif self.stimulus == 1:
                    self.sound_cue = '8kHz'
                    self.sound_pub.publish(self.sound_cue)
                    self.answer[self.currentTrialNumber] = 1
                    rospy.loginfo(
                        f"Correct stimulus-response choice is: {self.answer[self.currentTrialNumber]}")
                    self.success_chamber = self.right_goal_chamber
                    self.error_chamber = self.left_goal_chamber
                    self.trial_count[1] += 1
                    self.trial_count[-1] = 0

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START
                rospy.loginfo("START")

        elif self.mode == Mode.START:
            if (self.current_time - self.mode_start_time).to_sec() >= self.start_first_delay.to_sec():
                if self.pseudorandom_training == True:
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.SOUND_CUE
                    rospy.loginfo("SOUND_CUE")
                else:
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.START_TO_CHOICE
                    rospy.loginfo("START_TO_CHOICE")

        elif self.mode == Mode.SOUND_CUE:
            if (self.current_time - self.mode_start_time).to_sec() >= self.sound_delay.to_sec():
                # if self.training_mode is not None and self.training_mode in ["choice", "user_defined_forced_choice"]:
                # self.play_sound_cue(self.sound_cue)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TO_CHOICE
                rospy.loginfo("START_TO_CHOICE")

        elif self.mode == Mode.START_TO_CHOICE:
            if self.pretraining_phase_one == True:
                if self.currentTrialNumber == 0:
                    self.common_functions.lower_wall(
                        self.start_wall, send=True)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE
                    rospy.loginfo("CHOICE")
                else:
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE
                    rospy.loginfo("CHOICE")
            else:
                # Lower start wall to let the rat move to the choice point
                self.common_functions.lower_wall(self.start_wall, send=True)
                self.common_functions.lower_wall(
                    self.left_goal_entry_wall, send=True)
                self.common_functions.lower_wall(
                    self.right_goal_entry_wall, send=True)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.CHOICE
                rospy.loginfo("CHOICE")

        elif self.mode == Mode.CHOICE:
            # Raise start wall after he moves into the choice chamber
            if self.rat_body_chamber == self.central_chamber:
                if self.pretraining_phase_one == True:
                    if self.currentTrialNumber == 0:
                        self.common_functions.lower_wall(
                            self.left_return_wall, send=True)
                        self.common_functions.lower_wall(
                            self.right_return_wall, send=True)
                    else:
                        if self.success_chamber == self.left_goal_chamber:
                            self.common_functions.lower_wall(
                                self.right_return_wall, send=True)
                        elif self.success_chamber == self.right_goal_chamber:
                            self.common_functions.lower_wall(
                                self.left_return_wall, send=True)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE_TO_GOAL
                    rospy.loginfo("CHOICE_TO_GOAL")
                else:
                    self.common_functions.raise_wall(
                        self.start_wall, send=True)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE_TO_GOAL
                    rospy.loginfo("CHOICE_TO_GOAL")

        elif self.mode == Mode.CHOICE_TO_GOAL:
            if self.pretraining_phase_one == True or self.pretraining_phase_two == True:
                if self.rat_body_chamber in [self.left_goal_chamber, self.right_goal_chamber]:
                    rospy.loginfo(f"Rat is in: {self.rat_body_chamber}")
                    if self.left_count < 3 and self.right_count < 3:
                        if self.rat_body_chamber == self.left_goal_chamber:
                            self.success_chamber = self.left_goal_chamber
                            self.error_chamber = self.right_goal_chamber

                        elif self.rat_body_chamber == self.right_goal_chamber:
                            self.success_chamber = self.right_goal_chamber
                            self.error_chamber = self.left_goal_chamber

                    elif self.left_count >= 3:
                        self.success_chamber = self.right_goal_chamber
                        self.error_chamber = self.left_goal_chamber

                    elif self.right_count >= 3:
                        self.success_chamber = self.left_goal_chamber
                        self.error_chamber = self.right_goal_chamber

                if self.pretraining_phase_one == True and self.pretraining_phase_two == False:
                    if self.rat_body_chamber in [self.left_goal_chamber, self.right_goal_chamber]:
                        if self.rat_body_chamber == self.success_chamber:
                            if self.success_chamber == self.left_goal_chamber:
                                self.common_functions.raise_wall(
                                    self.left_goal_entry_wall, send=True)
                                self.common_functions.raise_wall(
                                    self.right_return_wall, send=True)
                            elif self.success_chamber == self.right_goal_chamber:
                                self.common_functions.raise_wall(
                                    self.right_goal_entry_wall, send=True)
                                self.common_functions.raise_wall(
                                    self.left_return_wall, send=True)

                            self.mode_start_time = rospy.Time.now()
                            self.mode = Mode.SUCCESS

                        elif self.rat_body_chamber == self.error_chamber:
                            if self.error_chamber == self.left_goal_chamber:
                                self.common_functions.raise_wall(
                                    self.left_goal_entry_wall, send=True)
                                self.common_functions.raise_wall(
                                    self.right_return_wall, send=True)
                            elif self.error_chamber == self.right_goal_chamber:
                                self.common_functions.raise_wall(
                                    self.right_goal_entry_wall, send=True)
                                self.common_functions.raise_wall(
                                    self.left_return_wall, send=True)

                            self.mode_start_time = rospy.Time.now()
                            self.mode = Mode.ERROR

                elif self.pretraining_phase_one == False and self.pretraining_phase_two == True:
                    if self.rat_body_chamber == self.success_chamber:
                        self.common_functions.raise_wall(
                            self.left_goal_entry_wall, send=True)
                        self.common_functions.raise_wall(
                            self.right_goal_entry_wall, send=True)
                        self.mode_start_time = rospy.Time.now()
                        self.mode = Mode.SUCCESS
                        rospy.loginfo("SUCCESS")

                    elif self.rat_body_chamber == self.error_chamber:
                        self.common_functions.raise_wall(
                            self.left_goal_entry_wall, send=True)
                        self.common_functions.raise_wall(
                            self.right_goal_entry_wall, send=True)
                        self.mode_start_time = rospy.Time.now()
                        self.mode = Mode.ERROR
                        rospy.loginfo("ERROR")
            else:

                # If rat moved chose the correct chamber, raise the entry wall of both the left and right goal chambers
                if self.rat_body_chamber == self.success_chamber:
                    self.common_functions.raise_wall(
                        self.left_goal_entry_wall, send=True)
                    self.common_functions.raise_wall(
                        self.right_goal_entry_wall, send=True)
                    # Record that the rat made the correct choice
                    self.correct[self.currentTrialNumber] = 1
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.SUCCESS
                    rospy.loginfo(
                        f"Choice is correct: {self.correct[self.currentTrialNumber]}")
                    rospy.loginfo("SUCCESS")

                # If rat chose the wrong chamber, raise the entry wall of both the left and right return chambers
                elif self.rat_body_chamber == self.error_chamber:
                    self.common_functions.raise_wall(
                        self.left_goal_entry_wall, send=True)
                    self.common_functions.raise_wall(
                        self.right_goal_entry_wall, send=True)
                    # Record that the rat made the wrong choice
                    self.correct[self.currentTrialNumber] = 0
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.ERROR
                    rospy.loginfo(
                        f"Choice is incorrect: {self.correct[self.currentTrialNumber]}")
                    rospy.loginfo("ERROR")

        elif self.mode == Mode.SUCCESS:
            if self.success_chamber == self.left_goal_chamber:
                # Record that the rat chose to turn left
                self.y[self.currentTrialNumber] = 2
                self.left_count += 1
                self.right_count = 0
                if self.pseudorandom_training == True:
                    rospy.loginfo(
                        f"choice (y) is left: {self.y[self.currentTrialNumber]}")
            else:
                # Record that the rat chose to turn right
                self.y[self.currentTrialNumber] = 1
                self.right_count += 1
                self.left_count = 0
                if self.pseudorandom_training == True:
                    rospy.loginfo(
                        f"choice (y) is right: {self.y[self.currentTrialNumber]}")

            self.success_center_x = self.maze_dim.chamber_centers[self.success_chamber][0]
            self.success_center_y = self.maze_dim.chamber_centers[self.success_chamber][1]
            self.gantry_pub.publish("move_to_coordinate", [
                                    self.success_center_x, self.success_center_y])

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.REWARD_START
            rospy.loginfo("REWARD_START")

        elif self.mode == Mode.REWARD_START:
            if (self.current_time - self.mode_start_time).to_sec() >= self.reward_start_delay.to_sec():
                # self.common_functions.reward_dispense()
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_END
                rospy.loginfo("REWARD_END")

        elif self.mode == Mode.REWARD_END:
            if self.pretraining_phase_one == True:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_TO_RETURN
                rospy.loginfo("REWARD_TO_RETURN")
            else:
                if (self.current_time - self.mode_start_time).to_sec() >= self.reward_end_delay.to_sec():
                    # self.gantry_pub.publish("start_harness_tracking", [])
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.REWARD_TO_RETURN
                    rospy.loginfo("REWARD_TO_RETURN")

        elif self.mode == Mode.REWARD_TO_RETURN:
            if self.pseudorandom_training == True:
                if self.success_chamber == self.left_goal_chamber:
                    self.common_functions.lower_wall(
                        self.left_goal_exit_wall, send=True)
                    self.common_functions.lower_wall(
                        self.left_return_wall, send=True)
                elif self.success_chamber == self.right_goal_chamber:
                    self.common_functions.lower_wall(
                        self.right_goal_exit_wall, send=True)
                    self.common_functions.lower_wall(
                        self.right_return_wall, send=True)

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_RETURN_TO_START
                rospy.loginfo("RETURN_TO_START")

            else:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_RETURN_TO_START
                rospy.loginfo("RETURN_TO_START")

        elif self.mode == Mode.REWARD_RETURN_TO_START:
            if self.rat_body_chamber == self.start_chamber:

                if self.pretraining_phase_one == True:
                    if self.success_chamber == self.left_goal_chamber:
                        self.common_functions.lower_wall(
                            self.left_goal_entry_wall, send=True)
                    elif self.success_chamber == self.right_goal_chamber:
                        self.common_functions.lower_wall(
                            self.right_goal_entry_wall, send=True)

                else:
                    rospy.loginfo("RAT IS BACK IN THE START CHAMBER")
                    if self.success_chamber == self.left_goal_chamber:
                        self.common_functions.raise_wall(
                            self.left_goal_exit_wall, send=True)
                        self.common_functions.raise_wall(
                            self.left_return_wall, send=True)

                    elif self.success_chamber == self.right_goal_chamber:
                        self.common_functions.raise_wall(
                            self.right_goal_exit_wall, send=True)
                        self.common_functions.raise_wall(
                            self.right_return_wall, send=True)
                    
                    rospy.loginfo("END_TRIAL")

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.END_TRIAL

        elif self.mode == Mode.ERROR:
            if self.error_chamber == self.left_goal_chamber:
                # Record that the rat chose to turn left
                self.y[self.currentTrialNumber] = 2
                self.left_count += 1
                self.right_count = 0
                if self.pseudorandom_training == True:
                    rospy.loginfo(
                        f"choice (y) is left: {self.y[self.currentTrialNumber]}")
            else:
                # Record that the rat chose to turn right
                self.y[self.currentTrialNumber] = 1
                self.right_count += 1
                self.left_count = 0
                if self.pseudorandom_training == True:
                    rospy.loginfo(
                        f"choice (y) is right: {self.y[self.currentTrialNumber]}")

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.ERROR_START
            rospy.loginfo("ERROR_START")

        elif self.mode == Mode.ERROR_START:
            if self.pretraining_phase_one == True:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR_END
            else:
                if (self.current_time - self.mode_start_time).to_sec() >= self.error_start_delay.to_sec():
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.ERROR_END
                    rospy.loginfo("ERROR_END")

        elif self.mode == Mode.ERROR_END:
            if self.pretraining_phase_one == True:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR_TO_RETURN
            else:
                if (self.current_time - self.mode_start_time).to_sec() >= self.error_end_delay.to_sec():
                    # self.gantry_pub.publish("start_harness_tracking", [])
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.ERROR_TO_RETURN
                    rospy.loginfo("ERROR_TO_RETURN")

        elif self.mode == Mode.ERROR_TO_RETURN:
            if self.pseudorandom_training == True:
                if self.error_chamber == self.left_goal_chamber:
                    self.common_functions.lower_wall(
                        self.left_goal_exit_wall, send=True)
                    self.common_functions.lower_wall(
                        self.left_return_wall, send=True)
                elif self.error_chamber == self.right_goal_chamber:
                    self.common_functions.lower_wall(
                        self.right_goal_exit_wall, send=True)
                    self.common_functions.lower_wall(
                        self.right_return_wall, send=True)

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR_RETURN_TO_START

            else:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR_RETURN_TO_START
                rospy.loginfo("RETURN_TO_START")

        elif self.mode == Mode.ERROR_RETURN_TO_START:
            if self.rat_body_chamber == self.start_chamber:
                if self.pretraining_phase_one == True:
                    if self.error_chamber == self.left_goal_chamber:
                        self.common_functions.lower_wall(
                            self.left_goal_entry_wall, send=True)
                    elif self.error_chamber == self.right_goal_chamber:
                        self.common_functions.lower_wall(
                            self.right_goal_entry_wall, send=True)

                else:
                    rospy.loginfo("RAT IS BACK IN THE START CHAMBER")
                    if self.error_chamber == self.left_goal_chamber:
                        self.common_functions.raise_wall(
                            self.left_goal_exit_wall, send=True)
                        self.common_functions.raise_wall(
                            self.left_return_wall, send=True)

                    elif self.error_chamber == self.right_goal_chamber:
                        self.common_functions.raise_wall(
                            self.right_goal_exit_wall, send=True)
                        self.common_functions.raise_wall(
                            self.right_return_wall, send=True)

                    rospy.loginfo("END_TRIAL")

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.END_TRIAL

        elif self.mode == Mode.END_TRIAL:
            if self.pretraining_phase_one == True:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.INTER_TRIAL_INTERVAL
            else:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.INTER_TRIAL_INTERVAL
                rospy.loginfo("INTER_TRIAL_INTERVAL")

        elif self.mode == Mode.INTER_TRIAL_INTERVAL:
            if self.pretraining_phase_one == True:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.RAT_IN_START_CHAMBER
            else:
                if (self.current_time - self.mode_start_time).to_sec() >= self.inter_trial_interval.to_sec():
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.RAT_IN_START_CHAMBER

        elif self.mode == Mode.PAUSE_EXPERIMENT:
            rospy.loginfo("PAUSE_EXPERIMENT")
            self.button_pub.publish("Pause_button_disabled")
            self.mode_start_time = rospy.Time.now()

        elif self.mode == Mode.RESUME_EXPERIMENT:
            rospy.loginfo("RESUME_EXPERIMENT")
            self.button_pub.publish("Pause_button_enabled")
            self.mode_start_time = rospy.Time.now()
            self.mode = self.mode_before_pause

        elif self.mode == Mode.END_EXPERIMENT:
            self.dayLength[self.nDay] = self.currentTrialNumber
            rospy.loginfo(
                f"Total trials for session {self.nDay} is: {self.dayLength[self.nDay]}")
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.END_EXPERIMENT
            rospy.loginfo("END_EXPERIMENT")


if __name__ == '__main__':
    rospy.init_node('pseudorandom_control')
    Interface()
    rospy.spin()
