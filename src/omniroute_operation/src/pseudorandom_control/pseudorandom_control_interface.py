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
from PyQt5.QtWidgets import QGraphicsScene, QButtonGroup, QFileDialog, QWidget, QApplication
from PyQt5.QtCore import QTimer

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin

import json
from shared_utils.wall_utilities import MazeDimensions
from experiment_controller.experiment_controller_interface import Wall
from experiment_controller.experiment_controller_interface import CommonFunctions

import psytrack as psy
from psytrack.helper.helperFunctions import read_input

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
        self.setObjectName('Dynamic Training Controller Interface')

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
            __file__)), 'dynamic_training_controller_interface.ui')
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
        # Depending on how we design the dynamic controller, we may need more than one control training mode
        self._widget.pseudorandomTrainingBtn.clicked.connect(
            self._handle_pseudorandomTrainingBtn_clicked)
        self._widget.testingPhaseBtn.clicked.connect(
            self._handle_testingPhaseBtn_clicked)
        self._widget.contTMazeBtn.clicked.connect(
            self._handle_contTMazeBtn_clicked)
        self._widget.lowerAllDoorsBtn.clicked.connect(
            self._handle_lowerAllDoorsBtn_clicked)
        # The starting chamber is always the same so no need to define it in the interface

        self.is_testing_phase = False
        self.alignMax_training = False
        self.learnMax_training = False
        self.pseudorandom_training = False

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

        self.experiment_pub.publish("dynamic_training_controller_experiment")

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
        self.mode = Mode.START
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # Time parameters
        self.timer = QTimer(self)
        # self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)

        # Delay parameters (change durations later)
        self.inter_trial_interval = rospy.Duration(1.0)  # time between trials
        # Delay between sound cue and lowering the doors of the start chamber
        self.sound_delay = rospy.Duration(0.5)
        # Duration to wait for rat to move to the choice point
        self.choice_delay = rospy.Duration(1.5)
        # Duration to wait to for the reward to dispense
        self.reward_start_delay = rospy.Duration(13)
        # Duration to wait to dispense reward if the rat made the right choice
        self.reward_end_delay = rospy.Duration(2)
        self.success_delay = rospy.Duration(1)  # Delay after reward ends
        self.error_delay = rospy.Duration(2)  # Delay after error
        self.end_trial_delay = rospy.Duration(
            1)  # Delay after the end of the trial

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

        # Weight Parameters
        self.current_weights = np.random.rand(
            10)  # Replace with wMode[:, -1] later
        self.goal_weights = self.w_U
        # pushing bias to the right
        self.w_R = np.array([2.2, 2.2, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # pushing bias to the left
        self.w_L = np.array([-2.2, 2.2, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.w_U = np.array([0, 2.2, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.delta = 1  # maybe change later

        self.weight_sum_neg1 = np.zeros_like(self.current_weights)
        self.weight_sum_pos1 = np.zeros_like(self.current_weights)

        # Learning Parameters
        self.stage = "early"
        self.early_learning_threshold = 300
        self.performance_threshold = 0.70

        # Input-Response Combinations
        self.next_trial_data = {
            "condition_1": {'y': 2, 'stimulus': -1},
            "condition_2": {'y': 1, 'stimulus': 1},
            "condition_3": {'y': 1, 'stimulus': -1},
            "condition_4": {'y': 2, 'stimulus': 1},
        }

        # Trial Data Parameters
        self.y = np.empty(self.nTrials, dtype=int)
        self.inputs = np.empty(self.nTrials, dtype=int)
        self.name = []
        self.answer = np.empty(self.nTrials, dtype=int)
        self.correct = np.empty(self.nTrials, dtype=int)
        self.dayLength = np.empty(self.nDay, dtype=int)
        self.performance = 0

        self.data = {
            'y':  np.zeros(self.nTrials, dtype=int),
            'inputs':  np.zeros(self.nTrials, dtype=int),
            'answer': np.zeros(self.nTrials, dtype=int),
            'correct': np.zeros(self.nTrials, dtype=int),
            'dayLength': np.zeros(1, dtype=int)
            }


        # PsyTrack Parameters
        self.predicted_weights = {}
        self.weights = {'bias': 1,
                        'stimulus': 1,
                        'stimH': 0,
                        'actionH': self.nPrevTrials,
                        'actionXposRewardH': self.nPrevTrials,
                        'actionXnegRewardH': self.nPrevTrials}

        # It is often useful to have the total number of weights K in your model
        K = np.sum([self.weights[i] for i in self.weights.keys()])

        self.hyper = {'sigInit': 2**4.,      # Set to a single, large value for all weights. Will not be optimized further.
                      # Each weight will have it's own sigma optimized, but all are initialized the same
                      'sigma': [2**-4.]*K,
                      'sigDay': 2**-2.}      # Indicates that session boundaries will be ignored in the optimization

        # The hyperparameters that are fitted
        self.optList = ['sigma', 'sigDay']

        self.g_xt = np.zeros(len(self.weights))
        self.g_xt[0] = 1.0  # Bias term
        self.g_xt[1] = self.stimulus
        self.g_xt[2] = self.stimH
        self.g_xt[3] = self.actionH
        self.g_xt[4] = self.actionXposRewardH
        self.g_xt[5] = self.actionXnegRewardH
        self.gw = 0
        self.pR = 0
        self.pL = 0

        # Training Algorithm Parameters
        self.selected_stimulus = None
        self.next_trial_data = {}
        self.stimulus = 0
        self.goal_weights = np.zeros(len(self.current_weights))
        self.bias = 0
        self.alignments = {}

        # Common functions
        self.common_functions = CommonFunctions()

        # Trial Types [Sound cue]
        self.trial_types = {
            1: ['1KHz'],
            2: ['8KHz']
        }

        self.trial_count = {key: 0 for key in self.trial_types}

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

    def _handle_testingPhaseBtn_clicked(self):
        self.is_testing_phase = True
        rospy.loginfo("Testing phase selected")

    def _handle_contTMazeBtn_clicked(self):
        rospy.loginfo("Continuous T Maze selected")
        self.setContTMazeConfig()
        self.setchamberSevenStartConfig()

    def setContTMazeConfig(self):
        # Raise all walls
        for i in range(9):
            for j in range(8):
                self.common_functions.raise_wall(Wall(i, j), False)

        # Lower walls between chambers 3 and 6, 1 and 4, 5 and 8
        for i in [4, 6, 8]:
            # Left and right goal walls remain raised until the first trial starts
            self.common_functions.lower_wall(Wall(i, 2), False)
        self.common_functions.activateWalls()

    def _handle_lowerAllDoorsBtn_clicked(self):
        self.setlowerConfig()

    # Lower Walls of the chambers that the rat is at risk of getting caught in (i.e. those that are lowered/ raised during the experiment)
    def setlowerConfig(self):
        # Lower Wall 6 in chamber 4 (central chamber);
        if self.rat_head_chamber == 7 or 4:
            self.common_functions.lower_wall(Wall(4, 6), False)
        # Lower Wall 4 in chamber 6 (left return chamber);
        elif self.rat_head_chamber == 6 or 7:
            self.common_functions.lower_wall(Wall(6, 4), False)
        # Lower Wall 0 in chamber 8 (right return chamber);
        elif self.rat_head_chamber == 8 or 7:
            self.common_functions.lower_wall(Wall(8, 0), False)
        # Lower Walls 4, 6 in chamber 0 (left goal chamber); and
        elif self.rat_head_chamber == 0 or 1 or 3:
            for i in [4, 6]:
                self.common_functions.lower_wall(Wall(0, i), False)
        # Lower Walls and 0, 6 in chamber 2 (right goal chamber)
        elif self.rat_head_chamber == 2 or 1 or 5:
            for i in [0, 6]:
                self.common_functions.lower_wall(Wall(2, i), False)
        self.common_functions.activateWalls()

    # Define chambers
    def setchamberSevenStartConfig(self):
        self.start_chamber = 7
        self.central_chaber = 4
        self.choice_chamber = 1
        self.left_goal_chamber = 0
        self.right_goal_chamber = 2
        self.left_return_chamber = 6
        self.right_return_chamber = 8

        self.start_wall = Wall(4, 6)
        self.left_goal_entry_wall = Wall(0, 4)
        self.right_goal_entry_wall = Wall(2, 0)
        self.left_goal_exit_wall = Wall(0, 6)
        self.right_goal_exit_wall = Wall(2, 6)
        self.left_return_wall = Wall(6, 4)
        self.right_return_wall = Wall(8, 0)

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

    def trial_callback(self, msg):
        # Convert the string back into a list (if necessary)
        trial_data = json.loads(msg.data)
        self.currentTrial = trial_data['trial']
        self.current_trial_index = trial_data['current_trial_index']
        self.trials = trial_data['trials']
        self.nTrials = trial_data['nTrials']

        # Log the received trial and index
        rospy.loginfo(f"Received selected trial: {self.currentTrial}")
        rospy.loginfo(
            f"Received current_trial_index: {self.current_trial_index}")

    def run_psytrack(self):
        hyp, evd, wMode, hess_info = psy.hyperOpt(self.data, self.hyper,self.weights, self.optList)

        self.current_weights = wMode[:, -1]  # Last column of weights
        return self.current_weights

    # Function to calculate dynamically adjusted goal weights
    def define_goal_weights(self):
        if self.stage == "early":
            if self.bias > self.delta:
                self.goal_weights = self.w_R
            elif self.bias < -self.delta:
                self.goal_weights = self.w_L
            else:
                self.goal_weights = self.w_U
        elif self.stage == "late":
            self.goal_weights = self.w_U
        else:
            raise ValueError("Invalid stage. Must be 'early' or 'late'.")

        assert self.goal_weights.shape == self.current_weights.shape

    # Function to dynamically update the stage of learning based on training progress and performance
    def update_stage(self):
        if self.currentTrialNumber > self.early_learning_threshold or self.performance >= self.performance_threshold:
            self.stage = "late"
        else:
            self.stage = "early"

    # Function to calculate probabilities of making a right (pR) or left (pL) turn on any given trial n
    def calculate_probabilities(self):
        self.gw = np.dot(self.g_xt)
        self.pR = 1 / (1 + np.exp(-self.gw))
        self.pL = 1 - self.pR

    def calculate_average_weights(self):
        for self.condition_name, self.trial_data in self.next_trial_data.items():

            if self.condition_name == 1:
                self.y[self.currentTrialNumber] = 2
                self.inputs[self.currentTrialNumber] = -1
                self.answer[self.currentTrialNumber] = 2
                self.correct[self.currentTrialNumber] = 1

            elif self.condition_name == 2:
                self.y[self.currentTrialNumber] = 1
                self.inputs[self.currentTrialNumber] = 1
                self.answer[self.currentTrialNumber] = 1
                self.correct[self.currentTrialNumber] = 1

            elif self.condition_name == 3:
                self.y[self.currentTrialNumber] = 1
                self.inputs[self.currentTrialNumber] = -1
                self.answer[self.currentTrialNumber] = 2
                self.correct[self.currentTrialNumber] = 0

            else:
                self.y[self.currentTrialNumber] = 2
                self.inputs[self.currentTrialNumber] = 1
                self.answer[self.currentTrialNumber] = 1
                self.correct[self.currentTrialNumber] = 0

            self.g_xt = np.zeros(len(self.current_weights))
            self.stimulus = self.inputs[self.currentTrialNumber]
            if self.currentTrialNumber >= self.nPrevTrials:
                self.stimH = self.inputs[self.currentTrialNumber - self.nPrevTrials]
                self.actionH = self.y[self.currentTrialNumber - self.nPrevTrials]

                self.prev_reward = self.correct[self.currentTrialNumber - self.nPrevTrials]
                if self.prev_reward == 1:
                    self.actionXposRewardH = self.y[self.currentTrialNumber - self.nPrevTrials]
                else:
                    self.actionXposRewardH = 0
                if self.prev_reward == 0:
                    self.actionXnegRewardH = - self.y[self.currentTrialNumber - self.nPrevTrials]
                else:
                    self.actionXnegRewardH = 0
            else:
                self.stimH = 0
                self.actionH = 0
                self.actionXposRewardH = 0
                self.actionXnegRewardH = 0

            rospy.loginfo(
                f"Input Vector for {self.condition_name}: {self.g_xt}")
            
            _, _, wMode_next, _ = psy.hyperOpt(self.data, self.hyper, self.weights, self.optList)
            

            # Ensure wMode_next is 2D before accessing [:, -1]
            #if wMode_next.ndim == 2:
                # Exclude last column
                #self.predicted_weights[self.condition_name] = wMode_next[:, :-1]
                #rospy.loginfo(f"Predicted weights for {self.condition_name}: {wMode_next[:, -1]}")
            #else:
                #rospy.logerr(f"Unexpected wMode_next shape: {wMode_next.shape}")

            self.predicted_weights[self.condition_name] = wMode_next[: -1]
            rospy.loginfo(
                f"Predicted weights for {self.condition_name}: {wMode_next[:, -1]}")

            self.pR, self.pL = self.calculate_probabilities()
            rospy.loginfo(
                f"Predicted probabilities for {self. condition_name}: pR = {self.pR:.3f}, pL = {self.pL:.3f}")

            if self.stimulus == -1:
                if self.y == 1:
                    self.weight_sum_neg1 += self.pR * self.predicted_weights[self.condition_name]
                elif self.y == 2:
                    self.weight_sum_neg1 += self.pL * self.predicted_weights[self.condition_name]
            elif self.stimulus == 1:
                if self.y == 1:
                    self.weight_sum_pos1 += self.pR * self.predicted_weights[self.condition_name]
                elif self.y == 2:
                    self.weight_sum_pos1 += self.pL * self.predicted_weights[self.condition_name]

        {'stimulus=-1': self.weight_sum_neg1, 'stimulus=1': self.weight_sum_pos1}

        rospy.loginfo(f"Average Weights for Each Stimulus Condition:",
                      self.calculate_average_weights)

    def alignmax_stimulus_selection(self):
        for self.stimulus_type, self.avg_weight in self.calculate_average_weights.items():
            rospy.loginfo(
                f"Average Weight for {self.stimulus_type}: {self.avg_weight}")
            self.expected_weight_diff = self.avg_weight - self.current_weights
            print(
                f"Expected Weight Diff for {self.stimulus_type}: {self.expected_weight_diff}")
            self.current_weight_diff = self.goal_weights - self.current_weights
            print(
                f"Current Weight Diff for {self.stimulus_type}: {self.current_weight_diff}")

            # Dot product measures alignment
            self.alignment = np.dot(
                self.expected_weight_diff, self.current_weight_diff)
            self.alignments[self.stimulus_type] = self.alignment
            print(f"{self.stimulus_type}: Alignment = {self.alignment:.3f}")

        # Determine the stimulus type with the maximum alignment (i.e. minumum distance)
        self.selected_stimulus = min(self.alignments, key=self.alignments.get)
        print(
            f"Selected Stimulus: {self.elected_stimulus} with Alignment: {self.alignments[selected_stimulus]:.3f}")

        # Extract the stimulus value from the key (e.g., 'stimulus=-1' -> -1, 'stimulus=1' -> 1)
        if self.selected_stimulus == 'stimulus=-1':
            return -1
        elif self.selected_stimulus == 'stimulus=1':
            return 1

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
            self.nDay = self.nDay + 1
            self.currentTrialNumber = self.current_trial_index-1

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.START_TRIAL

        elif self.mode == Mode.START_TRIAL:
            self.currentTrialNumber = self.currentTrialNumber+1
            rospy.loginfo(f"Current trial number: {self.currentTrialNumber}")
            if self.trials and 0 <= self.currentTrialNumber < len(self.trials):
                self.currentTrial = self.trials[self.currentTrialNumber]
            else:
                # Handle the case where trials is empty or currentTrialNumber is out of range
                self.currentTrial = None

            rospy.loginfo(f"START OF TRIAL {self.currentTrial}")

            # End experiment if there are no more trials from the predefined number of total trials
            if self.currentTrial is not None and self.currentTrialNumber >= self.nTrials:
                self.mode = Mode.END_EXPERIMENT

            if self.currentTrial is not None:
                if self.alignMax_training:
                    self.current_weights = self.run_psytrack()
                    rospy.loginfo(
                        f"Current weights for {self.name}: {self.current_weights}")
                    self.stimulus = self.alignmax_stimulus_selection(
                        self.calculate_average_weights)
                    rospy.loginfo(
                        f"Selected stimulus is: {self.selected_stimulus}")

                elif self.learnMax_training:
                    self.current_weights = self.run_psytrack()
                    rospy.loginfo(
                        f"Current weights for {self.name}: {self.current_weights}")
                    # Extract current bias directly from current_weights
                    self.bias = self.current_weights[0]
                    rospy.loginfo(f"Current Bias: {self.bias}")
                    self.goal_weights = self.define_goal_weights()
                    self.stimulus = self.alignmax_stimulus_selection(
                        self.calculate_average_weights)
                    rospy.loginfo(
                        f"Selected stimulus is: {self.selected_stimulus}")

                elif self.pseudorandom_training:
                    if self.trial_count[-1] < 3 and self.trial_count[1] < 3:
                        self.stimulus = random.choice([-1, 1])
                    elif self.trial_count[-1] >= 3:
                        self.stimulus = 1
                    elif self.trial_count[1] >= 3:
                        self.stimulus = -1
                    else:
                        self.stimulus = random.choice([-1, 1])

                    self.trial_count[self.stimulus] += 1
                    rospy.loginfo(f"Selected stimulus is: {self.stimulus}")
                    self.publish_stimulus(self.stimulus)

            if self.stimulus == -1:
                self.sound_cue = '1kHz'
                self.sound_pub.publish(self.sound_cue)
                self.answer[self.currentTrialNumber] = 1
                self.success_chamber = self.left_goal_chamber
                self.error_chamber = self.right_goal_chamber

            elif self.stimulus == 1:
                self.sound_cue = '8kHz'
                self.sound_pub.publish(self.sound_cue)
                self.answer[self.currentTrialNumber] = 1
                self.success_chamber = self.right_goal_chamber
                self.error_chamber = self.left_goal_chamber

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_IN_START_CHAMBER
            rospy.loginfo("RAT_IN_START_CHAMBER")

        elif self.mode == Mode.RAT_IN_START_CHAMBER:
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.START
            rospy.loginfo("START")

        elif self.mode == Mode.START:
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.SOUND_CUE
            rospy.loginfo("SOUND_CUE")

        elif self.mode == Mode.SOUND_CUE:
            # add a function for playing and publishing the sound cue based on trial type
            if (self.current_time - self.mode_start_time).to_sec() >= self.sound_delay.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TO_CHOICE
                rospy.loginfo("START_TO_CHOICE")

        elif self.mode == Mode.START_TO_CHOICE:
            # Lower start wall to let the rat move to the choice point
            self.common_functions.lower_wall(self.start_wall, send=True)
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.CHOICE
            rospy.loginfo("CHOICE")

        elif self.mode == Mode.CHOICE:
            if self.rat_body_chamber == self.choice_chamber:
                self.common_functions.raise_wall(self.start_wall, send=False)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.CHOICE_TO_GOAL
                rospy.loginfo("CHOICE_TO_GOAL")

        elif self.mode == Mode.CHOICE_TO_GOAL:
            # If rat moved chose the correct chamber, raise the entry wall of both the left and right goal chambers
            if self.rat_body_chamber == self.success_chamber:
                self.common_functions.raise_wall(
                    self.left_goal_entry_wall, send=False)
                self.common_functions.raise_wall(
                    self.right_goal_entry_wall, send=False)
                self.correct[self.currentTrialNumber] = 1
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.SUCCESS
                rospy.loginfo("SUCCESS")

            # If rat chose the wrong chamber, raise the entry wall of both the left and right return chambers
            elif self.rat_body_chamber == self.error_chamber:
                self.common_functions.raise_wall(
                    self.left_goal_entry_wall, send=False)
                self.common_functions.raise_wall(
                    self.right_goal_entry_wall, send=False)
                self.correct[self.currentTrialNumber] = 0
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR
                rospy.loginfo("ERROR")

        elif self.mode == Mode.SUCCESS:
            if self.success_chamber == self.left_goal_chamber:
                self.y[self.currentTrialNumber] = 2
                rospy.loginfo("Left goal chamber selected")
            else:
                self.y[self.currentTrialNumber] = 1
                rospy.loginfo("Right goal chamber selected")
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
            if (self.current_time - self.mode_start_time).to_sec() >= self.reward_end_delay.to_sec():
                # self.gantry_pub.publish("start_harness_tracking", [])
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.POST_REWARD
                rospy.loginfo("POST_REWARD")

        elif self.mode == Mode.POST_REWARD:
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.REWARD_TO_RETURN
            rospy.loginfo("REWARD_TO_RETURN")

        elif self.mode == Mode.REWARD_TO_RETURN:
            if self.success_chamber == self.left_goal_chamber:
                self.common_functions.lower_wall(
                    self.left_goal_exit_wall, send=False)
                self.common_functions.lower_wall(
                    self.left_return_wall, send=False)
            elif self.success_chamber == self.right_goal_chamber:
                self.common_functions.lower_wall(
                    self.right_goal_exit_wall, send=False)
                self.common_functions.lower_wall(
                    self.right_return_wall, send=False)
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RETURN_TO_START
            rospy.loginfo("RETURN_TO_START")

        elif self.mode == Mode.ERROR:
            if self.error_chamber == self.left_goal_chamber:
                self.y[self.currentTrialNumber] = 2
                rospy.loginfo("Left goal chamber selected")
            else:
                self.y[self.currentTrialNumber] = 1
                rospy.loginfo("Right goal chamber selected")
            if (self.current_time - self.mode_start_time).to_sec() >= self.error_delay.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR_TO_RETURN
                rospy.loginfo("ERROR_TO_RETURN")

        elif self.mode == Mode.ERROR_TO_RETURN:
            if self.rat_error_chamber == self.left_goal_chamber:
                self.common_functions.lower_wall(
                    self.left_goal_exit_wall, send=False)
                self.common_functions.lower_wall(
                    self.left_return_wall, send=False)
            elif self.error_chamber == self.right_goal_chamber:
                self.common_functions.lower_wall(
                    self.right_goal_exit_wall, send=False)
                self.common_functions.lower_wall(
                    self.right_return_wall, send=False)
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RETURN_TO_START
            rospy.loginfo("RETURN_TO_START")

        elif self.mode == Mode.RETURN_TO_START:
            if self.rat_body_chamber == self.start_chamber:
                self.common_functions.raise_wall(self.start_wall, send=False)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TRIAL
                rospy.loginfo("START_TRIAL")

        elif self.mode == Mode.END_TRIAL:
            self.mode = Mode.INTER_TRIAL_INTERVAL
            rospy.loginfo("INTER_TRIAL_INTERVAL")

        elif self.mode == Mode.INTER_TRIAL_INTERVAL:
            if (self.current_time - self.mode_start_time).to_sec() >= self.inter_trial_interval.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.END_TRIAL
                rospy.loginfo("END_TRIAL")

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
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.END_EXPERIMENT
            self.dayLength[self.nDay] = self.currentTrialNumber
            rospy.loginfo("END_EXPERIMENT")


if __name__ == '__main__':
    rospy.init_node('dynamic_training_controller')
    Interface()
    rospy.spin()
