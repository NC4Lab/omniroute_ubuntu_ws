#!/usr/bin/env python

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


class Mode(Enum):
    START = -1
    START_EXPERIMENT = 0
    START_TRIAL = 1
    RAT_IN_START_CHAMBER = 2
    RAT_WAITS = 17
    START_TO_CHOICE = 3
    RAT_IN_CHOICE_CHAMBER = 4
    CHOICE = 5
    CHOICE_TO_GOAL = 6
    SUCCESS = 7
    REWARD_START = 8
    REWARD_END = 9
    POST_REWARD = 10
    ERROR = 11
    END_TRIAL = 12
    END_EXPERIMENT = 13
    PAUSE_EXPERIMENT = 14
    RESUME_EXPERIMENT = 15
    ERROR_END = 16
    ERROR_START = 18
    MOVE_TO_START_CHAMBER = 19
    RAT_BACK_IN_START_CHAMBER = 20


class Interface(Plugin):
    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None
        self.setObjectName('Single T maze Experiment Interface')

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
        ui_file = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'single_T_maze_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        rospy.loginfo('Test Interface started')

        self._widget.setObjectName('SingleTmazeInterfacePluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self.scene = QGraphicsScene()

        # Set the window to a fixed size
        UIUtilities.set_fixed_size(self._widget)

        # Move the window
        UIUtilities.move_ui_window(
            self._widget, horizontal_alignment='right', vertical_alignment='top')

        self._widget.testingPhaseBtn.clicked.connect(
            self._handle_testingPhaseBtn_clicked)
        self._widget.trialGeneratorBtn.clicked.connect(
            self._handle_trialGeneratorBtn_clicked)
        # Button for designating if rewards should be despensed from the gantry
        self._widget.plusMazeBtn.clicked.connect(
            self._handle_plusMazeBtn_clicked)
        self._widget.lowerAllDoorsBtn.clicked.connect(
            self._handle_lowerAllDoorsBtn_clicked)

        self._widget.startChamberBtnGroup = QButtonGroup()
        self._widget.startChamberBtnGroup.addButton(
            self._widget.chamberOneBtn, id=1)
        self._widget.startChamberBtnGroup.addButton(
            self._widget.chamberThreeBtn, id=3)
        self._widget.startChamberBtnGroup.addButton(
            self._widget.chamberFiveBtn, id=5)
        self._widget.startChamberBtnGroup.addButton(
            self._widget.chamberSevenBtn, id=7)
        self._widget.startChamberBtnGroup.setExclusive(True)
        for button in self._widget.startChamberBtnGroup.buttons():
            button.setEnabled(True)

        self._widget.startChamberBtnGroup.buttonClicked.connect(
            self._handle_startChamberBtnGroup_clicked)
        
        self._widget.phaseOneBtn.clicked.connect(self._handle_phaseOneBtn_clicked)

        self._widget.phaseTwoBtn.clicked.connect(self._handle_phaseTwoBtn_clicked)

        self._widget.phaseThreeBtn.clicked.connect(self._handle_phaseThreeBtn_clicked)

        self.is_testing_phase = False
        self.trial_generator = False
        self.phase_one = False
        self.phase_two = False
        self.phase_three = False

        self.maze_dim = MazeDimensions()

        # self.trial_dir = '/media/big_gulp/nc4_rat_data/Maze_Rats'

        # self.rat = 6
        # self.date = '240829'

        # self.rat_folder = os.path.join(self.trial_dir, 'NC4%04d' % self.rat)

        # if '-' in self.date:
        #     self.date = parsedate(self.date).strftime('%y%m%d')

        # date_folder = os.path.join(self.rat_folder, self.date)

        # self.trial_summary_path = os.path.join(date_folder, 'Past_seven_days_biases.csv')

        # self.df = pd.read_csv(self.trial_summary_path)

        self._widget.lowerAllDoorsBtn.setStyleSheet(
            "background-color: red; color: yellow")

        self.sound_pub = rospy.Publisher('sound_cmd', String, queue_size=1)

        self.projection_pub = rospy.Publisher(
            'projection_walls', String, queue_size=100)
        self.projection_floor_pub = rospy.Publisher(
            'projection_image_floor_num', Int32, queue_size=100)
        self.projection_wall_img_pub = rospy.Publisher(
            'projection_image_wall_num', Int32, queue_size=1)
        self.gantry_pub = rospy.Publisher(
            '/gantry_cmd', GantryCmd, queue_size=1)
        self.write_sync_ease_pub = rospy.Publisher(
            '/Esmacat_write_sync_ease', ease_registers, queue_size=1)
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)
        self.trial_sub = rospy.Subscriber(
            '/selected_trial', String, self.trial_callback)

        # rospy.Subscriber('/selected_chamber', String,self.chamber_callback, queue_size=1)

        rospy.Subscriber('/mode', String, self.mode_callback, queue_size=1)
        self.button_pub = rospy.Publisher('/button', String, queue_size=1)

        rospy.Subscriber('/rat_head_chamber', Int8,
                         self.rat_head_chamber_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/rat_body_chamber', Int8,
                         self.rat_body_chamber_callback, queue_size=1, tcp_nodelay=True)

        self.experiment_pub = rospy.Publisher(
            '/experiment', String, queue_size=1)

        self.rat_head_chamber = -1
        self.rat_body_chamber = -1

        # Time for setting up publishers and subscribers
        rospy.sleep(1.0)

        self.experiment_pub.publish("single_T_maze_experiment")

        # Experiment parameters
        self.delay = rospy.Duration(3.0)  # Duration of delay between each loop
        # Duration of delay in the beginning of the trial
        self.start_first_delay = rospy.Duration(5.0)
        # Duration of delay in the beginning of the trial
        self.start_second_delay = rospy.Duration(1.0)
        # Duration to wait for rat to move to the choice point
        self.choice_delay = rospy.Duration(1.5)
        # Duration to wait to dispense reward if the rat made the right choice
        self.reward_start_delay = rospy.Duration(5)
        # Duration to wait to for the reward to despense
        self.reward_end_delay = rospy.Duration(2)
        # Duration to wait if the rat made the right choice
        self.right_choice_delay = rospy.Duration(5)
        # Duration to wait if the rat made the wrong choice
        self.wrong_choice_delay = rospy.Duration(1)
        # Duration to wait if the rat made the wrong choice
        self.wrong_choice_first_delay = rospy.Duration(34.0)
        self.wrong_choice_second_delay = rospy.Duration(5.0)
        # Duration to wait if the rat made the wrong choice
        self.wrong_choice_delay = rospy.Duration(40)
        self.moving_back_to_start_chamber_delay = rospy.Duration(
            1)  # Duration to wait if the rat made the wrong choice
        # Duration to wait at the end of the trial
        self.end_trial_delay = rospy.Duration(1.0)

        self.mode = Mode.START
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        self.currentTrial = []
        self.currentTrialNumber = 0
        self.nTrials = 0
        self.trials = []

        self.currentStartConfig = 0

        self.current_file_index = 0
        self.current_trial_index = 0

        self.training_mode = None
        self.previous_rat_chamber = -1

        self.wallStates = WallState()
        self.wallStates.state = None

        self.project_left_cue_triangle = 0
        self.project_right_cue_triangle = 0

        self.success_chamber = 0
        self.error_chamber = 0
        self.start_wall = Wall(0, 0)
        self.central_chamber = 0
        self.start_chamber = 0
        self.rat_choice_chamber = 0

        self.left_goal_wall = Wall(0, 0)
        self.right_goal_wall = Wall(0, 0)

        self.floor_img_black_num = 0
        self.floor_img_green_num = 1
        self.wall_img_triangle_num = 3
        self.wall_img_black_num = 0

        self.starting_config = 0

        self.cued_chamber = 0
        self.previous_cued_chamber = 0

        self.switch_trial = None

        # self.project_floor_img = Wall(9, 0).to_dict()

        self.maze_dim = MazeDimensions()
        self.common_functions = CommonFunctions()  # Create an instance

        self.chamber_walls_list = {1:
                                   [Wall(1, 0).to_dict(),
                                    Wall(1, 1).to_dict(),
                                    Wall(1, 2).to_dict(),
                                    Wall(1, 3).to_dict(),
                                    Wall(1, 4).to_dict(),
                                    Wall(1, 5).to_dict(),
                                    Wall(1, 6).to_dict(),
                                    Wall(1, 7).to_dict(),
                                    Wall(4, 1).to_dict(),
                                    Wall(4, 3).to_dict()],
                                3: [Wall(3, 0).to_dict(),
                                    Wall(3, 1).to_dict(),
                                    Wall(3, 2).to_dict(),
                                    Wall(3, 3).to_dict(),
                                    Wall(3, 4).to_dict(),
                                    Wall(3, 5).to_dict(),
                                    Wall(3, 6).to_dict(),
                                    Wall(3, 7).to_dict(),
                                    Wall(4, 1).to_dict(),
                                    Wall(4, 7).to_dict()],
                                5: [Wall(5, 0).to_dict(),
                                    Wall(5, 1).to_dict(),
                                    Wall(5, 2).to_dict(),
                                    Wall(5, 3).to_dict(),
                                    Wall(5, 4).to_dict(),
                                    Wall(5, 5).to_dict(),
                                    Wall(5, 6).to_dict(),
                                    Wall(5, 7).to_dict(),
                                    Wall(4, 3).to_dict(),
                                    Wall(4, 5).to_dict()],
                                7: [Wall(7, 0).to_dict(),
                                    Wall(7, 1).to_dict(),
                                    Wall(7, 2).to_dict(),
                                    Wall(7, 3).to_dict(),
                                    Wall(7, 4).to_dict(),
                                    Wall(7, 5).to_dict(),
                                    Wall(7, 6).to_dict(),
                                    Wall(7, 7).to_dict(),
                                    Wall(4, 5).to_dict(),
                                    Wall(4, 7).to_dict()]
                                   }

        # Trial Types: ['Start Chamber', 'Left Cue', 'Right Cue', 'Sound Cue']
        self.trial_types = {
            1: ['5', 'Triangle', 'No_Cue', 'Black'],
            2: ['5', 'No_Cue', 'Triangle', 'Black'],
            3: ['5', 'Triangle', 'No_Cue', 'Green'],
            4: ['5', 'No_Cue', 'Triangle', 'Green']
        }

        self.trial_type_success_count = {key: 0 for key in self.trial_types}
        self.success_count = 0
        self.previous_trial_result = None
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)
        

    def number_of_correct_trials_types(self, dict, group):
        if group == 'group1':
            sum_success = sum([dict[key] for key in dict if key in [3, 4]])
    
        else:
            sum_success = sum([dict[key] for key in dict if key in [1, 2]])
          
        #if sum_success > 0 and sum_success % 10 == 0:
        if sum_success >= 6:
            if group == 'group1':
               dict[3] = 0
               dict[4] = 0
            else:
               dict[1] = 0
               dict[2] = 0
            return True
        else:
            return False
        
    def pick_trial_phase_one(self):
        # Initialize current_group if it doesn't exist yet
        if not hasattr(self, 'current_group'):
            self.current_group = 'group1'

        if self.number_of_correct_trials_types(self.trial_type_success_count, self.current_group):
            # Switch to the other group
            if self.current_group == 'group1':
                self.current_group = 'group2'
            else:
                self.current_group = 'group1'

            print(f"Switching to {self.current_group}")

        if self.current_group == 'group1':
            # Randomly select a trial from trial types with keys 3 and 4
            trial_key = random.choice([3, 4])
        else:
            # Randomly select a trial from trial types with keys 1 and 2
            trial_key = random.choice([1, 2])

        trial = self.trial_types[trial_key]
        print(f"Selected trial: {trial} from key {trial_key}")

        return trial, trial_key
    
    def pick_trial_phase_two(self):
        trial_key = random.choice([1, 2, 3, 4])
        trial = self.trial_types[trial_key]
        print(f"Selected trial: {trial} from key {trial_key}")
        return trial, trial_key
    

    def _handle_testingPhaseBtn_clicked(self):
        self.is_testing_phase = True
        rospy.loginfo("Testing phase selected")

    def is_recording_on(self):
        list_cmd = subprocess.Popen(
            "rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        ret = 0
        for str in list_output.decode().split("\n"):
            if (str.startswith("/record")):
                ret = 1
        return ret

    def terminate_ros_node(self, s):
        list_cmd = subprocess.Popen(
            "rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.decode().split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def _handle_trialGeneratorBtn_clicked(self):
        self.trial_generator = True
        rospy.loginfo("Trial Generator enabled")

    def _handle_phaseOneBtn_clicked(self):
        self.phase_one = True
        rospy.loginfo("Phase One selected")

    def _handle_phaseTwoBtn_clicked(self):
        self.phase_two = True
        rospy.loginfo("Phase Two selected")

    def _handle_phaseThreeBtn_clicked(self):
        self.phase_three = True
        rospy.loginfo("Phase Three selected")

    def _handle_lowerAllDoorsBtn_clicked(self):
        self.setLowerConfig()

    def setLowerConfig(self):
        # Lower Walls 0,2,4,6 in chamber 4 (central chamber)
        for i in [0, 2, 4, 6]:
            self.common_functions.lower_wall(Wall(4, i), False)
        self.common_functions.activateWalls()

    # def _handle_plusMazeBtn_clicked(self):
    #     self.getPlusConfig()

    # def getPlusConfig(self):
    #     # Access shared state variable
    #     wall_cfg_dir_default = rospy.get_param(
    #         '/shared_state/wall_cfg_dir_default')
    #     # Load the wall configuration file
    #     WallConfig.load_from_csv(wall_cfg_dir_default, 'p_maze.csv')
    #     # Send command to run walls
    #     self.common_functions.activateWalls()

    def _handle_plusMazeBtn_clicked(self):
        self.setPlusConfig()

    def setPlusConfig(self):
        # Lower all walls
        # for i in range(9):
        #     for j in range(8):
        #         self.common_functions.lower_wall(Wall(i, j), False)

        # for i in [1, 3, 4, 5, 7]:
        #     for j in range(8):
        #         self.common_functions.raise_wall(Wall(i, j), False)
        for i in range(9):
            for j in range(8):
                self.common_functions.raise_wall(Wall(i, j), False)

        self.common_functions.activateWalls()

    def _handle_startChamberBtnGroup_clicked(self):
        if self._widget.startChamberBtnGroup.checkedId() == 1:
            self.setChamberOneStartConfig()
            rospy.loginfo("Chamber 1 selected")
        elif self._widget.startChamberBtnGroup.checkedId() == 3:
            self.setChamberThreeStartConfig()
            rospy.loginfo("Chamber 3 selected")
        elif self._widget.startChamberBtnGroup.checkedId() == 5:
            self.setChamberFiveStartConfig()
            rospy.loginfo("Chamber 5 selected")
        elif self._widget.startChamberBtnGroup.checkedId() == 7:
            self.setChamberSevenStartConfig()
            rospy.loginfo("Chamber 7 selected")

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

    def rat_head_chamber_callback(self, msg):
        self.rat_head_chamber = msg.data

    def rat_body_chamber_callback(self, msg):
        self.rat_body_chamber = msg.data

    def setChamberOneStartConfig(self):
        self.start_chamber = 1
        self.central_chamber = 4
        self.left_chamber = 5
        self.right_chamber = 3

        self.left_walls = [Wall(5, 0).to_dict(),
                           Wall(5, 1).to_dict(),
                           Wall(2, 6).to_dict(),
                           Wall(5, 3).to_dict(),
                           Wall(5, 4).to_dict(),
                           Wall(5, 5).to_dict(),
                           Wall(8, 2).to_dict(),
                           Wall(5, 7).to_dict(),
                           Wall(4, 5).to_dict()]

        self.right_walls = [Wall(3, 0).to_dict(),
                            Wall(3, 1).to_dict(),
                            Wall(0, 6).to_dict(),
                            Wall(3, 3).to_dict(),
                            Wall(3, 4).to_dict(),
                            Wall(3, 5).to_dict(),
                            Wall(6, 2).to_dict(),
                            Wall(3, 7).to_dict(),
                            Wall(4, 7).to_dict()]

        self.start_wall = Wall(1, 6)
        self.left_goal_wall = Wall(4, 4)
        self.right_goal_wall = Wall(4, 0)
        self.right_exit_wall = Wall(0, 6)
        self.left_exit_wall = Wall(2, 6)
        self.right_start_chamber_enter_wall = Wall(0, 4)
        self.left_start_chamber_enter_wall = Wall(2, 0)

    def setChamberThreeStartConfig(self):
        self.start_chamber = 3
        self.central_chamber = 4
        self.left_chamber = 1
        self.right_chamber = 7

        self.left_walls = [Wall(0, 4).to_dict(),
                           Wall(1, 1).to_dict(),
                           Wall(1, 2).to_dict(),
                           Wall(1, 3).to_dict(),
                           Wall(1, 4).to_dict(),
                           Wall(1, 5).to_dict(),
                           Wall(1, 6).to_dict(),
                           Wall(1, 7).to_dict(),
                           Wall(4, 3).to_dict()]

        self.right_walls = [Wall(6, 4).to_dict(),
                            Wall(7, 1).to_dict(),
                            Wall(7, 2).to_dict(),
                            Wall(7, 3).to_dict(),
                            Wall(8, 0).to_dict(),
                            Wall(7, 5).to_dict(),
                            Wall(7, 6).to_dict(),
                            Wall(7, 7).to_dict(),
                            Wall(4, 5).to_dict()]

        self.start_wall = Wall(3, 4)
        self.left_goal_wall = Wall(4, 2)
        self.right_goal_wall = Wall(4, 6)
        self.right_exit_wall = Wall(6, 4)
        self.left_exit_wall = Wall(0, 4)
        self.right_start_chamber_enter_wall = Wall(6, 2)
        self.left_start_chamber_enter_wall = Wall(0, 6)

    def setChamberFiveStartConfig(self):
        self.start_chamber = 5
        self.central_chamber = 4
        self.left_chamber = 7
        self.right_chamber = 1

        self.left_walls = [Wall(6, 4).to_dict(),
                           Wall(7, 1).to_dict(),
                           Wall(7, 2).to_dict(),
                           Wall(7, 3).to_dict(),
                           Wall(8, 0).to_dict(),
                           Wall(7, 5).to_dict(),
                           Wall(7, 6).to_dict(),
                           Wall(7, 7).to_dict(),
                           Wall(4, 7).to_dict()]

        self.right_walls = [Wall(0, 4).to_dict(),
                            Wall(1, 1).to_dict(),
                            Wall(1, 2).to_dict(),
                            Wall(1, 3).to_dict(),
                            Wall(1, 4).to_dict(),
                            Wall(1, 5).to_dict(),
                            Wall(1, 6).to_dict(),
                            Wall(1, 7).to_dict(),
                            Wall(4, 1).to_dict()]


        self.start_wall = Wall(5, 0)
        self.left_goal_wall = Wall(4, 6)
        self.right_goal_wall = Wall(4, 2)
        self.right_exit_wall = Wall(2, 0)
        self.left_exit_wall = Wall(8, 0)
        self.right_start_chamber_enter_wall = Wall(2, 6)
        self.left_start_chamber_enter_wall = Wall(8, 2)

    def setChamberSevenStartConfig(self):
        self.start_chamber = 7
        self.central_chamber = 4
        self.left_chamber = 3
        self.right_chamber = 5

        self.left_walls = [Wall(3, 0).to_dict(),
                           Wall(3, 1).to_dict(),
                           Wall(0, 6).to_dict(),
                           Wall(3, 3).to_dict(),
                           Wall(3, 4).to_dict(),
                           Wall(3, 5).to_dict(),
                           Wall(6, 2).to_dict(),
                           Wall(3, 7).to_dict(),
                           Wall(4, 1).to_dict()]

        self.right_walls = [Wall(5, 0).to_dict(),
                            Wall(5, 1).to_dict(),
                            Wall(2, 6).to_dict(),
                            Wall(5, 3).to_dict(),
                            Wall(5, 4).to_dict(),
                            Wall(5, 5).to_dict(),
                            Wall(8, 2).to_dict(),
                            Wall(5, 7).to_dict(),
                            Wall(4, 3).to_dict()]

        self.start_wall = Wall(7, 2)
        self.left_goal_wall = Wall(4, 0)
        self.right_goal_wall = Wall(4, 4)
        self.right_exit_wall = Wall(8, 2)
        self.left_exit_wall = Wall(6, 2)
        self.right_start_chamber_enter_wall = Wall(8, 0)
        self.left_start_chamber_enter_wall = Wall(6, 4)

    def publish_walls(self, previous_cued_chamber, chamber_walls_list):
        if previous_cued_chamber in chamber_walls_list:
            # Get the list of objects associated with the key
            object_list = chamber_walls_list[previous_cued_chamber]

            # Publish each object in the list
            for obj in object_list:
                # Assuming obj can be serialized with json.dumps
                self.projection_pub.publish(json.dumps(obj))
                rospy.loginfo(f"Published: {obj}")
        else:
            rospy.logwarn(
                f"Key {previous_cued_chamber} not found in the dictionary.")
            
    def choose_start_config(self, start_chamber_ID):
        if start_chamber_ID == 1:
            self.setChamberOneStartConfig()
            rospy.loginfo("Chamber 1 selected")
        elif start_chamber_ID == 3:
            self.setChamberThreeStartConfig()
            rospy.loginfo("Chamber 3 selected")
        elif start_chamber_ID == 5:
            self.setChamberFiveStartConfig()
            rospy.loginfo("Chamber 5 selected")
        elif start_chamber_ID == 7:
            self.setChamberSevenStartConfig()
            rospy.loginfo("Chamber 7 selected")
        else:
            rospy.logwarn(f"Invalid start chamber ID: {start_chamber_ID}")


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

            self.currentStartConfig = self._widget.startChamberBtnGroup.checkedId()

            self.currentTrialNumber = self.current_trial_index-1

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.START_TRIAL

        elif self.mode == Mode.START_TRIAL:
            # publish the images to be projected on the walls
            self.currentTrialNumber = self.currentTrialNumber+1
            rospy.loginfo(f"Current trial number: {self.currentTrialNumber}")
            #if self.trial_generator:
            if self.phase_one:
                trial, trial_type_key = self.pick_trial_phase_one()
                self.left_visual_cue = trial[1]
                self.right_visual_cue = trial[2]
                self.floor_cue = trial[3]
                self.trail_type_key = trial_type_key
                rospy.loginfo(f"START OF TRIAL {[self.left_visual_cue, self.right_visual_cue, self.floor_cue]}")
                
            elif self.phase_two or self.phase_three:
                trial, trial_type_key = self.pick_trial_phase_two()
                self.left_visual_cue = trial[1]
                self.right_visual_cue = trial[2]
                self.floor_cue = trial[3]
                self.trail_type_key = trial_type_key
                rospy.loginfo(f"START OF TRIAL {[self.left_visual_cue, self.right_visual_cue, self.floor_cue]}")

            else:

                if self.trials and 0 <= self.currentTrialNumber < len(self.trials):
                    self.currentTrial = self.trials[self.currentTrialNumber]
                else:
                    # Handle the case where trials is empty or currentTrialNumber is out of range
                    self.currentTrial = None

                # self.projection_wall_img_pub.publish(self.wall_img_num)

                rospy.loginfo(f"START OF TRIAL {self.currentTrial}")

                if self.currentTrial is not None and self.currentTrialNumber >= self.nTrials:
                    self.mode = Mode.END_EXPERIMENT

                if self.currentTrial is not None:
                    # Set training mode from file if the automatic mode is selected
                    self.training_mode = self.currentTrial[3]
                    self.left_visual_cue = self.currentTrial[0]
                    self.right_visual_cue = self.currentTrial[1]
                    self.floor_cue = self.currentTrial[2]
            

            self.sound_pub.publish("Starting_Sound")
            rospy.loginfo("Starting sound played")


            if self.floor_cue == "Green":
                self.projection_floor_pub.publish(self.floor_img_green_num)
                if self.left_visual_cue == "Triangle":
                    self.cued_chamber = self.left_chamber
                    self.projection_wall_img_pub.publish(self.wall_img_triangle_num)
                    for i in self.left_walls:
                        self.projection_pub.publish(json.dumps(i))
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.left_chamber
                    self.error_chamber = self.right_chamber
                else:
                    self.cued_chamber = self.right_chamber
                    self.projection_wall_img_pub.publish(
                        self.wall_img_triangle_num)
                    for i in self.right_walls:
                        self.projection_pub.publish(json.dumps(i))
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.right_chamber
                    self.error_chamber = self.left_chamber
            else:
                if self.left_visual_cue == "No_Cue":
                    self.cued_chamber = self.right_chamber
                    self.projection_wall_img_pub.publish(self.wall_img_triangle_num)
                    for i in self.right_walls:
                        self.projection_pub.publish(json.dumps(i))
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.left_chamber
                    self.error_chamber = self.right_chamber
                else:
                    self.cued_chamber = self.left_chamber
                    self.projection_wall_img_pub.publish(self.wall_img_triangle_num)
                    for i in self.left_walls:
                        self.projection_pub.publish(json.dumps(i))
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.right_chamber
                    self.error_chamber = self.left_chamber

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_WAITS
            rospy.loginfo("RAT_WAITS")

        elif self.mode == Mode.RAT_WAITS:
            if (self.current_time - self.mode_start_time).to_sec() >= self.start_first_delay.to_sec():
                self.projection_floor_pub.publish(self.floor_img_black_num)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.RAT_IN_START_CHAMBER
                rospy.loginfo("RAT_IN_START_CHAMBER")

        # AWL Wait for walls to be initialized
        if not rospy.get_param('/shared_state/is_maze_initialized'):
            return

        elif self.mode == Mode.RAT_IN_START_CHAMBER:

            if (self.current_time - self.mode_start_time).to_sec() >= self.delay.to_sec():
                if not self.phase_one and not self.phase_two and not self.phase_three:
                    if self.training_mode is not None and self.training_mode in ["forced_choice", "user_defined_forced_choice"]:
                        if self.success_chamber == self.left_chamber:
                            self.common_functions.lower_wall(
                                self.left_goal_wall, send=True)
                            rospy.loginfo("Lowering left goal wall")
                        else:
                            self.common_functions.lower_wall(
                                self.right_goal_wall, send=True)
                            rospy.loginfo("Lowering right goal wall")
                    elif self.training_mode is not None and self.training_mode in ["choice", "user_defined_choice"]:
                        self.common_functions.lower_wall(
                            self.left_goal_wall, send=False)
                        self.common_functions.lower_wall(
                            self.right_goal_wall, send=True)
                else:
                    self.common_functions.lower_wall(
                        self.left_goal_wall, send=False)
                    self.common_functions.lower_wall(
                        self.right_goal_wall, send=True)

                self.mode = Mode.START
                rospy.loginfo("START")

        elif self.mode == Mode.START:
            if (self.current_time - self.mode_start_time).to_sec() >= self.start_second_delay.to_sec():
                self.common_functions.lower_wall(self.start_wall, send=True)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TO_CHOICE
                rospy.loginfo("START_TO_CHOICE")

        elif self.mode == Mode.START_TO_CHOICE:
            # Wait for the rat to move to the choice point
            if self.rat_head_chamber == self.central_chamber:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.RAT_IN_CHOICE_CHAMBER
                rospy.loginfo("RAT_IN_CHOICE_CHAMBER")

        elif self.mode == Mode.RAT_IN_CHOICE_CHAMBER:
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.CHOICE
            rospy.loginfo("CHOICE")

        elif self.mode == Mode.CHOICE:
            #if (self.current_time - self.mode_start_time).to_sec() >= self.choice_delay.to_sec():
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.CHOICE_TO_GOAL
            rospy.loginfo("CHOICE TO GOAL")

        elif self.mode == Mode.CHOICE_TO_GOAL:
            if self.rat_body_chamber == self.success_chamber:
                self.common_functions.raise_wall(
                    self.left_goal_wall, send=False)
                self.common_functions.raise_wall(
                    self.right_goal_wall, send=False)
                self.common_functions.raise_wall(self.start_wall, send=True)
                if self.phase_one:
                    self.success_count += 1

                    # Update trial success count based on conditions
                    if self.success_count == 1 or (self.success_count > 1 and self.previous_trial_result == "Success"):
                        self.trial_type_success_count[self.trail_type_key] += 1
                        self.previous_trial_result = "Success"
                elif self.phase_two:
                    self.trial_type_success_count[self.trail_type_key] += 1

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.SUCCESS
                rospy.loginfo("SUCCESS")

            elif self.rat_body_chamber == self.error_chamber:
                self.common_functions.raise_wall(
                    self.left_goal_wall, send=False)
                self.common_functions.raise_wall(
                    self.right_goal_wall, send=False)
                self.common_functions.raise_wall(self.start_wall, send=True)
                if self.phase_one: 
                    self.success_count = 0
                    for key in self.trial_type_success_count:
                        self.trial_type_success_count[key] = 0
                    self.previous_trial_result = "Error"
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR
                rospy.loginfo("ERROR")

        elif self.mode == Mode.SUCCESS:
            if self.success_chamber == self.left_chamber:
                rospy.loginfo("Left chamber selected and chamber number is {}".format(
                    self.success_chamber))
            else:
                rospy.loginfo("Right chamber selected and chamber number is {}".format(
                    self.success_chamber))
            self.success_center_x = self.maze_dim.chamber_centers[self.success_chamber][0]
            self.success_center_y = self.maze_dim.chamber_centers[self.success_chamber][1]
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.REWARD_START
            rospy.loginfo("REWARD_START")

        elif self.mode == Mode.REWARD_START:
            if (self.current_time - self.mode_start_time).to_sec() >= self.reward_start_delay.to_sec():
                # self.common_functions.reward_dispense()
                self.previous_cued_chamber = self.cued_chamber
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_END
                rospy.loginfo("REWARD END")

        elif self.mode == Mode.REWARD_END:
            if (self.current_time - self.mode_start_time).to_sec() >= self.reward_end_delay.to_sec():
                # self.gantry_pub.publish("start_rat_tracking", [])
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.POST_REWARD
                rospy.loginfo("POST REWARD")

        elif self.mode == Mode.POST_REWARD:
            if (self.current_time - self.mode_start_time).to_sec() >= self.right_choice_delay.to_sec():
                if self.phase_three:
                    if self.currentTrialNumber < 9:
                        # Before trial 10, do not switch even if self.success_chamber == self.left_chamber
                        print(f"Trial {self.currentTrialNumber}: No switching before trial 10.")
                        self.choose_start_config(self.start_chamber)  

                    elif self.currentTrialNumber == 9 and self.success_chamber == self.left_chamber:
                        # First switch happens at trial 10
                        self.switch_trial = self.currentTrialNumber
                        self.starting_config = self.success_chamber
                        self.choose_start_config(self.starting_config)
                        print(f"First switch at trial {self.currentTrialNumber}")

                    elif self.success_chamber == self.left_chamber:
                         # Switch only if at least 2 trials have passed since the last switch
                        if self.switch_trial is None or self.currentTrialNumber >= self.switch_trial + 10:
                            self.switch_trial = self.currentTrialNumber
                            self.starting_config = self.success_chamber
                            self.choose_start_config(self.starting_config)
                            print(f"Switch at trial {self.currentTrialNumber}")
                        else:
                            self.choose_start_config(self.start_chamber)
                    else:
                        self.choose_start_config(self.start_chamber)
                else:
                    self.setChamberFiveStartConfig()
                    rospy.loginfo("Chamber 5 selected")

                self.projection_wall_img_pub.publish(self.wall_img_black_num)
                self.publish_walls(self.previous_cued_chamber, self.chamber_walls_list)                

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.MOVE_TO_START_CHAMBER
                rospy.loginfo("MOVE_TO_START_CHAMBER")

        elif self.mode == Mode.ERROR:
            # if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_delay.to_sec():
            self.sound_pub.publish("Error")
            rospy.loginfo("Error sound played")
            if self.error_chamber == self.left_chamber:
                rospy.loginfo(
                    "Left chamber selected and chamber number is {}".format(self.error_chamber))
            else:
                rospy.loginfo(
                    "Right chamber selected and chamber number is {}".format(self.error_chamber))
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.ERROR_START
            rospy.loginfo("ERROR_START")

        elif self.mode == Mode.ERROR_START:
            if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_first_delay.to_sec():
                self.previous_cued_chamber = self.cued_chamber
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR_END
                rospy.loginfo("ERROR_END")

        elif self.mode == Mode.ERROR_END:
            if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_second_delay.to_sec():
                if self.phase_three:
                    if self.currentTrialNumber < 9:
                        # Before trial 10, do not switch even if self.success_chamber == self.left_chamber
                        print(f"Trial {self.currentTrialNumber}: No switching before trial 10.")
                        self.choose_start_config(self.start_chamber)  

                    elif self.currentTrialNumber == 9 and self.error_chamber == self.left_chamber:
                        # First switch happens at trial 10
                        self.switch_trial = self.currentTrialNumber
                        self.starting_config = self.error_chamber
                        self.choose_start_config(self.starting_config)
                        print(f"First switch at trial {self.currentTrialNumber}")

                    elif self.error_chamber == self.left_chamber:
                         # Switch only if at least 10 trials have passed since the last switch
                        if self.switch_trial is None or self.currentTrialNumber >= self.switch_trial + 10:
                            self.switch_trial = self.currentTrialNumber
                            self.starting_config = self.error_chamber
                            self.choose_start_config(self.starting_config)
                            print(f"Switch at trial {self.currentTrialNumber}")
                        else:
                            self.choose_start_config(self.start_chamber)
                    else:
                        self.choose_start_config(self.start_chamber)

                else:
                    self.setChamberFiveStartConfig()
                    rospy.loginfo("Chamber 5 selected")

                self.projection_wall_img_pub.publish(self.wall_img_black_num)
                self.publish_walls(self.previous_cued_chamber, self.chamber_walls_list)

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.MOVE_TO_START_CHAMBER
                rospy.loginfo("MOVE_TO_START_CHAMBER")

        elif self.mode == Mode.MOVE_TO_START_CHAMBER:
            if self.rat_body_chamber == self.right_chamber:
                self.common_functions.lower_wall(
                    self.right_start_chamber_enter_wall, send=False)
                self.common_functions.lower_wall(
                    self.right_exit_wall, send=True)
                self.rat_choice_chamber = self.right_chamber

            elif self.rat_body_chamber == self.left_chamber:
                self.common_functions.lower_wall(
                    self.left_start_chamber_enter_wall, send=False)
                self.common_functions.lower_wall(
                    self.left_exit_wall, send=True)
                self.rat_choice_chamber = self.left_chamber

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_BACK_IN_START_CHAMBER
            rospy.loginfo("RAT_BACK_IN_START_CHAMBER")

        elif self.mode == Mode.RAT_BACK_IN_START_CHAMBER:
            if (self.current_time - self.mode_start_time).to_sec() >= self.moving_back_to_start_chamber_delay.to_sec():
                if self.rat_body_chamber == self.start_chamber:
                    if self.rat_choice_chamber == self.left_chamber:
                        self.common_functions.raise_wall(
                            self.left_exit_wall, send=False)
                        self.common_functions.raise_wall(
                            self.left_start_chamber_enter_wall, send=True)

                    elif self.rat_choice_chamber == self.right_chamber:
                        self.common_functions.raise_wall(
                            self.right_exit_wall, send=False)
                        self.common_functions.raise_wall(
                            self.right_start_chamber_enter_wall, send=True)
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

        elif self.mode == Mode.END_TRIAL:
            if (self.current_time - self.mode_start_time).to_sec() >= self.end_trial_delay.to_sec():
                self.mode = Mode.START_TRIAL
                rospy.loginfo("START_TRIAL")


if __name__ == '__main__':
    rospy.init_node('single_T_maze')
    Interface()
    rospy.spin()