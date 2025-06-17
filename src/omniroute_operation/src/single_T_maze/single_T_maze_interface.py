#!/usr/bin/env python

#!/usr/bin/env python
from shared_utils.ui_utilities import UIUtilities
from shared_utils.wall_utilities import MazeDimensions 
from experiment_controller.experiment_controller_interface import Wall
from experiment_controller.experiment_controller_interface import CommonFunctions

import os
import rospy
import subprocess
import random
from std_msgs.msg import String, Int32, Int8
from omniroute_operation.msg import *
from omniroute_esmacat_ros.msg import *
from dateutil.parser import parse as parsedate

from enum import Enum, auto
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL
from PyQt5.QtWidgets import QGraphicsScene, QButtonGroup, QWidget
from PyQt5.QtCore import QTimer

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin

import json

class FloorCue(Enum):
    GREEN = 1
    BLACK = 2

class TriangleCue(Enum):
    LEFT = 1
    RIGHT = 2

class TrainingMode(Enum):
    FORCED_CHOICE = 1
    FREE_CHOICE = 2

class Trial:
    def __init__(self, visual_cue, floor_cue, training_mode = TrainingMode.FREE_CHOICE):
        self.visual_cue = visual_cue
        self.floor_cue = floor_cue
        self.training_mode = training_mode
    
    def __print__(self):
        return f"Trial(visual_cue={self.visual_cue}, floor_cue={self.floor_cue}, training_mode={self.training_mode})"
    

class ExperimentPhases(Enum):
    PHASE_ONE = 1
    PHASE_TWO = 2
    PHASE_THREE = 3
    FROM_CSV = 4


class Mode(Enum):
    START = auto()
    START_EXPERIMENT = auto()
    START_TRIAL = auto()
    RAT_IN_START_CHAMBER = auto()
    RAT_WAITS = auto()
    START_TO_CHOICE = auto()
    CHOICE_TO_GOAL = auto()
    SUCCESS = auto()
    REWARD_START = auto()
    REWARD_END = auto()
    POST_REWARD = auto()
    ERROR = auto()
    END_TRIAL = auto()
    END_EXPERIMENT = auto()
    PAUSE_EXPERIMENT = auto()
    RESUME_EXPERIMENT = auto()
    ERROR_END = auto()
    ERROR_START = auto()
    MOVE_TO_START_CHAMBER = auto()
    RAT_BACK_IN_START_CHAMBER = auto()


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

        self._widget.trialGeneratorBtn.clicked.connect(
            self._handle_trialGeneratorBtn_clicked)
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
        self.phase = ExperimentPhases.PHASE_ONE

        self.maze_dim = MazeDimensions()

        self._widget.lowerAllDoorsBtn.setStyleSheet(
            "background-color: red; color: yellow")

        self.sound_pub = rospy.Publisher('sound_cmd', String, queue_size=1)

        self.projection_pub = rospy.Publisher(
            'projection_walls', String, queue_size=100)
        self.projection_floor_pub = rospy.Publisher(
            'projection_image_floor_num', Int32, queue_size=100)
        self.projection_wall_img_pub = rospy.Publisher(
            'projection_image_wall_num', Int32, queue_size=1)
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
        self.moving_back_to_start_chamber_delay = rospy.Duration(1)  
        # Duration to wait at the end of the trial 
        self.end_trial_delay = rospy.Duration(1.0)

        self.mode = Mode.START
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        self.currentTrial = Trial(TriangleCue.LEFT, FloorCue.GREEN, TrainingMode.FREE_CHOICE)
        self.currentTrialNumber = 0
        self.nTrials = 0
        self.trials = []

        self.training_mode = None

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

        self.rat_position = 0

        self.floor_img_black_num = 0
        self.floor_img_green_num = 1
        self.wall_img_triangle_num = 3
        self.wall_img_black_num = 0

        self.cued_chamber = 0

        self.switch_trial = None

        # self.project_floor_img = Wall(9, 0).to_dict()
        self.cf = CommonFunctions()  # Create an instance

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

        self.cts_success_count = {
            FloorCue.GREEN: 0,
            FloorCue.BLACK: 0
        }

        self.total_success_count = 0
        self.previous_trial_result = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10) # 10 ms
    
    def pick_trial_phase_one(self):
        rospy.loginfo("Picking trial phase one")
        if self.cts_success_count[self.currentTrial.floor_cue] > 2:
            self.cts_success_count[self.currentTrial.floor_cue] = 0

            # Switch to the other group
            if self.currentTrial.floor_cue == FloorCue.GREEN:
                nextTrial = Trial(random.choice(list(TriangleCue)), FloorCue.BLACK, self.training_mode)
            else:
                nextTrial = Trial(random.choice(list(TriangleCue)), FloorCue.GREEN, self.training_mode)

            print(f"Switching to trial: {nextTrial}")
        else:
            nextTrial = Trial(random.choice(list(TriangleCue)), self.currentTrial.floor_cue, self.training_mode)

        return nextTrial

    def pick_trial_phase_two(self):
        # Randomly select a trial type from the dictionary
        return Trial(random.choice(list(TriangleCue)), random.choice(list(FloorCue)), self.training_mode)

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
        self.phase = ExperimentPhases.PHASE_ONE
        rospy.loginfo("Phase One selected")

    def _handle_phaseTwoBtn_clicked(self):
        self.phase = ExperimentPhases.PHASE_TWO
        rospy.loginfo("Phase Two selected")

    def _handle_phaseThreeBtn_clicked(self):
        self.phase = ExperimentPhases.PHASE_THREE
        rospy.loginfo("Phase Three selected")

    def _handle_lowerAllDoorsBtn_clicked(self):
        # Lower Walls 0,2,4,6 in chamber 4 (central chamber)
        for i in [0, 2, 4, 6]:
            self.cf.lower_wall(Wall(4, i), False)
        self.cf.activateWalls()

    def _handle_plusMazeBtn_clicked(self):
        for i in range(9):
            for j in range(8):
                self.cf.raise_wall(Wall(i, j), False)

        self.cf.activateWalls()

    def _handle_startChamberBtnGroup_clicked(self):
        self.choose_start_config(self._widget.startChamberBtnGroup.checkedId())

    def mode_callback(self, msg):
        mode = msg.data
        if mode == "START_EXPERIMENT":
            self.mode = Mode.START_EXPERIMENT
        elif mode == "PAUSE_EXPERIMENT":
            self.mode_before_pause = self.mode
            self.mode = Mode.PAUSE_EXPERIMENT
        elif mode == "RESUME_EXPERIMENT":
            self.mode = Mode.RESUME_EXPERIMENT
        
    def trial_class_from_data(self, trial_data):
        # Convert the string back into a list (if necessary)
        if trial_data[0] == "Triangle":
            triangle_cue = TriangleCue.LEFT
        else:
            triangle_cue = TriangleCue.RIGHT
        
        if trial_data[3] == "Green":
            floor_cue = FloorCue.GREEN
        else:
            floor_cue = FloorCue.BLACK
        
        if trial_data[4] == "forced_choice":
            training_mode = TrainingMode.FORCED_CHOICE
        else:
            training_mode = TrainingMode.FREE_CHOICE

        return Trial(triangle_cue, floor_cue, training_mode)

    def trial_callback(self, msg):
        # Convert the string back into a list (if necessary)
        self.phase = ExperimentPhases.FROM_CSV
        trial_data = json.loads(msg.data)
        self.currentTrial = self.trial_class_from_data(msg.data)
        self.currentTrialNumber = trial_data['current_trial_index']-1
        self.nTrials = trial_data['nTrials']
        for i in range(self.nTrials):
            self.trials.append(self.trial_class_from_data(trial_data['trials'][i]))

        # Log the received trial and index
        rospy.loginfo(f"Received selected trial: {self.currentTrial}")
        rospy.loginfo(
            f"Received current_trial_index: {self.currentTrialNumber}")

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
                           Wall(5, 2).to_dict(),
                           Wall(5, 3).to_dict(),
                           Wall(5, 4).to_dict(),
                           Wall(5, 5).to_dict(),
                           Wall(5, 6).to_dict(),
                           Wall(5, 7).to_dict(),
                           Wall(4, 5).to_dict()]

        self.right_walls = [Wall(3, 0).to_dict(),
                            Wall(3, 1).to_dict(),
                            Wall(3, 2).to_dict(),
                            Wall(3, 3).to_dict(),
                            Wall(3, 4).to_dict(),
                            Wall(3, 5).to_dict(),
                            Wall(3, 6).to_dict(),
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

        self.left_walls = [Wall(1, 0).to_dict(),
                           Wall(1, 1).to_dict(),
                           Wall(1, 2).to_dict(),
                           Wall(1, 3).to_dict(),
                           Wall(1, 4).to_dict(),
                           Wall(1, 5).to_dict(),
                           Wall(1, 6).to_dict(),
                           Wall(1, 7).to_dict(),
                           Wall(4, 3).to_dict()]

        self.right_walls = [Wall(7, 0).to_dict(),
                            Wall(7, 1).to_dict(),
                            Wall(7, 2).to_dict(),
                            Wall(7, 3).to_dict(),
                            Wall(7, 4).to_dict(),
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

        self.left_walls = [Wall(7, 0).to_dict(),
                           Wall(7, 1).to_dict(),
                           Wall(7, 2).to_dict(),
                           Wall(7, 3).to_dict(),
                           Wall(7, 4).to_dict(),
                           Wall(7, 5).to_dict(),
                           Wall(7, 6).to_dict(),
                           Wall(7, 7).to_dict(),
                           Wall(4, 7).to_dict()]

        self.right_walls = [Wall(1, 0).to_dict(),
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
                           Wall(3, 2).to_dict(),
                           Wall(3, 3).to_dict(),
                           Wall(3, 4).to_dict(),
                           Wall(3, 5).to_dict(),
                           Wall(3, 6).to_dict(),
                           Wall(3, 7).to_dict(),
                           Wall(4, 1).to_dict()]

        self.right_walls = [Wall(5, 0).to_dict(),
                            Wall(5, 1).to_dict(),
                            Wall(5, 2).to_dict(),
                            Wall(5, 3).to_dict(),
                            Wall(5, 4).to_dict(),
                            Wall(5, 5).to_dict(),
                            Wall(5, 6).to_dict(),
                            Wall(5, 7).to_dict(),
                            Wall(4, 3).to_dict()]

        self.start_wall = Wall(7, 2)
        self.left_goal_wall = Wall(4, 0)
        self.right_goal_wall = Wall(4, 4)
        self.right_exit_wall = Wall(8, 2)
        self.left_exit_wall = Wall(6, 2)
        self.right_start_chamber_enter_wall = Wall(8, 0)
        self.left_start_chamber_enter_wall = Wall(6, 4)

    def blank_cued_walls(self):
        self.projection_wall_img_pub.publish(self.wall_img_black_num)
        for key in self.chamber_walls_list:
            for obj in self.chamber_walls_list[key]:
                self.projection_pub.publish(json.dumps(obj))

        # if self.cued_chamber in self.chamber_walls_list:
        #     # Get the list of objects associated with the key
        #     object_list = self.chamber_walls_list[self.cued_chamber]

        #     # Publish each object in the list
        #     for obj in object_list:
        #         # Assuming obj can be serialized with json.dumps
        #         self.projection_pub.publish(json.dumps(obj))
        #         rospy.loginfo(f"Published: {obj}")
        #         # Add a small delay to ensure proper publishing
        #         # rospy.sleep(0.1)
        # else:
        #     rospy.logwarn(
        #         f"Key {self.cued_chamber} not found in the dictionary.")
            
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

    def switch_to_mode(self, mode):
        self.mode_start_time = self.current_time
        self.mode = mode
        rospy.loginfo(f"Switching to mode: {mode}")

    def run_experiment(self):
        self.current_time = rospy.Time.now()

        if self.mode == Mode.START_EXPERIMENT:
            self.switch_to_mode(Mode.START_TRIAL)
            self.switch_trial = self.currentTrialNumber

            # AWL Wait for walls to be initialized
            if not rospy.get_param('/shared_state/is_maze_initialized'):
                return

        elif self.mode == Mode.START_TRIAL:
            # publish the images to be projected on the walls
            self.currentTrialNumber = self.currentTrialNumber+1
            rospy.loginfo(f"Current trial number: {self.currentTrialNumber}")
            #if self.trial_generator:
            if self.phase == ExperimentPhases.PHASE_ONE:
                self.currentTrial = self.pick_trial_phase_one()
                
            elif self.phase == ExperimentPhases.PHASE_TWO or self.phase == ExperimentPhases.PHASE_THREE:
                self.currentTrial = self.pick_trial_phase_two()

            else:
                if self.trials and 0 <= self.currentTrialNumber < len(self.trials):
                    self.currentTrial = self.trials[self.currentTrialNumber]
                else:
                    # Handle the case where trials is empty or currentTrialNumber is out of range
                    self.currentTrial = None

                if self.currentTrial is None or self.currentTrialNumber >= self.nTrials:
                    self.switch_to_mode(Mode.END_EXPERIMENT)

            self.sound_pub.publish("Starting_Sound")
            rospy.loginfo("Starting sound played")
            # rospy.sleep(0.1)

            if self.currentTrial.floor_cue == FloorCue.GREEN:
                self.projection_floor_pub.publish(self.floor_img_green_num)
                # rospy.sleep(0.1)
                if self.currentTrial.visual_cue == TriangleCue.LEFT:
                    self.cued_chamber = self.left_chamber
                    self.projection_wall_img_pub.publish(self.wall_img_triangle_num)
                    # rospy.sleep(0.1)
                    for i in self.left_walls:
                        self.projection_pub.publish(json.dumps(i))
                        # rospy.sleep(0.1)
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.left_chamber
                    self.error_chamber = self.right_chamber
                elif self.currentTrial.visual_cue == TriangleCue.RIGHT:
                    self.cued_chamber = self.right_chamber
                    self.projection_wall_img_pub.publish(self.wall_img_triangle_num)
                    # rospy.sleep(0.1)
                    for i in self.right_walls:
                        self.projection_pub.publish(json.dumps(i))
                        # rospy.sleep(0.1)
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.right_chamber
                    self.error_chamber = self.left_chamber
            elif self.currentTrial.floor_cue == FloorCue.BLACK:
                if self.currentTrial.visual_cue == TriangleCue.RIGHT:
                    self.cued_chamber = self.right_chamber
                    self.projection_wall_img_pub.publish(self.wall_img_triangle_num)
                    # rospy.sleep(0.1)
                    for i in self.right_walls:
                        self.projection_pub.publish(json.dumps(i))
                        # rospy.sleep(0.1)
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.left_chamber
                    self.error_chamber = self.right_chamber
                elif self.currentTrial.visual_cue == TriangleCue.LEFT:
                    self.cued_chamber = self.left_chamber
                    self.projection_wall_img_pub.publish(self.wall_img_triangle_num)
                    # rospy.sleep(0.1)
                    for i in self.left_walls:
                        self.projection_pub.publish(json.dumps(i))
                        # rospy.sleep(0.1)
                    rospy.loginfo("Projecting wall images")
                    self.success_chamber = self.right_chamber
                    self.error_chamber = self.left_chamber

            self.switch_to_mode(Mode.RAT_WAITS)

        elif self.mode == Mode.RAT_WAITS:
            if (self.current_time - self.mode_start_time) >= self.start_first_delay:
                self.projection_floor_pub.publish(self.floor_img_black_num)
                # rospy.sleep(0.1)
                self.switch_to_mode(Mode.RAT_IN_START_CHAMBER)

        elif self.mode == Mode.RAT_IN_START_CHAMBER:

            if (self.current_time - self.mode_start_time) >= self.delay:
                if self.phase == ExperimentPhases.FROM_CSV:
                    if self.training_mode in ["forced_choice", "user_defined_forced_choice"]:
                        if self.success_chamber == self.left_chamber:
                            self.cf.lower_wall(
                                self.left_goal_wall, send=True)
                            rospy.loginfo("Lowering left goal wall")
                        else:
                            self.cf.lower_wall(
                                self.right_goal_wall, send=True)
                            rospy.loginfo("Lowering right goal wall")
                    elif self.training_mode in ["choice", "user_defined_choice"]:
                        self.cf.lower_wall(
                            self.left_goal_wall, send=False)
                        self.cf.lower_wall(
                            self.right_goal_wall, send=True)
                else: # self.phase is PHASE_ONE or PHASE_TWO or PHASE_THREE
                    self.cf.lower_wall(
                        self.left_goal_wall, send=False)
                    self.cf.lower_wall(
                        self.right_goal_wall, send=True)

                self.switch_to_mode(Mode.START)

        elif self.mode == Mode.START:
            if (self.current_time - self.mode_start_time) >= self.start_second_delay:
                self.cf.lower_wall(self.start_wall, send=True)
                self.switch_to_mode(Mode.START_TO_CHOICE)

        elif self.mode == Mode.START_TO_CHOICE:
            # Wait for the rat to move to the choice point
            if self.rat_head_chamber == self.central_chamber:
                self.switch_to_mode(Mode.CHOICE_TO_GOAL)

        elif self.mode == Mode.CHOICE_TO_GOAL:
            if self.rat_body_chamber == self.success_chamber:
                self.switch_to_mode(Mode.SUCCESS)

            elif self.rat_body_chamber == self.error_chamber:
                self.switch_to_mode(Mode.ERROR)

        elif self.mode == Mode.SUCCESS:
            self.cf.raise_wall(
                self.left_goal_wall, send=False)
            self.cf.raise_wall(
                self.right_goal_wall, send=False)
            self.cf.raise_wall(self.start_wall, send=True)

            self.total_success_count += 1
            self.cts_success_count[self.currentTrial.floor_cue] += 1

            if self.success_chamber == self.left_chamber:
                rospy.loginfo("Left chamber selected and chamber number is {}".format(
                    self.success_chamber))
            else:
                rospy.loginfo("Right chamber selected and chamber number is {}".format(
                    self.success_chamber))

            self.switch_to_mode(Mode.REWARD_START)

        elif self.mode == Mode.REWARD_START:
            if (self.current_time - self.mode_start_time) >= self.reward_start_delay:
                # self.common_functions.reward_dispense()
                self.switch_to_mode(Mode.REWARD_END)

        elif self.mode == Mode.REWARD_END:
            if (self.current_time - self.mode_start_time) >= self.reward_end_delay:
                self.switch_to_mode(Mode.POST_REWARD)

        elif self.mode == Mode.POST_REWARD:
            if (self.current_time - self.mode_start_time) >= self.right_choice_delay:
                if self.phase == ExperimentPhases.PHASE_THREE and self.success_chamber == self.left_chamber and self.currentTrialNumber >= self.switch_trial + 15:
                    self.switch_trial = self.currentTrialNumber
                    self.choose_start_config(self.success_chamber)
                    print(f"Switch at trial {self.currentTrialNumber}")
                elif self.phase != ExperimentPhases.PHASE_THREE:
                    self.setChamberFiveStartConfig()
                    rospy.loginfo("Chamber 5 selected")
                else:
                    self.choose_start_config(self.start_chamber)

                self.blank_cued_walls()
                # rospy.sleep(0.1)

                self.switch_to_mode(Mode.MOVE_TO_START_CHAMBER)

        elif self.mode == Mode.ERROR:
            self.cf.raise_wall(
                self.left_goal_wall, send=False)
            self.cf.raise_wall(
                self.right_goal_wall, send=False)
            self.cf.raise_wall(self.start_wall, send=True)

            self.cts_success_count[self.currentTrial.floor_cue] = 0                

            self.sound_pub.publish("Error")
            rospy.loginfo("Error sound played")
            if self.error_chamber == self.left_chamber:
                rospy.loginfo(
                    "Left chamber selected and chamber number is {}".format(self.error_chamber))
            else:
                rospy.loginfo(
                    "Right chamber selected and chamber number is {}".format(self.error_chamber))
            self.switch_to_mode(Mode.ERROR_START)

        elif self.mode == Mode.ERROR_START:
            if (self.current_time - self.mode_start_time) >= self.wrong_choice_first_delay:
                self.switch_to_mode(Mode.ERROR_END)

        elif self.mode == Mode.ERROR_END:
            if (self.current_time - self.mode_start_time) >= self.wrong_choice_second_delay:
                if self.phase == ExperimentPhases.PHASE_THREE and self.error_chamber == self.left_chamber and self.currentTrialNumber >= self.switch_trial + 15:
                    self.switch_trial = self.currentTrialNumber
                    self.choose_start_config(self.error_chamber)
                    print(f"Switch at trial {self.currentTrialNumber}")

                elif self.phase != ExperimentPhases.PHASE_THREE:
                    self.setChamberFiveStartConfig()
                    rospy.loginfo("Chamber 5 selected")
                else:
                    self.choose_start_config(self.start_chamber)

                self.blank_cued_walls()
                # rospy.sleep(0.1)
                self.switch_to_mode(Mode.MOVE_TO_START_CHAMBER)

        elif self.mode == Mode.MOVE_TO_START_CHAMBER:
            if self.rat_body_chamber == self.right_chamber: # If the rat is in the right chamber, lower the wall to go back to the start chamber
                self.cf.lower_wall(
                    self.right_start_chamber_enter_wall, send=False)
                self.cf.lower_wall(
                    self.right_exit_wall, send=True)
                self.rat_choice_chamber = self.right_chamber

            elif self.rat_body_chamber == self.left_chamber: # If the rat is in the left chamber, lower the wall to go back to the start chamber
                self.cf.lower_wall(
                    self.left_start_chamber_enter_wall, send=False)
                self.cf.lower_wall(
                    self.left_exit_wall, send=True)
                self.rat_choice_chamber = self.left_chamber

            self.switch_to_mode(Mode.RAT_BACK_IN_START_CHAMBER)

        elif self.mode == Mode.RAT_BACK_IN_START_CHAMBER:
            if (self.current_time - self.mode_start_time) >= self.moving_back_to_start_chamber_delay:
                if self.rat_body_chamber == self.start_chamber:
                    if self.rat_choice_chamber == self.left_chamber:
                        self.cf.raise_wall(
                            self.left_exit_wall, send=False)
                        self.cf.raise_wall(
                            self.left_start_chamber_enter_wall, send=True)

                    elif self.rat_choice_chamber == self.right_chamber:
                        self.cf.raise_wall(
                            self.right_exit_wall, send=False)
                        self.cf.raise_wall(
                            self.right_start_chamber_enter_wall, send=True)
                    self.switch_to_mode(Mode.END_TRIAL)

        elif self.mode == Mode.PAUSE_EXPERIMENT:
            rospy.loginfo("PAUSE_EXPERIMENT")
            self.button_pub.publish("Pause_button_disabled")
            self.mode_start_time = self.current_time

        elif self.mode == Mode.RESUME_EXPERIMENT:
            rospy.loginfo("RESUME_EXPERIMENT")
            self.button_pub.publish("Pause_button_enabled")
            self.mode_start_time = self.current_time
            self.mode = self.mode_before_pause

        elif self.mode == Mode.END_TRIAL:
            if (self.current_time - self.mode_start_time) >= self.end_trial_delay:
                self.switch_to_mode(Mode.START_TRIAL)

if __name__ == '__main__':
    rospy.init_node('single_T_maze')
    Interface()
    rospy.spin()
