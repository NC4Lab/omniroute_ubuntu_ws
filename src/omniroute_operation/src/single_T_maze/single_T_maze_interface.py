#!/usr/bin/env python
#!/usr/bin/env python
import os,time
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
from experiment_controller.experiment_controller_interface import Interface as ExistingInterface


class Mode(Enum):
    START = -1
    START_EXPERIMENT = 0
    START_TRIAL = 1
    RAT_IN_START_CHAMBER = 2
    START_TO_CHOICE = 3
    RAT_IN_CHOICE_CHAMBER = 4
    CHOICE = 5
    CHOICE_TO_GOAL = 6
    SUCCESS = 7
    REWARD_START =8
    REWARD_END = 9
    POST_REWARD = 10
    ERROR = 11
    END_TRIAL = 12
    END_EXPERIMENT = 13
    PAUSE_EXPERIMENT = 14
    RESUME_EXPERIMENT = 15
    ERROR_END = 16

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'single_T_maze_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        rospy.loginfo('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))


        # Add widget to the user interface
        context.add_widget(self._widget)

        self.scene = QGraphicsScene()

        self._widget.browseBtn_2.clicked.connect(self._handle_browseBtn_2_clicked)
        self._widget.recordBtn.clicked[bool].connect(self._handle_recordBtn_clicked)
        self._widget.testingPhaseBtn.clicked.connect(self._handle_testingPhaseBtn_clicked)
        self._widget.trialGeneratorBtn.clicked.connect(self._handle_trialGeneratorBtn_clicked)  
        # Button for designating if rewards should be despensed from the gantry
        self._widget.gantryRewardTogBtn.clicked.connect(self._handle_gantryRewardTogBtn_clicked)
        self._widget.plusMazeBtn.clicked.connect(self._handle_plusMazeBtn_clicked)
        self._widget.lowerAllDoorsBtn.clicked.connect(self._handle_lowerAllDoorsBtn_clicked)
        
        self.do_gantry_reward = False
        self.is_testing_phase = False
        self.trial_generator = False
        
        self.maze_dim = MazeDimensions()

        #self.trial_dir = '/media/big_gulp/nc4_rat_data/Maze_Rats'

        # self.rat = 6
        # self.date = '240829'

        #self.rat_folder = os.path.join(self.trial_dir, 'NC4%04d' % self.rat)

        # if '-' in self.date: 
        #     self.date = parsedate(self.date).strftime('%y%m%d')

        # date_folder = os.path.join(self.rat_folder, self.date)

        # self.trial_summary_path = os.path.join(date_folder, 'Past_seven_days_biases.csv')

        # self.df = pd.read_csv(self.trial_summary_path)

        self._widget.lowerAllDoorsBtn.setStyleSheet("background-color: red; color: yellow")

        self.dataDir = os.path.expanduser(os.path.join('~', 'maze_data')) # Default data directory
        self.defaultDataDir = self.dataDir

        self._widget.recordDataDir.setText(self.defaultDataDir)
        
        self.isRecording = self.is_recording_on()

        if self.isRecording:
            self._widget.recordBtn.setStyleSheet("background-color: red; color: yellow")
            self._widget.recordBtn.setText("Stop")
        else:
            self._widget.recordBtn.setStyleSheet("background-color: green; color: yellow")
            self._widget.recordBtn.setText("Record")

        self.projection_pub = rospy.Publisher('projection_cmd', Int32, queue_size=1)
        self.gantry_pub = rospy.Publisher('/gantry_cmd', GantryCmd, queue_size=1)
        self.write_sync_ease_pub = rospy.Publisher('/Esmacat_write_sync_ease', ease_registers, queue_size=1)
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)
        self.trial_sub = rospy.Subscriber('/selected_trial', String, self.trial_callback)

        rospy.Subscriber('/selected_chamber', String,self.chamber_callback, queue_size=1)

        rospy.Subscriber('/mode', String, self.mode_callback, queue_size=1)
        self.button_pub = rospy.Publisher('/button', String, queue_size=1)

        rospy.Subscriber('/rat_head_chamber', Int8, self.rat_head_chamber_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/rat_body_chamber', Int8, self.rat_body_chamber_callback, queue_size=1, tcp_nodelay=True)

        self.experiment_pub = rospy.Publisher('/experiment', String, queue_size=1)

        self.rat_head_chamber = -1
        self.rat_body_chamber = -1
        
        # Time for setting up publishers and subscribers
        rospy.sleep(1.0)

        self.experiment_pub.publish("single_T_maze_experiment")
 
        # Experiment parameters
        self.start_first_delay = rospy.Duration(5.0)  # Duration of delay in the beginning of the trial
        self.start_second_delay = rospy.Duration(6.0)  # Duration of delay in the beginning of the trial
        self.choice_delay = rospy.Duration(1.5)  # Duration to wait for rat to move to the choice point
        self.reward_start_delay = rospy.Duration(13)  # Duration to wait to dispense reward if the rat made the right choice
        self.reward_end_delay = rospy.Duration(2)  # Duration to wait to for the reward to despense
        self.right_choice_delay = rospy.Duration(5)  # Duration to wait if the rat made the right choice
        self.wrong_choice_first_delay = rospy.Duration(35.0)  # Duration to wait if the rat made the wrong choice
        self.wrong_choice_second_delay = rospy.Duration(5.0) 
        self.wrong_choice_delay = rospy.Duration(40)  # Duration to wait if the rat made the wrong choice
        self.end_trial_delay = rospy.Duration(1.0)  # Duration to wait at the end of the trial

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

        self.left_goal_wall = Wall(0, 0)
        self.right_goal_wall = Wall(0, 0)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)

        self.rat_position = 0

        self.maze_dim = MazeDimensions()
        self.existing_interface = ExistingInterface(context)  # Create an instance

        #Trial Types: ['Start Chamber', 'Left Cue', 'Right Cue', 'Sound Cue']
        self.trial_types = {
        1: ['1', 'Triangle', 'No_Cue', 'White_Noise'],
        2: ['1', 'No_Cue', 'Triangle', '5KHz'],
        3: ['1', 'Triangle', 'No_Cue', '5KHz'],
        4: ['1', 'No_Cue', 'Triangle', 'White_Noise'],
        5: ['3', 'Triangle', 'No_Cue', 'White_Noise'],
        6: ['3', 'No_Cue', 'Triangle', '5KHz'],
        7: ['3', 'Triangle', 'No_Cue', '5KHz'],
        8: ['3', 'No_Cue', 'Triangle', 'White_Noise'],
        9: ['5', 'Triangle', 'No_Cue', 'White_Noise'],
        10: ['5', 'No_Cue', 'Triangle', '5KHz'],
        11: ['5', 'Triangle', 'No_Cue', '5KHz'],
        12: ['5', 'No_Cue', 'Triangle', 'White_Noise'],
        13: ['7', 'Triangle', 'No_Cue', 'White_Noise'],
        14: ['7', 'No_Cue', 'Triangle', '5KHz'],
        15: ['7', 'Triangle', 'No_Cue', '5KHz'],
        16: ['7', 'No_Cue', 'Triangle', 'White_Noise']
        }

        self.trial_count = {key: 0 for key in self.trial_types}

    def find_start_chamber(self, id_value, df):
        if id_value == 1:
            trial_types_to_check = [1, 2, 3, 4]
        elif id_value == 3:
            trial_types_to_check = [5, 6, 7, 8]
        elif id_value == 5:
            trial_types_to_check = [9, 10, 11, 12]
        elif id_value == 7:
            trial_types_to_check = [13, 14, 15, 16]
        else:
            raise ValueError("Invalid ID value. It must be 1, 3, 5, or 7.")

        # Extract the values from the specified trial types
        subset_df = df[df['Trial Type'].isin(trial_types_to_check)]
      
        values = subset_df['Error_Count'].to_numpy()
     
        # Normalize the values to create probabilities
        total = np.sum(values)
        probabilities = values / total
        
        # Choose one value based on probabilities
        selected_value = np.random.choice(values, size=1, p=probabilities)
     
        selected_value_index = values.tolist().index(selected_value)

        trial_type = subset_df['Trial Type'].iloc[selected_value_index]
        
        return trial_type

    def generate_trial(self, id_value, df):
        start_chamber = self.find_start_chamber(id_value, df)
        trial = self.trial_types[start_chamber]

        return trial

    def _handle_recordBtn_clicked(self, checked):
        #this function is called when the record button is clicked. It starts/stops recording data files.It saves all the ROS topics to a bag file.
        if not self.isRecording:   # Start recording
            self.dataDir = self._widget.recordDataDir.text()
            if not os.path.isdir(self.dataDir):
                self._widget.recordDataDir.setText(self.defaultDataDir)
                self.dataDir = self.defaultDataDir
                self._widget.recordDataDir.setText(self.dataDir)

            # Record all ROS topics to domeExperimentData.bag
            command_data = f"rosbag record -a -o singleTmazeExperimentData"
            self.recordDataPid = subprocess.Popen(command_data, shell=True, cwd=self.dataDir)

            # Pause for 3 seconds to allow the bag file to be created
            rospy.sleep(3)

            # Send message to send positive TTL output to Optitrack eSync2 which is handled by the sync_sender node
            self.event_pub.publish("start_optitrack_sync", rospy.Time.now())
            
            self._widget.recordBtn.setStyleSheet("background-color: red; color: yellow")
            self._widget.recordBtn.setText("Stop")
            rospy.loginfo('Recording data files')
            self.isRecording = 1

        else:   # Stop recording
            # Send message to send negative TTL output to Optitrack eSync2 which is handled by the sync_sender node
            self.event_pub.publish("stop_optitrack_sync", rospy.Time.now())

            rospy.sleep(1)

            self.terminate_ros_node("/record")

            self._widget.recordBtn.setStyleSheet("background-color: green; color: yellow")
            self._widget.recordBtn.setText("Record")
            rospy.loginfo('Stopping recording')

            self.isRecording = 0

    def _handle_browseBtn_2_clicked(self):
        res = QFileDialog.getExistingDirectory(None,"Select directory for recording",self.dataDir,QFileDialog.ShowDirsOnly)
        self._widget.recordDataDir.setText(res)

    def _handle_testingPhaseBtn_clicked(self):
        self.is_testing_phase = True
        rospy.loginfo("Testing phase selected")
    
    def is_recording_on(self):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        ret = 0
        for str in list_output.decode().split("\n"):
            if (str.startswith("/record")):
                ret = 1
        return ret
    
    def terminate_ros_node(self,s):
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.decode().split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def _handle_trialGeneratorBtn_clicked(self):
        self.trial_generator = True
        rospy.loginfo("Trial Generator enabled")  

    def _handle_lowerAllDoorsBtn_clicked(self):
        self.setLowerConfig()

    def setLowerConfig(self):
        # Lower Walls 0,2,4,6 in chamber 4 (central chamber)
        for i in [0, 2, 4, 6]:
            self.existing_interface.lower_wall(Wall(4, i), False)
        self.existing_interface.activateWalls()

    def _handle_plusMazeBtn_clicked(self):
        self.setPlusConfig()

    def setPlusConfig(self):
        # Lower all walls
        for i in range(9):
            for j in range(8):
                self.existing_interface.lower_wall(Wall(i, j), False)

        for i in [1, 3, 4, 5, 7]:
            for j in range(8):
                self.existing_interface.raise_wall(Wall(i, j), False)

        self.existing_interface.activateWalls()

    def _handle_gantryRewardTogBtn_clicked(self):
        if self._widget.gantryRewardTogBtn.isChecked():
            self.do_gantry_reward = True
        else:
            self.do_gantry_reward = False

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
        rospy.loginfo(f"Received current_trial_index: {self.current_trial_index}")

    def chamber_callback(self, msg):
        chamber_data = json.loads(msg.data)
        self.start_chamber = chamber_data['start_chamber']
        self.central_chamber = chamber_data['central_chamber']
        self.left_chamber = chamber_data['left_chamber']
        self.right_chamber = chamber_data['right_chamber']

        self.project_left_cue_triangle = chamber_data['project_left_cue_triangle']
        self.project_right_cue_triangle = chamber_data['project_right_cue_triangle']

        self.start_wall = Wall.from_dict(chamber_data['start_wall'])
        self.left_goal_wall = Wall.from_dict(chamber_data['left_goal_wall'])
        self.right_goal_wall = Wall.from_dict(chamber_data['right_goal_wall'])

    def rat_head_chamber_callback(self, msg):
        self.rat_head_chamber = msg.data

    def rat_body_chamber_callback(self, msg):
        self.rat_body_chamber = msg.data

    def run_experiment(self):

        self.current_time = rospy.Time.now()
        current_rat_chamber = self.rat_head_chamber

        rospy.loginfo("Single T-maze experiment")
        if current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1:
            # The rat has moved to a different chamber, update the gantry position
            self.existing_interface.move_gantry_to_chamber(current_rat_chamber)

            # Update the previous_rat_chamber for the next iteration
            self.previous_rat_chamber = current_rat_chamber

        if self.mode == Mode.START_EXPERIMENT:
            rospy.loginfo("START OF THE EXPERIMENT")
        
            self.currentStartConfig = self.existing_interface._widget.startChamberBtnGroup.checkedId()

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
        
            if self.currentTrial is not None and self.currentTrialNumber >= self.nTrials:
                self.mode = Mode.END_EXPERIMENT

            if self.currentTrial is not None:
                # Set training mode from file if the automatic mode is selected
                if self.existing_interface._widget.trainingModeBtnGroup.checkedId() == 3:
                    self.training_mode = self.currentTrial[3]

                self.left_visual_cue = self.currentTrial[0]
                self.right_visual_cue = self.currentTrial[1]
                self.floor_cue = self.currentTrial[2]

            if self.floor_cue == "Green":
                if self.left_visual_cue == "Triangle":  
                    self.projection_pub.publish(self.project_left_cue_triangle)
                    rospy.loginfo("Projecting left cue triangle")
                    self.success_chamber = self.left_chamber
                    self.error_chamber = self.right_chamber
                else:
                    self.projection_pub.publish(self.project_right_cue_triangle)
                    rospy.loginfo("Projecting right cue triangle")
                    self.success_chamber = self.right_chamber
                    self.error_chamber = self.left_chamber
            else:
                if self.left_visual_cue == "No_Cue":
                    self.projection_pub.publish(self.project_right_cue_triangle)
                    rospy.loginfo("Projecting right cue triangle")
                    self.success_chamber = self.left_chamber
                    self.error_chamber = self.right_chamber
                else:
                    self.projection_pub.publish(self.project_left_cue_triangle)
                    rospy.loginfo("Projecting left cue triangle")
                    self.success_chamber = self.right_chamber
                    self.error_chamber = self.left_chamber
    
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_IN_START_CHAMBER
            rospy.loginfo("RAT_IN_START_CHAMBER")

        elif self.mode == Mode.RAT_IN_START_CHAMBER:
            if (self.current_time - self.mode_start_time).to_sec() >= self.start_first_delay.to_sec():
                if not self.trial_generator:
                    if self.training_mode is not None and self.training_mode in ["forced_choice", "user_defined_forced_choice"]: 
                        if self.success_chamber == self.left_chamber:
                            self.existing_interface.lower_wall(self.left_goal_wall, send=True)
                        else:
                            self.existing_interface.lower_wall(self.right_goal_wall, send=True)
                    elif self.training_mode is not None and self.training_mode in ["choice", "user_defined_choice"]:
                        self.existing_interface.lower_wall(self.left_goal_wall, send=False)
                        self.existing_interface.lower_wall(self.right_goal_wall, send=True)
                else:
                    self.existing_interface.lower_wall(self.left_goal_wall, send=True)
                    self.existing_interface.lower_wall(self.right_goal_wall, send=True)
                
                self.mode = Mode.START
                rospy.loginfo("START")

        elif self.mode == Mode.START:
            if (self.current_time - self.mode_start_time).to_sec() >= self.start_second_delay.to_sec():
                self.existing_interface.lower_wall(self.start_wall, send=True)
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
            if (self.current_time - self.mode_start_time).to_sec() >= self.choice_delay.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.CHOICE_TO_GOAL
                rospy.loginfo("CHOICE TO GOAL")

        elif self.mode == Mode.CHOICE_TO_GOAL:
            if self.rat_body_chamber == self.success_chamber:
                self.existing_interface.raise_wall(self.left_goal_wall, send=False)
                self.existing_interface.raise_wall(self.right_goal_wall, send=False)
                self.existing_interface.raise_wall(self.start_wall, send=True)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.SUCCESS
                rospy.loginfo("SUCCESS")

            elif self.rat_body_chamber == self.error_chamber:
                self.existing_interface.raise_wall(self.left_goal_wall, send=False)
                self.existing_interface.raise_wall(self.right_goal_wall, send=False)
                self.existing_interface.raise_wall(self.start_wall, send=True)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR
                rospy.loginfo("ERROR")
                if self.error_chamber == self.left_chamber:
                    rospy.loginfo("Left chamber selected and chamber number is {}".format(self.error_chamber))
                else:
                    rospy.loginfo("Right chamber selected and chamber number is {}".format(self.error_chamber))

        elif self.mode == Mode.SUCCESS:
            if self.success_chamber == self.left_chamber:
                rospy.loginfo("Left chamber selected and chamber number is {}".format(self.success_chamber))
            else:
                rospy.loginfo("Right chamber selected and chamber number is {}".format(self.success_chamber))
            self.success_center_x = self.maze_dim.chamber_centers[self.success_chamber][0]
            self.success_center_y = self.maze_dim.chamber_centers[self.success_chamber][1]
            self.gantry_pub.publish("MOVE", [self.success_center_x, self.success_center_y])
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.REWARD_START
            rospy.loginfo("REWARD_START")

        elif self.mode == Mode.REWARD_START:
            if (self.current_time - self.mode_start_time).to_sec() >= self.reward_start_delay.to_sec():
                if self.do_gantry_reward:
                    self.reward_dispense()
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_END
                rospy.loginfo("REWARD END")

        elif self.mode == Mode.REWARD_END:
            if (self.current_time - self.mode_start_time).to_sec() >= self.reward_end_delay.to_sec():
                #self.gantry_pub.publish("TRACK_HARNESS", [])
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.POST_REWARD
                rospy.loginfo("POST REWARD") 

        elif self.mode == Mode.POST_REWARD:
            if (self.current_time - self.mode_start_time).to_sec() >= self.right_choice_delay.to_sec():
                self.existing_interface.setChamberFiveStartConfig()
                rospy.loginfo("Chamber 5 selected")
                        
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.END_TRIAL
                rospy.loginfo("END TRIAL")
                
        elif self.mode == Mode.ERROR:
            if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_first_delay.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR_END
                rospy.loginfo("ERROR_END")

        elif self.mode == Mode.ERROR_END:
            if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_second_delay.to_sec():
                self.existing_interface.setChamberFiveStartConfig()
                rospy.loginfo("Chamber 5 selected")
                
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.END_TRIAL
                rospy.loginfo("END TRIAL")

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

    def reward_dispense(self):
        self.gantry_pub.publish("REWARD", [4.0]) # Send with pump duration (sec)

    def move_gantry_to_chamber(self, chamber_num):
        x = self.maze_dim.chamber_centers[chamber_num][0]
        y = self.maze_dim.chamber_centers[chamber_num][1]


if __name__ == '__main__':
    rospy.init_node('single_T_maze')
    Interface()
    rospy.spin()

