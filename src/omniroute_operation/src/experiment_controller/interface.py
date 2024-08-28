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
from omniroute_controller.interface import MazeDimensions
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


class Mode(Enum):
    # START = -1
    # START_EXPERIMENT = 0
    # START_TRIAL = 1
    # RAT_IN_START_CHAMBER = 2
    # START_TO_CHOICE = 3
    # RAT_IN_CHOICE_CHAMBER = 4
    # CHOICE = 5
    # CHOICE_TO_GOAL = 6
    # SUCCESS = 7
    # REWARD =8
    # POST_REWARD = 9
    # ERROR = 10
    # END_TRIAL = 11
    # END_EXPERIMENT = 12
    # PAUSE_EXPERIMENT = 13
    # RESUME_EXPERIMENT = 14
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

class Wall:
    def __init__(self, chamber_num, wall_num):

        
        # Create equivalence between walls
        if chamber_num==1:
            if wall_num==0:
                chamber_num = 0
                wall_num = 4
            elif wall_num==4:
                chamber_num = 2
                wall_num = 0
            elif wall_num==6:
                chamber_num = 4
                wall_num = 2
        elif chamber_num==3:
            if wall_num==2:
                chamber_num = 0
                wall_num = 6
            elif wall_num==4:
                chamber_num = 4
                wall_num = 0
            elif wall_num==6:
                chamber_num = 6
                wall_num = 2
        elif chamber_num==5:
            if wall_num==0:
                chamber_num = 4
                wall_num = 4
            elif wall_num==2:
                chamber_num = 2
                wall_num = 6
            elif wall_num==6:
                chamber_num = 8
                wall_num = 2
        elif chamber_num==7:
            if wall_num==0:
                chamber_num = 6
                wall_num = 4
            elif wall_num==2:
                chamber_num = 4
                wall_num = 6
            elif wall_num==4:
                chamber_num = 8
                wall_num = 0            

        self.chamber_num = chamber_num
        self.wall_num = wall_num  

    def __repr__(self):
            return f'Wall({self.chamber_num}, {self.wall_num})'    

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'experiment_controller_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        rospy.loginfo('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))


        # Add widget to the user interface
        context.add_widget(self._widget)

        self.scene = QGraphicsScene()

        self._widget.pauseBtn.setEnabled(True)
        self._widget.resumeBtn.setEnabled(False)

        self._widget.startChamberBtnGroup = QButtonGroup()
        self._widget.startChamberBtnGroup.addButton(self._widget.chamberOneBtn, id=1)
        self._widget.startChamberBtnGroup.addButton(self._widget.chamberThreeBtn, id=3)
        self._widget.startChamberBtnGroup.addButton(self._widget.chamberFiveBtn, id=5)
        self._widget.startChamberBtnGroup.addButton(self._widget.chamberSevenBtn, id=7)
        self._widget.startChamberBtnGroup.setExclusive(True)
        for button in self._widget.startChamberBtnGroup.buttons():
            button.setEnabled(True)

        self._widget.trainingModeBtnGroup = QButtonGroup()
        self._widget.trainingModeBtnGroup.addButton(self._widget.forcedChoiceBtn, id=1)
        self._widget.trainingModeBtnGroup.addButton(self._widget.choiceBtn, id=2)
        self._widget.trainingModeBtnGroup.addButton(self._widget.automaticBtn, id=3)
        self._widget.trainingModeBtnGroup.setExclusive(True)
        for button in self._widget.trainingModeBtnGroup.buttons():
            button.setEnabled(True)
        self._widget.automaticBtn.setChecked(True)  # Default to automatic mode

        self._widget.browseBtn.clicked.connect(self._handle_browseBtn_clicked)
        self._widget.previousBtn.clicked.connect(self._handle_previousBtn_clicked)
        self._widget.nextBtn.clicked.connect(self._handle_nextBtn_clicked)
        self._widget.nextBtn_2.clicked.connect(self._handle_nextBtn_2_clicked)
        self._widget.previousBtn_2.clicked.connect(self._handle_previousBtn_2_clicked)
        self._widget.startBtn.clicked.connect(self._handle_startBtn_clicked)
        self._widget.resumeBtn.clicked.connect(self._handle_resumeBtn_clicked)
        self._widget.pauseBtn.clicked.connect(self._handle_pauseBtn_clicked)
        self._widget.startChamberBtnGroup.buttonClicked.connect(self._handle_startChamberBtnGroup_clicked)
        self._widget.trainingModeBtnGroup.buttonClicked.connect(self._handle_trainingModeBtnGroup_clicked)
        self._widget.xlsxFileListWidget.itemClicked.connect(self._handle_xlsxFileListWidget_item_clicked)
        self._widget.trialListWidget.itemClicked.connect(self._handle_trialListWidget_item_clicked)
        self._widget.pumpGantryBtn.clicked.connect(self.reward_dispense)
        self._widget.pumpStartBtn.clicked.connect(self._handle_pumpStartBtn_clicked)
        self._widget.plusMazeBtn.clicked.connect(self._handle_plusMazeBtn_clicked)
        self._widget.homeBtn.clicked.connect(self._handle_homeBtn_clicked)
        self._widget.stopTrackingBtn.clicked.connect(self._handle_stopTrackingBtn_clicked)
        self._widget.startTrackingBtn.clicked.connect(self._handle_startTrackingBtn_clicked)
        self._widget.stopPumpBtn.clicked.connect(self._handle_stopPumpBtn_clicked)
        self._widget.browseBtn_2.clicked.connect(self._handle_browseBtn_2_clicked)
        self._widget.recordBtn.clicked[bool].connect(self._handle_recordBtn_clicked)
        self._widget.testingPhaseBtn.clicked.connect(self._handle_testingPhaseBtn_clicked)
        self._widget.trialGeneratorBtn.clicked.connect(self._handle_trialGeneratorBtn_clicked)  
        self._widget.whiteNoiseBtn.clicked.connect(self._handle_whiteNoiseBtn_clicked)
        self._widget.fiveKhzBtn.clicked.connect(self._handle_fiveKhzBtn_clicked)
        self._widget.triangleLeftBtn.clicked.connect(self._handle_triangleLeftBtn_clicked)
        self._widget.triangleRightBtn.clicked.connect(self._handle_triangleRightBtn_clicked)
        self._widget.lowerAllDoorsBtn.clicked.connect(self._handle_lowerAllDoorsBtn_clicked)
        # Button for designating if this is the phys rat
        self._widget.ephysRatTogBtn.clicked.connect(self._handle_ephysRatTogBtn_clicked)
        
        self.is_ephys_rat = False
        self.is_testing_phase = False
        self.trial_generator = False
        self.white_noise = False
        self.five_khz = False
        self.triangle_left = False
        self.triangle_right = False

        self.maze_dim = MazeDimensions()

        self.trial_dir = '/media/big_gulp/nc4_rat_data/Maze_Rats'

        self.rat = 6
        self.date = '240826'

        self.rat_folder = os.path.join(self.trial_dir, 'NC4%04d' % self.rat)

        if '-' in self.date: 
            self.date = parsedate(self.date).strftime('%y%m%d')

        date_folder = os.path.join(self.rat_folder, self.date)

        self.trial_summary_path = os.path.join(date_folder, 'Past_seven_days_biases.csv')

        self.df = pd.read_csv(self.trial_summary_path)

        self.curDir = os.path.dirname(__file__)

        self._widget.pathDirEdit.setText(self.curDir)

        self.dataDir = os.path.expanduser(os.path.join('~', 'omniroute_ubuntu_ws', 'src', 'omniroute_operation', 'src','experiment_controller','data')) # Default data directory
        self.defaultDataDir = self.dataDir

        self._widget.lowerAllDoorsBtn.setStyleSheet("background-color: red; color: yellow")

        self._widget.recordDataDir.setText(self.defaultDataDir)
        
        self.isRecording = self.is_recording_on()

        if self.isRecording:
            self._widget.recordBtn.setStyleSheet("background-color: red; color: yellow")
            self._widget.recordBtn.setText("Stop")
        else:
            self._widget.recordBtn.setStyleSheet("background-color: green; color: yellow")
            self._widget.recordBtn.setText("Record")

        self.sound_pub = rospy.Publisher('sound_cmd', String, queue_size=1)
        self.door_pub = rospy.Publisher('/wall_state_cmd', WallState, queue_size=200)
        self.projection_pub = rospy.Publisher('projection_cmd', Int32, queue_size=1)
        self.gantry_pub = rospy.Publisher('/gantry_cmd', GantryCmd, queue_size=1)
        self.write_sync_ease_pub = rospy.Publisher('/Esmacat_write_sync_ease', ease_registers, queue_size=1)
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)
        
        #Initialize the subsrciber for reading from harness and maze boundary markers posistions
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        self.harness_pose = PoseStamped()
        self.harness_x = 0.0
        self.harness_y = 0.0
        
        rospy.Subscriber('/rat_head_chamber', Int8, self.rat_head_chamber_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/rat_body_chamber', Int8, self.rat_body_chamber_callback, queue_size=1, tcp_nodelay=True)

        self.rat_head_chamber = -1
        self.rat_body_chamber = -1
        
        # Time for setting up publishers and subscribers
        rospy.sleep(1.0)

        # # Not needed anymore
        # reg = [0]*8
        # reg[0] = 0
        # self.write_sync_ease_pub.publish(*reg)

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

        # Filter the DataFrame for the relevant trial types
        section = df[df['Trial Type'].isin(trial_types_to_check)]

        # Find the row with the maximum value in column 'Error_Count'
        max_row = section.loc[section['Error_Count'].idxmax()]

        # Get the trial type from the selected row
        trial_type = max_row['Trial Type']

        # Check if this trial type has been selected 8 times
        if self.trial_count[trial_type] < 8:
            # If not selected 8 times, return this trial type
            return trial_type
        else:
            # If selected 8 times, find the next trial type with the highest 'Error_Count'
            section = section.drop(section[section['Trial Type'] == trial_type].index)
            if not section.empty:
                next_max_row = section.loc[section['Error_Count'].idxmax()]
                return next_max_row['Trial Type']
            else:
                raise ValueError("No remaining trial types available with high error counts.")


    def generate_trial(self, id_value, df):
        start_chamber = self.find_start_chamber(id_value, df)
        trial = self.trial_types[start_chamber]

        # Update the count for the selected trial type
        self.trial_count[start_chamber] += 1

        return trial

    
    def _handle_browseBtn_clicked(self):
        pathDir = os.path.dirname((__file__))
        filter = "Text Files (*.xlsx)"  
        files,_ = QFileDialog.getOpenFileNames(None, "Select files to add", pathDir, filter)

        self.xlsxFiles = [os.path.basename(f) for f in files]
        self.xlsxDir = os.path.dirname(files[0])
        self._widget.pathDirEdit.setText(self.xlsxDir)

        if files:
            # Clear the list widget to remove any previous selections
            self._widget.xlsxFileListWidget.clear()

            # Add the selected files to the list widget
            self._widget.xlsxFileListWidget.addItems(self.xlsxFiles)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(files) > 1:
                self._widget.nextBtn.setEnabled(True)
                self._widget.previousBtn.setEnabled(True)
            else:
                self._widget.nextBtn.setEnabled(False)
                self._widget.previousBtn.setEnabled(False)

            # Save the list of selected files as an attribute of the class
            self.files = files

            # Connect the "Next" and "Previous" buttons tof their respective callback functions
            self._widget.previousBtn.clicked.connect(self._handle_previousBtn_clicked)
            self._widget.nextBtn.clicked.connect(self._handle_nextBtn_clicked)
            
    def _handle_xlsxFileListWidget_item_clicked(self):
        # Get the current file index from the list widget
        self.current_file_index = self._widget.xlsxFileListWidget.currentRow()

        # Set the current file in the list widget
        # self._widget.xlsxFileListWidget.setCurrentRow(self.current_file_index)

        # Get the full path of the selected file
        self.selected_file_path = os.path.join(self.xlsxDir, self.xlsxFiles[self.current_file_index])

        rospy.loginfo(f"Selected file: {self.xlsxFiles[self.current_file_index]}")

        # Load the file by passing the file path to load_xlsxfile
        self.load_xlsx_file(self.selected_file_path)
        self.display_excel_content(self.selected_file_path)

    def load_xlsx_file(self, file_path):
        # Load the xlsx file into a pandas dataframe
        self.df = pd.read_excel(file_path)
        self.trials = self.df.values.tolist()
        self.nTrials = len(self.trials)

    def _handle_nextBtn_clicked(self):
        # Increment the current file index
        self.current_file_index += 1

         # If we've reached the end of the list, loop back to the beginning
        if self.current_file_index >= len(self.files):f
        self._widget.xlsxFileListWidget.setCurrentRow(self.current_file_index)

    def _handle_previousBtn_clicked(self):
        # Decrement the current file index
        self.current_file_index -= 1

        # If we've reached the beginning of the list, loop back to the end
        if self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1

        # Set the current file in the list widget
        self._widget.xlsxFileListWidget.setCurrentRow(self.current_file_index)

    def _handle_trialListWidget_item_clicked(self):
        # Get the current trial index from the trial list widget
        self.current_trial_index = self._widget.trialListWidget.currentRow()

        # Set the current trial in the trial list widget
        self._widget.trialListWidget.setCurrentRow(self.current_trial_index)
        
    def _handle_nextBtn_2_clicked(self):
        # Increment the current trial index
        self.current_trial_index += 1

         # If we've reached the end of the list, loop back to the beginning
        if self.current_trial_index >= len(self.trials):
            self.current_trial_index = 0

        # Set the current trial in the trial list widget
        self._widget.trialListWidget.setCurrentRow(self.current_trial_index)

    def _handle_previousBtn_2_clicked(self):
        # Decrement the current trial index
        self.current_trial_index -= 1

        # If we've reached the beginning of the list, loop back to the end
        if self.current_trial_index < 0:
            self.current_trial_index = len(self.trials) - 1

        # Set the current trial in the trial list widget
        self._widget.trialListWidget.setCurrentRow(self.current_trial_index) 
    
    def _handle_startBtn_clicked(self):
        self.mode = Mode.START_EXPERIMENT
        
    def _handle_pauseBtn_clicked(self):
        # rospy.loginfo("Experiment paused")
        self.mode_before_pause = self.mode
        self.mode = Mode.PAUSE_EXPERIMENT

    def _handle_resumeBtn_clicked(self):
        # rospy.loginfo("Experiment resumed")
        self.mode = Mode.RESUME_EXPERIMENT

    def _handle_recordBtn_clicked(self, checked):
        #this function is called when the record button is clicked. It starts/stops recording data files.It saves all the ROS topics to a bag file.
        if not self.isRecording:   # Start recording
            self.dataDir = self._widget.recordDataDir.text()
            if not os.path.isdir(self.dataDir):
                self._widget.recordDataDir.setText(self.defaultDataDir)
                self.dataDir = self.defaultDataDir
                self._widget.recordDataDir.setText(self.dataDir)

            # Record all ROS topics to domeExperimentData.bag
            command_data = f"rosbag record -a -o plusMazeExperimentData"
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

    def _handle_pumpStartBtn_clicked(self):
        self.gantry_pub.publish("START_PUMP", [])

    def _handle_plusMazeBtn_clicked(self):
        self.setPlusConfig()

    def _handle_stopTrackingBtn_clicked(self):
        self.gantry_pub.publish("IDLE", [])

    def _handle_startTrackingBtn_clicked(self):
        self.gantry_pub.publish("TRACK_HARNESS", [])
    
    def _handle_homeBtn_clicked(self):
        self.gantry_pub.publish("HOME",[])

    def _handle_stopPumpBtn_clicked(self):
        self.gantry_pub.publish("STOP_PUMP",[])

    def _handle_ephysRatTogBtn_clicked(self):
        if self._widget.ephysRatTogBtn.isChecked():
            self.is_ephys_rat = True
        else:
            self.is_ephys_rat = False

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

    def _handle_trainingModeBtnGroup_clicked(self):
        if self._widget.trainingModeBtnGroup.checkedId() == 1:
            self.setForcedChoiceMode()
        elif self._widget.trainingModeBtnGroup.checkedId() == 2:
            self.setChoiceMode()

    def _handle_lowerAllDoorsBtn_clicked(self):
        self.setLowerConfig()

    def _handle_trialGeneratorBtn_clicked(self):
        self.trial_generator == True
        rospy.loginfo("Trial Generator enabled")  

    def _handle_whiteNoiseBtn_clicked(self):
        self.white_noise = True
        rospy.loginfo("White noise selected")    

    def _handle_fiveKhzBtn_clicked(self):
        self.five_khz = True
        rospy.loginfo("5 kHz selected")
       
    def _handle_triangleLeftBtn_clicked(self):
        self.triangle_left = True
        rospy.loginfo("Triangle left selected")     

    def _handle_triangleRightBtn_clicked(self):
        self.triangle_right = True
        rospy.loginfo("Triangle right selected")    
           

    #In the following functions, we define the starting maze configuration for each chamber. 
    #The starting maze configuration is defined by the chamber number, the walls that are present in the chamber.
    #In doing so the following wall map from omniroute_controller is used:
    #    WALL_MAP = {  # wall map for 3x3 maze [chamber_num][wall_num]
    #        0: [0, 1, 2, 3, 4, 5, 6, 7],
    #        1: [1, 2, 3, 5, 7],
    #        2: [0, 1, 2, 3, 4, 5, 6, 7],
    #        3: [0, 1, 3, 5, 7],
    #        4: [0, 1, 2, 3, 4, 5, 6, 7],
    #        5: [1, 3, 4, 5, 7],
    #        6: [0, 1, 2, 3, 4, 5, 6, 7],
    #        7: [1, 3, 5, 6, 7],
    #        8: [0, 1, 2, 3, 4, 5, 6, 7]
    #    }
    #The central chamber is chamber 4. The start_door, left_goal_door, right_goal_door, project_left_cue_wall, and project_right_cue_wall are defined as walls of chamber 4.

    def activateWalls(self):
        self.wallStates.chamber = -1
        self.wallStates.wall = [0]
        self.wallStates.state = True
        self.wallStates.send = True
        self.door_pub.publish(self.wallStates)
    
    def setPlusConfig(self):
        # Lower all walls
        for i in range(9):
            for j in range(8):
                self.lower_wall(Wall(i,j), False)
        
        for i in [1, 3, 4, 5, 7]:
            for j in range(8):
                self.raise_wall(Wall(i, j), False)
        
        self.activateWalls()

    def setLowerConfig(self):
        #Lower Walls 0,2,4,6 in chamber 4 (central chamber)
        for i in [0, 2, 4, 6]:
            self.lower_wall(Wall(4, i), False)
        self.activateWalls()

    
    def setChamberOneStartConfig(self):
        self.start_chamber = 1
        self.central_chamber = 4
        self.left_chamber = 5
        self.right_chamber = 3
        
        self.project_left_cue_triangle = 4 
        self.project_right_cue_triangle = 3
        
        self.start_wall = Wall(1, 6)
        self.left_goal_wall = Wall(4, 4)
        self.right_goal_wall = Wall(4, 0)

    def setChamberThreeStartConfig(self):
        self.start_chamber = 3
        self.central_chamber = 4
        self.left_chamber = 1
        self.right_chamber = 7

        self.project_left_cue_triangle = 2 
        self.project_right_cue_triangle = 1

        self.start_wall = Wall(3, 4)
        self.left_goal_wall = Wall(4, 2)
        self.right_goal_wall = Wall(4, 6)

    def setChamberFiveStartConfig(self):
        self.start_chamber = 5
        self.central_chamber = 4
        self.left_chamber = 7
        self.right_chamber = 1

        self.project_left_cue_triangle = 6
        self.project_right_cue_triangle = 5

        self.start_wall = Wall(5, 0)
        self.left_goal_wall = Wall(4, 6)
        self.right_goal_wall = Wall(4, 2)

    def setChamberSevenStartConfig(self):
        self.start_chamber = 7
        self.central_chamber = 4
        self.left_chamber = 3
        self.right_chamber = 5

        self.project_left_cue_triangle = 8
        self.project_right_cue_triangle = 7

        self.start_wall = Wall(7, 2)
        self.left_goal_wall = Wall(4, 0)
        self.right_goal_wall = Wall(4, 4)

    def setForcedChoiceMode(self):
        self.training_mode = "user_defined_forced_choice"

    def setChoiceMode(self):
        self.training_mode = "user_defined_choice"
           

    def display_excel_content(self, file_path):
        # Display the content in the QListWidget
        self._widget.trialListWidget.clear()
        for index, row in self.df.iterrows():
            row_list = row.tolist()  # Convert the row to a list
            item_text = ', '.join(map(str, row_list))
            self._widget.trialListWidget.addItem(item_text)


    def mazeboundary_marker0_callback(self, msg):
        self.mazeboundary_marker0 = np.array([msg.point.x, msg.point.y, msg.point.z])

    def mazeboundary_marker1_callback(self, msg):
        self.mazeboundary_marker1 = np.array([msg.point.x, msg.point.y, msg.point.z])
    
    def mazeboundary_marker2_callback(self, msg):
        self.mazeboundary_marker2 = np.array([msg.point.x, msg.point.y, msg.point.z])
    
    def mazeboundary_marker3_callback(self, msg):
        self.mazeboundary_marker3 = np.array([msg.point.x, msg.point.y, msg.point.z])
    
    def harness_pose_callback(self, msg):
        self.harness_x = msg.pose.position.x
        self.harness_y = msg.pose.position.y
    # print("Harness Pose: ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def rat_head_chamber_callback(self, msg):
        self.rat_head_chamber = msg.data

    def rat_body_chamber_callback(self, msg):
        self.rat_body_chamber = msg.data

    def run_experiment(self):

        self.current_time = rospy.Time.now()
        current_rat_chamber = self.rat_head_chamber

        # Run legacy state machine for ephys rat
        if self.is_ephys_rat:

            # Check if the rat has moved to a different chamber
            if current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1:
                # The rat has moved to a different chamber, update the gantry position
                self.move_gantry_to_chamber(current_rat_chamber)

                # Update the previous_rat_chamber for the next iteration
                self.previous_rat_chamber = current_rat_chamber
            
            if self.mode == Mode.START_EXPERIMENT:
                rospy.loginfo("START OF THE EXPERIMENT")

                #self.currentTrialNumber= -1
                self.currentStartConfig = self._widget.startChamberBtnGroup.checkedId()
                for button in self._widget.startChamberBtnGroup.buttons():
                    button.setEnabled(False)

                self.currentTrialNumber = self.current_trial_index-1
                #rospy.loginfo(f"Current trial number: {self.currentTrialNumber}")
                
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

                    self.left_visual_cue = self.currentTrial[0]
                    self.right_visual_cue = self.currentTrial[1]
                    self.training_mode = self.currentTrial[2]
                    
                    self.start_chamber = self._widget.startChamberBtnGroup.checkedId()
                    
                    if self.left_visual_cue == "Triangle":
                        self.projection_pub.publish(self.project_left_cue_triangle)
                        rospy.loginfo("Projecting left cue triangle")
                        self.success_chamber = self.left_chamber
                        #self.success_chamber_seq = self.left_chamber_seq
                        self.error_chamber = self.right_chamber
                        #self.error_chamber_seq = self.right_chamber_seq
                    else:  
                        self.projection_pub.publish(self.project_right_cue_triangle)
                        rospy.loginfo("Projecting right cue triangle")
                        self.success_chamber = self.right_chamber    
                        self.error_chamber = self.left_chamber
                            

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.RAT_IN_START_CHAMBER
                rospy.loginfo("RAT_IN_START_CHAMBER")


            elif self.mode == Mode.RAT_IN_START_CHAMBER:
                if (self.current_time - self.mode_start_time).to_sec() >= self.start_first_delay.to_sec():
                    #if self.training_mode is not None and self.training_mode in ["forced_choice", "user_defined_forced_choice"]: 
                    # if self.success_chamber == self.left_chamber:
                    #     self.lower_wall(self.left_goal_wall, send=True)
                    # else:
                    #     self.lower_wall(self.right_goal_wall, send=True)
                #elif self.training_mode is not None and self.training_mode in ["choice", "user_defined_choice"]:
                    self.lower_wall(self.left_goal_wall, send=False)
                    self.lower_wall(self.right_goal_wall, send=True)

                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.START
                    rospy.loginfo("START")
                    
            elif self.mode == Mode.START:
                if (self.current_time - self.mode_start_time).to_sec() >= self.start_second_delay.to_sec():
                    self.lower_wall(self.start_wall, send=True)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.START_TO_CHOICE
                    rospy.loginfo("START_TO_CHOICE")

            elif self.mode == Mode.START_TO_CHOICE:
                # Wait for the rat to move to the choice point
                if self.rat_head_chamber == self.central_chamber:
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE
                    rospy.loginfo("CHOICE")
            
            elif self.mode == Mode.CHOICE:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.CHOICE_TO_GOAL
                rospy.loginfo("CHOICE_TO_GOAL")

            elif self.mode == Mode.CHOICE_TO_GOAL:
                if self.rat_body_chamber == self.success_chamber:
                    self.raise_wall(self.left_goal_wall, send=True)
                    self.raise_wall(self.right_goal_wall, send=True)
                    self.raise_wall(self.start_wall, send=True)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.SUCCESS
                    rospy.loginfo("SUCCESS")

                elif self.rat_body_chamber == self.error_chamber:
                    self.sound_pub.publish("Error")
                    self.raise_wall(self.left_goal_wall, send=True)
                    self.raise_wall(self.right_goal_wall, send=True)
                    self.raise_wall(self.start_wall, send=True)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.ERROR
                    rospy.loginfo("ERROR")

            elif self.mode == Mode.SUCCESS:
                # self.success_center_x = self.chamber_centers[self.success_chamber][0]
                # self.success_center_y = self.chamber_centers[self.success_chamber][1]
                #if not self.is_ephys_rat:
                    #self.gantry_pub.publish("MOVE_TO_COORDINATE", [self.success_center_x, self.success_center_y])
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_START
                rospy.loginfo("REWARD START")

            elif self.mode == Mode.REWARD_START:
                if (self.current_time - self.mode_start_time).to_sec() >= self.reward_start_delay.to_sec():
                    #self.reward_dispense()
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
                        if self.success_chamber == 1:
                            self.setChamberOneStartConfig()
                        elif self.success_chamber == 3:
                            self.setChamberThreeStartConfig()
                        elif self.success_chamber  == 5:
                            self.setChamberFiveStartConfig()
                        elif self.success_chamber  == 7:
                            self.setChamberSevenStartConfig()
                            
                        self.mode_start_time = rospy.Time.now()
                        self.mode = Mode.END_TRIAL
                        rospy.loginfo("END TRIAL")
                    
            elif self.mode == Mode.ERROR:
                if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_delay.to_sec():
                    if self.error_chamber == 1:
                        self.setChamberOneStartConfig()
                    elif self.error_chamber == 3:
                        self.setChamberThreeStartConfig()
                    elif self.error_chamber  == 5:
                        self.setChamberFiveStartConfig()
                    elif self.error_chamber  == 7:
                        self.setChamberSevenStartConfig()
                    
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.END_TRIAL
                    rospy.loginfo("END TRIAL")

            elif self.mode == Mode.END_TRIAL:
                if (self.current_time - self.mode_start_time).to_sec() >= self.end_trial_delay.to_sec():
                    self.mode = Mode.START_TRIAL
                    rospy.loginfo("START_TRIAL")

        # Run current state machine for non-ephys rat
        else:

            # Check if the rat has moved to a different chamber
            if current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1:
                # The rat has moved to a different chamber, update the gantry position
                self.move_gantry_to_chamber(current_rat_chamber)

                # Update the previous_rat_chamber for the next iteration
                self.previous_rat_chamber = current_rat_chamber

            if self.mode == Mode.START_EXPERIMENT:
                rospy.loginfo("START OF THE EXPERIMENT")
           
                self.currentStartConfig = self._widget.startChamberBtnGroup.checkedId()

                self.currentTrialNumber = self.current_trial_index-1

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TRIAL

            elif self.mode == Mode.START_TRIAL:
                self.currentTrialNumber = self.currentTrialNumber+1
                rospy.loginfo(f"Current trial number: {self.currentTrialNumber}")
                
                trial = self.generate_trial(self.start_chamber, self.df)
                self.left_visual_cue = trial[1]
                self.right_visual_cue = trial[2]
                self.sound_cue = trial[3]

                rospy.loginfo(f"START OF TRIAL {[self.left_visual_cue, self.right_visual_cue, self.sound_cue]}") 
                
                # if self.trials and 0 <= self.currentTrialNumber < len(self.trials):
                #     self.currentTrial = self.trials[self.currentTrialNumber]
                # else:
                #     # Handle the case where trials is empty or currentTrialNumber is out of range
                #     self.currentTrial = None

                # rospy.loginfo(f"START OF TRIAL {self.currentTrial}")
            
                # if self.currentTrial is not None and self.currentTrialNumber >= self.nTrials:
                #     self.mode = Mode.END_EXPERIMENT

                # if self.currentTrial is not None:
                #     # Set training mode from file if the automatic mode is selected
                #     if self._widget.trainingModeBtnGroup.checkedId() == 3:
                #         self.training_mode = self.currentTrial[3]

                #     self.left_visual_cue = self.currentTrial[0]
                #     self.right_visual_cue = self.currentTrial[1]
                #     self.sound_cue = self.currentTrial[2]
                       

                if self.is_testing_phase:
                    self.play_sound_cue(self.sound_cue)
                else:
                    self.sound_cue_training_start = self.sound_cue + "_Training_Start"
                    self.sound_cue_training_stop = self.sound_cue + "_Training_Stop"
                    self.play_sound_cue(self.sound_cue_training_start)
                
                if self.sound_cue == "White_Noise":
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
                                self.lower_wall(self.left_goal_wall, send=True)
                            else:
                                self.lower_wall(self.right_goal_wall, send=True)
                        elif self.training_mode is not None and self.training_mode in ["choice", "user_defined_choice"]:
                            self.lower_wall(self.left_goal_wall, send=False)
                            self.lower_wall(self.right_goal_wall, send=True)
                    else:
                        self.lower_wall(self.left_goal_wall, send=True)
                        self.lower_wall(self.right_goal_wall, send=True)
                    
                    self.mode = Mode.START
                    rospy.loginfo("START")

            elif self.mode == Mode.START:
                if (self.current_time - self.mode_start_time).to_sec() >= self.start_second_delay.to_sec():
                    self.lower_wall(self.start_wall, send=True)
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
                    # if self.is_testing_phase:
                    #     self.play_sound_cue(self.sound_cue)
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
                    self.raise_wall(self.left_goal_wall, send=False)
                    self.raise_wall(self.right_goal_wall, send=False)
                    self.raise_wall(self.start_wall, send=True)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.SUCCESS
                    rospy.loginfo("SUCCESS")

                elif self.rat_body_chamber == self.error_chamber:
                    self.raise_wall(self.left_goal_wall, send=False)
                    self.raise_wall(self.right_goal_wall, send=False)
                    self.raise_wall(self.start_wall, send=True)
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
                # self.success_center_x = self.chamber_centers[self.success_chamber][0]
                # self.success_center_y = self.chamber_centers[self.success_chamber][1]
                #self.gantry_pub.publish("MOVE", [self.success_center_x, self.success_center_y])
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_START
                rospy.loginfo("REWARD_START")

            elif self.mode == Mode.REWARD_START:
                if (self.current_time - self.mode_start_time).to_sec() >= self.reward_start_delay.to_sec():
                    #self.reward_dispense()
                    # if self.is_testing_phase:
                    #     self.play_sound_cue(self.sound_cue)
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.REWARD_END
                    rospy.loginfo("REWARD END")

            elif self.mode == Mode.REWARD_END:
                if (self.current_time - self.mode_start_time).to_sec() >= self.reward_end_delay.to_sec():
                    #self.gantry_pub.publish("TRACK_HARNESS", [])
                    if not self.is_testing_phase:
                    #     self.play_sound_cue(self.sound_cue)
                    # else:
                        self.play_sound_cue(self.sound_cue_training_stop) 
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.POST_REWARD
                    rospy.loginfo("POST REWARD") 
    
            elif self.mode == Mode.POST_REWARD:
                if (self.current_time - self.mode_start_time).to_sec() >= self.right_choice_delay.to_sec():
                        if self.success_chamber == 1:
                            self.setChamberOneStartConfig()
                            rospy.loginfo("Chamber 1 selected")
                        elif self.success_chamber == 3:
                            self.setChamberThreeStartConfig()
                            rospy.loginfo("Chamber 3 selected")
                        elif self.success_chamber  == 5:
                            self.setChamberFiveStartConfig()
                            rospy.loginfo("Chamber 5 selected")
                        elif self.success_chamber  == 7:
                            self.setChamberSevenStartConfig()
                            rospy.loginfo("Chamber 7 selected")
                            
                        self.mode_start_time = rospy.Time.now()
                        self.mode = Mode.END_TRIAL
                        rospy.loginfo("END TRIAL")
                    
            elif self.mode == Mode.ERROR:
                if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_first_delay.to_sec():
                    if not self.is_testing_phase:
                    #     self.play_sound_cue(self.sound_cue)
                    # else:
                        self.play_sound_cue(self.sound_cue_training_stop) 
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.ERROR_END
                    rospy.loginfo("ERROR_END")

            elif self.mode == Mode.ERROR_END:
                if (self.current_time - self.mode_start_time).to_sec() >= self.wrong_choice_second_delay.to_sec():
                    if self.error_chamber == 1:
                        self.setChamberOneStartConfig()
                        rospy.loginfo("Chamber 1 selected")
                    elif self.error_chamber == 3:
                        self.setChamberThreeStartConfig()
                        rospy.loginfo("Chamber 3 selected")
                    elif self.error_chamber  == 5:
                        self.setChamberFiveStartConfig()
                        rospy.loginfo("Chamber 5 selected")
                    elif self.error_chamber  == 7:
                        self.setChamberSevenStartConfig()
                        rospy.loginfo("Chamber 7 selected")
                    
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.END_TRIAL
                    rospy.loginfo("END TRIAL")

            elif self.mode == Mode.PAUSE_EXPERIMENT:
                rospy.loginfo("PAUSE_EXPERIMENT")
                self.mode_start_time = rospy.Time.now()
                self._widget.pauseBtn.setEnabled(False)
                self._widget.resumeBtn.setEnabled(True)
                
            elif self.mode == Mode.RESUME_EXPERIMENT:
                rospy.loginfo("RESUME_EXPERIMENT")
                self._widget.pauseBtn.setEnabled(True)
                self._widget.resumeBtn.setEnabled(False)
                self.mode_start_time = rospy.Time.now()
                self.mode = self.mode_before_pause

            elif self.mode == Mode.END_TRIAL:
                if (self.current_time - self.mode_start_time).to_sec() >= self.end_trial_delay.to_sec():
                    self.mode = Mode.START_TRIAL
                    rospy.loginfo("START_TRIAL")

    def play_sound_cue(self, sound_cue):
        if self.is_ephys_rat:
            return
        rospy.loginfo(f"Play sound cue {sound_cue}")    
        if sound_cue == "White_Noise":
            self.sound_pub.publish("White_Noise")

        elif sound_cue == "White_Noise_Training_Start":
            self.sound_pub.publish("White_Noise_Training_Start")

        elif sound_cue == "White_Noise_Training_Stop":
            self.sound_pub.publish("White_Noise_Training_Stop")

        elif sound_cue == "5KHz":
            self.sound_pub.publish("5KHz")

        elif sound_cue == "5KHz_Training_Start":
            self.sound_pub.publish("5KHz_Training_Start")

        elif sound_cue == "5KHz_Training_Stop":
            self.sound_pub.publish("5KHz_Training_Stop")

        elif sound_cue == "Error":
            self.sound_pub.publish("Error")


    def raise_wall(self, wall, send):
        self.wallStates.chamber = wall.chamber_num
        self.wallStates.wall = [wall.wall_num]
        self.wallStates.state = True
        self.wallStates.send = send
        self.door_pub.publish(self.wallStates)

    def lower_wall(self, wall, send):
        self.wallStates.chamber = wall.chamber_num
        self.wallStates.wall = [wall.wall_num]
        self.wallStates.state = False
        self.wallStates.send = send
        self.door_pub.publish(self.wallStates)

    def reward_dispense(self):
        if self.is_ephys_rat:
            return
        self.gantry_pub.publish("REWARD", [4.0])

    def move_gantry_to_chamber(self, chamber_num):
        x = self.maze_dim.chamber_centers[chamber_num][0]
        y = self.maze_dim.chamber_centers[chamber_num][1]


if __name__ == '__main__':
    rospy.init_node('experiment_controller')
    Interface()
    rospy.spin()
