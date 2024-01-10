#!/usr/bin/env python
import os,time
import rospy
from std_msgs.msg import String
from omniroute_operation.msg import *

import pandas as pd
from enum import Enum
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL
from PyQt5.QtWidgets import QGraphicsScene, QButtonGroup

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin


class Mode(Enum):
    START_EXPERIMENT = 0
    START_TRIAL = 1
    RAT_IN_START_CHAMBER = 2
    START_TO_CHOICE = 3
    CHOICE = 4
    CHOICE_TO_GOAL = 5
    SUCCESS = 6
    ERROR = 7
    END_TRIAL = 8
    END_EXPERIMENT = 9
    PAUSE_EXPERIMENT = 10
    RESUME_EXPERIMENT = 11


#class Trial:
 #   def __init__(self):
  #      self.file_path = 'Training_Trials.xlsx'
    #    self.df = pd.read_excel(self.file_path)
     #   self.trials = self.df.values.tolist()
       # self.nTrials = len(self.rows_list)


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
        
        rospy.logerr('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self.scene = QGraphicsScene()

        self._widget.pauseBtn.setEnabled(True)
        self._widget.resumeBtn.setEnabled(False)

        self._widget.startCellBtnGroup = QButtonGroup()
        self._widget.startCellBtnGroup.addButton(self._widget.cellOneBtn, id=1)
        self._widget.startCellBtnGroup.addButton(self._widget.cellThreeBtn, id=3)
        self._widget.startCellBtnGroup.addButton(self._widget.cellFiveBtn, id=5)
        self._widget.startCellBtnGroup.addButton(self._widget.cellSevenBtn, id=7)
        self._widget.startCellBtnGroup.setExclusive(True)

        self._widget.trainingModeBtnGroup = QButtonGroup()
        self._widget.trainingModeBtnGroup.addButton(self._widget.forcedChoiceBtn, id=1)
        self._widget.trainingModeBtnGroup.addButton(self._widget.choiceBtn, id=2)
        self._widget.trainingModeBtnGroup.addButton(self._widget.automaticBtn, id=3)
        self._widget.trainingModeBtnGroup.setExclusive(True)

        self._widget.browseBtn.clicked.connect(self._handle_browseBtn_clicked)
        self._widget.previousBtn.clicked.connect(self._handle_previousBtn_clicked)
        self._widget.nextBtn.clicked.connect(self._handle_nextBtn_clicked)
        self._widget.browseBtn.clicked.connect(self._handle_browseBtn_clicked)
        self._widget.nextBtn_2.clicked.connect(self._handle_nextBtn_2_clicked)
        self._widget.previousBtn_2.clicked.connect(self._handle_previousBtn_2_clicked)
        self._widget.startBtn.clicked.connect(self._handle_startBtn_clicked)
        self._widget.resumeBtn.clicked.connect(self._handle_resumeBtn_clicked)
        self._widget.pauseBtn.clicked.connect(self._handle_pauseBtn_clicked)
        self._widget.startCellBtnGroup.buttonClicked.connect(self._handle_startCellBtnGroup_clicked)
        self._widget.trainingModeBtnGroup.buttonClicked.connect(self._handle_trainingModeBtnGroup_clicked)
        self._widget.listWidget.itemClicked.connect(self._handle_listWidget_item_clicked)
        self._widget.excelListWidget.itemClicked.connect(self._handle_excelListWidget_item_clicked)

        self._widget.pathDirEdit.setText(
            os.path.expanduser(os.path.join('~', 'omniroute_ubuntu_ws', 'src', 'experiment_controller', 'interface')))

        #rospy.init_node('experiment_controller', anonymous=True)
        self.sound_pub = rospy.Publisher('sound_cmd', String, queue_size=1)
        self.door_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)
        self.projector_pub = rospy.Publisher('projector_cmd', String, queue_size=1)
        self.reward_pub = rospy.Publisher('reward_cmd', String, queue_size=1)

        # Experiment parameters
        self.start_wait_duration = rospy.Duration(5.0)  # Duration of delay in the beginning of the trial
        self.choice_wait_duration = rospy.Duration(15.0)  # Duration to wait for rat to move to the choice point
        self.reward_duration = rospy.Duration(20.0)  # Duration to dispense reward if the rat made the right choice
        self.wrong_choice_duration = rospy.Duration(40.0)  # Duration to wait if the rat made the wrong choice

        self.mode = Mode.START_EXPERIMENT
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        self.currentTrial = []
        self.currentTrialNumber = 0 
        self.nTrials = 0 
        self.trials = [] 

        self.currentStartConfig = 0

        self.current_file_index = 0
        self.current_trial_index = 0
        

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.run_experiment()
            r.sleep()


    def _handle_browseBtn_clicked(self):
        pathDir = os.path.expanduser(os.path.join('~','omniroute_ubuntu_ws', 'src', 'experimet_controller', 'interface'))
        filter = "Text Files (*.xlsx)"  
        files, _ = QFileDialog.getOpenFileNames(None, "Select files to add", pathDir, filter)

        if files:
            # Clear the list widget to remove any previous selections
            self._widget.listWidget.clear()

            # Add the selected files to the list widget
            self._widget.listWidget.addItems(files)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(files) > 1:
                self._widget.nextBtn.setEnabled(True)
                self._widget.previousBtn.setEnabled(True)
            else:
                self._widget.nextBtn.setEnabled(False)
                self._widget.previousBtn.setEnabled(False)

            # Save the list of selected files as an attribute of the class
            self.files = files

            # Connect the "Next" and "Previous" buttons to their respective callback functions
            self._widget.previousBtn.clicked.connect(self._handle_previousBtn_clicked)
            self._widget.nextBtn.clicked.connect(self._handle_nextBtn_clicked)
            
    def _handle_listWidget_item_clicked(self):
        # Get the current file index from the list widget
        self.current_file_index = self._widget.listWidget.currentRow()

        # Set the current file in the list widget
        self._widget.listWidget.setCurrentRow(self.current_file_index)

        # Get the full path of the selected file
        self.selected_file_path = os.path.expanduser(os.path.join('~', 'omniroute_ubuntu_ws', 'src', 'experimet_controller', 'interface'))

        # Load the file by passing the file path to load_csvfile
        self.load_csv_file(self.selected_file_path)
        self.display_excel_content(self.selected_file_path)


    def _handle_nextBtn_clicked(self):
        # Increment the current file index
        self.current_file_index += 1

         # If we've reached the end of the list, loop back to the beginning
        if self.current_file_index >= len(self.files):
            self.current_file_index = 0

        # Set the current file in the list widget
        self._widget.listWidget.setCurrentRow(self.current_file_index)


    def _handle_previousBtn_clicked(self):
        # Decrement the current file index
        self.current_file_index -= 1

        # If we've reached the beginning of the list, loop back to the end
        if self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1

        # Set the current file in the list widget
        self._widget.listWidget.setCurrentRow(self.current_file_index)


    def _handle_excelListWidget_item_clicked(self):
        # Get the current trial index from the excel list widget
        self.current_trial_index = self._widget.excelListWidget.currentRow()

        # Set the current trial in the excel list widget
        self._widget.excelListWidget.setCurrentRow(self.current_trial_index)
        

    def _handle_nextBtn_2_clicked(self):
        # Increment the current trial index
        self.current_trial_index += 1

         # If we've reached the end of the list, loop back to the beginning
        if self.current_trial_index >= len(self.trials):
            self.current_trial_index = 0

        # Set the current trial in the excel list widget
        self._widget.excelListWidget.setCurrentRow(self.current_trial_index)


    def _handle_previousBtn_2_clicked(self):
        # Decrement the current trial index
        self.current_trial_index -= 1

        # If we've reached the beginning of the list, loop back to the end
        if self.current_trial_index < 0:
            self.current_trial_index = len(self.trials) - 1

        # Set the current trial in the excel list widget
        self._widget.excelListWidget.setCurrentRow(self.current_trial_index) 

        
    def _handle_startBtn_clicked(self):
        self.mode = Mode.START_EXPERIMENT
        self.nTrials = self._widget.nTrialsEdit.text()
        

    def _handle_pauseBtn_clicked(self):
        rospy.loginfo("Experiment paused")
        self.mode_before_pause = self.mode
        self.mode = Mode.PAUSE_EXPERIMENT

    def _handle_resumeBtn_clicked(self):
        rospy.loginfo("Experiment resumed")
        self.mode = Mode.RESUME_EXPERIMENT
    
    def _handle_startCellBtnGroup_clicked(self):
        if self._widget.startCellBtnGroup.checkedId() == 1:
            self.setCellOneStartConfig()
        elif self._widget.startCellBtnGroup.checkedId() == 3:
            self.setCellThreeStartConfig()
        elif self._widget.startCellBtnGroup.checkedId() == 5:
            self.setCellFiveStartConfig()
        elif self._widget.startCellBtnGroup.checkedId() == 7:
            self.setCellSevenStartConfig()

    def _handle_trainingModeBtnGroup_clicked(self):
        if self._widget.trainingModeBtnGroup.checkedId() == 1:
            self.setForcedChoiceMode()
        elif self._widget.trainingModeBtnGroup.checkedId() == 2:
            self.setChoiceMode()


    #In the following functions, we define the starting maze configuration for each chamber. 
    #The starting maze configuration is defined by the chamber number, the walls that are present in the chamber.
    #In doing so the following wall map from three_by_three_interface is used:
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
    #For example, if the start_door is wall 2, it means chamber 4 wall 2.

    def setCellOneStartConfig(self):
        self.chambers_list = [1, 3, 4, 5]
        self.walls_list = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7]]
        self.left_chamber = [5]
        self.right_chamber = [3]
        self.start_door_open = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7]]
        self.start_door_close = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7]]
        self.project_left_cue_wall = [4]
        self.project_right_cue_wall = [0]
        self.left_goal_door_open = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3 ,5, 6, 7], [1, 3, 4, 5, 7]]
        self.left_goal_door_close = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7]]
        self.right_goal_door_open = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7]]
        self.right_goal_door_close = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7]]
        self.wallStates = WallState()
        self.wallStates.chamber = self.chambers_list
        self.wallStates.wall = self.walls_list
        self.wallStates.state = True
        self.door_pub.publish(self.wallStates)

    def setCellThreeStartConfig(self):
        self.chambers_list = [1, 3, 4, 7]
        self.walls_list = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 5, 6, 7]]
        self.left_chamber = [1]
        self.right_chamber = [7]
        self.start_door_open = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [1, 2, 3, 4 ,5, 6, 7], [1, 3, 5, 6, 7]]
        self.start_door_close = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 5, 6, 7]]
        self.project_left_cue_wall = [2]
        self.project_right_cue_wall = [6]
        self.left_goal_door_open = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 3, 4 ,5, 6, 7], [1, 3, 5, 6, 7]]
        self.left_goal_door_close = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 5, 6, 7]]
        self.right_goal_door_open = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 7], [1, 3, 5, 6, 7]]
        self.right_goal_door_close = [[1, 2, 3, 5, 7], [0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 5, 6, 7]]
        self.wallStates = WallState()
        self.wallStates.chamber = self.chambers_list
        self.wallStates.wall = self.walls_list
        self.wallStates.state = True
        self.door_pub.publish(self.wallStates)

    def setCellFiveStartConfig(self):
        self.chambers_list = [1, 4, 5, 7]
        self.walls_list = [[1, 2, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.left_chamber = [7]
        self.right_chamber = [1]
        self.start_door_open = [[1, 2, 3, 5, 7], [0, 1, 2, 3, 5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.start_door_close = [[1, 2, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.project_left_cue_wall = [6]
        self.project_right_cue_wall = [2]
        self.left_goal_door_open = [[1, 2, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.left_goal_door_close = [[1, 2, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.right_goal_door_open = [[1, 2, 3, 5, 7], [0, 1, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.right_goal_door_close = [[1, 2, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.wallStates = WallState()
        self.wallStates.chamber = self.chambers_list
        self.wallStates.wall = self.walls_list
        self.wallStates.state = True
        self.door_pub.publish(self.wallStates)

    def setCellSevenStartConfig(self):
        self.chambers_list = [3, 4, 5, 7]
        self.walls_list = [[0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.left_chamber = [3]
        self.right_chamber = [5]
        self.start_door_open = [[0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.start_door_close = [[0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.project_left_cue_wall = [1]
        self.project_right_cue_wall = [2]
        self.left_goal_door_open = [[0, 1, 3, 5, 7], [1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.left_goal_door_close = [[0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.right_goal_door_open = [[0, 1, 3, 5, 7], [0, 1, 2, 3 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.right_goal_door_close = [[0, 1, 3, 5, 7], [0, 1, 2, 3, 4 ,5, 6, 7], [1, 3, 4, 5, 7], [1, 3, 5, 6, 7]]
        self.wallStates = WallState()
        self.wallStates.chamber = self.chambers_list
        self.wallStates.wall = self.walls_list
        self.wallStates.state = True
        self.door_pub.publish(self.wallStates)


    def setForcedChoiceMode(self):
        if self._widget.startCellBtnGroup.checkedId() == 1:

        

    def setChoiceMode(self):


    def setAutomaticMode(self):


    def load_csv_file(self, file_path):
        # Load the csv file into a pandas dataframe
        self.df = pd.read_excel(file_path)
        self.trials = self.df.iloc[1:].values.tolist()
        self.nTrials = len(self.trials)

    def display_excel_content(self, file_path):
        # Display the content in the QListWidget
        self.excelListWidget.clear()
        for index, row in self.df.iterrows():
            row_list = row.tolist()  # Convert the row to a list
            item_text = ', '.join(map(str, row_list))
            self.listWidget.addItem(item_text)
         

    def run_experiment(self):
        self.current_time = rospy.Time.now()

        if self.mode == Mode.START_EXPERIMENT:
            rospy.loginfo("START OF THE EXPERIMENT")

            #self.currentTrialNumber= -1
            self.currentStartConfig = self._widget.startCellBtnGroup.checkedId()
            for button in self._widget.startCellBtnGroup.buttons():
                button.setEnabled(False)

            # Load starting maze config
            # Wait for experimenter signal
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.START_TRIAL

        elif self.mode == Mode.START_TRIAL:
            #self.currentTrialNumber = self.currentTrialNumber+1
            self.currentTrialNumber = self.current_trial_index 
            if self.trials and 0 <= self.currentTrialNumber < len(self.trials):
                self.currentTrial = self.trials[self.currentTrialNumber]
            else:
                # Handle the case where trials is empty or currentTrialNumber is out of range
                self.currentTrial = None
            rospy.loginfo(f"START OF TRIAL {self.currentTrial}")

            if self.currentTrial is not None and self.currentTrial >= self.nTrials:
                self.mode = Mode.END_EXPERIMENT

            # Load maze config according to animal location
            # Project cues

            # Play sound cue
            if self.currentTrial is not None:
                self.sound_cue = self.currentTrial[2]
                self.play_sound_cue(self.sound_cue)

                self.left_visual_cue = self.currentTrial[0]
                self.right_visual_cue = self.currentTrial[1]
                self.project_left_cue(self.left_visual_cue)
                self.project_right_cue(self.right_visual_cue)

                self.training_mode = self.currentTrial[3]

                self.start_chamber = self._widget.startCellBtnGroup.checkedId()
                
                if self.sound_cue == "white_noise":
                    if self.left_visual_cue == "triangle":
                        self.success_chamber = self.left_chamber
                        self.error_chamber = self.right_chamber
                    else:
                        self.success_chamber = self.right_chamber
                        self.error_chamber = self.left_chamber
                else:
                    if self.left_visual_cue == "square":
                        self.success_chamber = self.left_chamber
                        self.error_chamber = self.right_chamber
                    else:
                        self.success_chamber = self.right_chamber
                        self.error_chamber = self.left_chamber
                
                # Define self.rat_chamber    

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_IN_START_CHAMBER

        elif self.mode == Mode.RAT_IN_START_CHAMBER:
            if (self.current_time - self.mode_start_time).to_sec() >= self.start_wait_duration.to_sec():
                self.wallStates.wall = self.start_door_open
                self.door_pub.publish(self.wallStates)
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TO_CHOICE

        elif self.mode == Mode.START_TO_CHOICE:
            rospy.loginfo("START TO CHOICE")
            # Wait for the rat to move to the choice point
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.CHOICE

        elif self.mode == Mode.CHOICE:
            rospy.loginfo("CHOICE")
            self.wallStates.wall = self.start_door_close
            self.door_pub.publish(self.wallStates)

            if (self.current_time - self.mode_start_time).to_sec() >= self.choice_wait_duration.to_sec():
                self.door_activate()
                self.stop_sound_cue()
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.CHOICE_TO_GOAL

        elif self.mode == Mode.CHOICE_TO_GOAL:
            rospy.loginfo("CHOICE TO GOAL")
            if rat_made_right_choice:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.SUCCESS
            else:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.ERROR

        elif self.mode == Mode.SUCCESS:
            rospy.loginfo("SUCCESS")
            self.door_deactivate()
            self.reward_dispense()
            if (self.current_time - self.start_time).to_sec() == self.reward_duration.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.END_TRIAL
                
        elif self.mode == Mode.ERROR:
            self.door_deactivate()
            if (self.current_time - self.start_time).to_sec() == self.wrong_choice_duration.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.END_TRIAL

        elif self.mode == Mode.PAUSE_EXPERIMENT:
            rospy.loginfo("PAUSE_EXPERIMENT")
            self.mode_start_time = rospy.Time.now()
            self._widget.pauseBtn.setEnabled(False)
            self._widget.resumeBtn.setEnabled(True)


            #sender_button = self.sender()
            #if sender_button == self._widget.resumeBtn:
            #    self.mode == Mode.RESUME_EXPERIMENT
            
        elif self.mode == Mode.RESUME_EXPERIMENT:
            rospy.loginfo("RESUME_EXPERIMENT")
            self._widget.pauseBtn.setEnabled(True)
            self._widget.resumeBtn.setEnabled(False)
            self.mode_start_time = rospy.Time.now()
            self.mode = self.mode_before_pause

        elif self.mode == Mode.END_TRIAL:
            rospy.loginfo("END TRIAL")
            self.mode_start_time = rospy.Time.now()
            self.mode == Mode.START_TRIAL

    def play_sound_cue(self):
        if self.sound_cue == "white_noise":
            self.sound_pub.publish("white_noise")
        elif self.sound_cue == "5khz_tone":
            self.sound_pub.publish("5khz_tone")

    def stop_sound_cue(self):
        self.sound_pub.publish("stop_sound")

    def project_left_cue(self):
        self.projector_pub.publish("project_left_cue on the wall number ?")

    def project_right_cue(self):
        self.projector_pub.publish("project_right_cue on the wall number ?")

    def door_activate(self):
        self.door_pub.publish("open_start_door")

    def door_deactivate(self):
        self.door_pub.publish("close_start_door")

    def reward_dispense(self):
        self.reward_pub.publish("dispense_reward", [])


if __name__ == '__main__':
    rospy.init_node('experiment_controller')
    Interface()
    rospy.spin()