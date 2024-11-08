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
from omniroute_controller.omniroute_controller_interface import MazeDimensions
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
    
    # Add to_dict method
    def to_dict(self):
        return {
            'chamber_num': self.chamber_num,
            'wall_num': self.wall_num
        }
    
    # Add from_dict method
    @classmethod
    def from_dict(cls, data):
        return cls(data['chamber_num'], data['wall_num'])

class Interface(Plugin):
    def __init__(self, context, add = True):
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
        if add:
            context.add_widget(self._widget)

        self.scene = QGraphicsScene()

        self._widget.pauseBtn.setEnabled(True)
        self._widget.resumeBtn.setEnabled(False)

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
        self._widget.trainingModeBtnGroup.buttonClicked.connect(self._handle_trainingModeBtnGroup_clicked)
        self._widget.xlsxFileListWidget.itemClicked.connect(self._handle_xlsxFileListWidget_item_clicked)
        self._widget.trialListWidget.itemClicked.connect(self._handle_trialListWidget_item_clicked)
        self._widget.browseBtn_2.clicked.connect(self._handle_browseBtn_2_clicked)
        self._widget.recordBtn.clicked[bool].connect(self._handle_recordBtn_clicked)

        self.curDir = os.path.dirname(__file__)

        self._widget.pathDirEdit.setText(self.curDir)

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

        self.write_sync_ease_pub = rospy.Publisher('/Esmacat_write_sync_ease', ease_registers, queue_size=1)
        self.sync_pub = rospy.Publisher('/sync_cmd', SyncCmd, queue_size=1)
        self.trial_pub = rospy.Publisher('/selected_trial', String, queue_size=10)

        self.mode_pub = rospy.Publisher('/mode', String, queue_size=1)

        rospy.Subscriber('/button', String, self.button_callback, queue_size=1)
        rospy.Subscriber('/experiment', String, self.experiment_callback, queue_size=1)   

        # Time for setting up publishers and subscribers
        rospy.sleep(1.0)

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

        self.currentTrial = []
        self.currentTrialNumber = 0 
        self.nTrials = 0 
        self.trials = [] 
        
        self.timer = QTimer(self)
        self.timer.start(10)


    def experiment_callback(self, msg):
        experiment = msg.data
        if experiment == "rule_based_experiment":
            self.experiment_type = "rule_based_experiment"
            rospy.loginfo("Rule-based experiment selected")    
        elif experiment == "single_T_maze_experiment":
            self.experiment_type = "single_T_maze_experiment"
            rospy.loginfo("Single T-maze experiment selected")

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
        trial_data = {
            # Selected trial data
            'trials': self.trials,  # List of trials
            'nTrials': self.nTrials,  # Number of trials
            'trial': self.trials[self.current_trial_index],
            'current_trial_index': self.current_trial_index  # Current trial index
        }

        # Serialize the dictionary to a JSON string
        selected_trial_json = json.dumps(trial_data)
        self.trial_pub.publish(selected_trial_json)
        #rospy.loginfo(f"Published selected trial: {selected_trial_json}")
        
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
        self.mode_pub.publish("START_EXPERIMENT")
        
    def _handle_pauseBtn_clicked(self):
        self.mode_pub.publish("PAUSE_EXPERIMENT")

    def _handle_resumeBtn_clicked(self):
        self.mode_pub.publish("RESUME_EXPERIMENT")


    def _handle_recordBtn_clicked(self, checked):
        #this function is called when the record button is clicked. It starts/stops recording data files.It saves all the ROS topics to a bag file.
        if not self.isRecording:   # Start recording
            self.dataDir = self._widget.recordDataDir.text()
            if not os.path.isdir(self.dataDir):
                self._widget.recordDataDir.setText(self.defaultDataDir)
                self.dataDir = self.defaultDataDir
                self._widget.recordDataDir.setText(self.dataDir)

            if self.experiment_type == "rule_based_experiment":
                # Record all ROS topics to domeExperimentData.bag
                command_data = f"rosbag record -a -o ruleBasedExperimentData"
            elif self.experiment_type == "single_T_maze_experiment":
                # Record all ROS topics to domeExperimentData.bag
                command_data = f"rosbag record -a -o singleTmazeExperimentData"
            else:
                rospy.logerr(f"Unknown experiment type: {self.experiment_type}")
                return
                
            self.recordDataPid = subprocess.Popen(command_data, shell=True, cwd=self.dataDir)
            # Pause for 3 seconds to allow the bag file to be created
            rospy.sleep(3)

            # Send message to send positive TTL output to Optitrack eSync2 which is handled by the sync_sender node
            self.sync_pub.publish("start_optitrack_sync", rospy.Time.now())
            
            self._widget.recordBtn.setStyleSheet("background-color: red; color: yellow")
            self._widget.recordBtn.setText("Stop")
            rospy.loginfo('Recording data files')
            self.isRecording = 1

        else:   # Stop recording
            # Send message to send negative TTL output to Optitrack eSync2 which is handled by the sync_sender node
            self.sync_pub.publish("stop_optitrack_sync", rospy.Time.now())

            rospy.sleep(1)

            self.terminate_ros_node("/record")

            self._widget.recordBtn.setStyleSheet("background-color: green; color: yellow")
            self._widget.recordBtn.setText("Record")
            rospy.loginfo('Stopping recording')

            self.isRecording = 0

    def _handle_browseBtn_2_clicked(self):
        res = QFileDialog.getExistingDirectory(None,"Select directory for recording",self.dataDir,QFileDialog.ShowDirsOnly)
        self._widget.recordDataDir.setText(res)
    
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

    def _handle_trainingModeBtnGroup_clicked(self):
        if self._widget.trainingModeBtnGroup.checkedId() == 1:
            self.setForcedChoiceMode()
        elif self._widget.trainingModeBtnGroup.checkedId() == 2:
            self.setChoiceMode()

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

    def button_callback(self, msg):
        button_mode = msg.data
        if button_mode == "Pause_button_disabled":
            self._widget.pauseBtn.setEnabled(False)
            self._widget.resumeBtn.setEnabled(True)
        elif button_mode == "Pause_button_enabled":
            self._widget.pauseBtn.setEnabled(True)
            self._widget.resumeBtn.setEnabled(False)

    def experiment_callback(self, msg):
        experiment = msg.data
        if experiment == "rule_based_experiment":
            self.experiment_type = "rule_based_experiment"
            rospy.loginfo("Rule-based experiment selected")    
        elif experiment == "single_T_maze_experiment":
            self.experiment_type = "single_T_maze_experiment"
            rospy.loginfo("Single T-maze experiment selected")


class CommonFunctions:
    def __init__(self):

        self.gantry_pub = rospy.Publisher('/gantry_cmd', GantryCmd, queue_size=1)
        self.door_pub = rospy.Publisher('/wall_state_cmd', WallState, queue_size=200)

        self.maze_dim = MazeDimensions()
        self.wallStates = WallState()

    def activateWalls(self):
        self.wallStates.chamber = -1
        self.wallStates.wall = [0]
        self.wallStates.state = True
        self.wallStates.send = True
        self.door_pub.publish(self.wallStates)

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
        self.gantry_pub.publish("deliver_reward", [4.0]) # Send with pump duration (sec)

    def move_gantry_to_chamber(self, chamber_num):
        x = self.maze_dim.chamber_centers[chamber_num][0]
        y = self.maze_dim.chamber_centers[chamber_num][1]


if __name__ == '__main__':
    rospy.init_node('experiment_controller')
    Interface()
    rospy.spin()
