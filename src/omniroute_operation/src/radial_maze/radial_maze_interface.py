#!/usr/bin/env python
from shared_utils.ui_utilities import UIUtilities
from shared_utils.maze_debug import MazeDB
from shared_utils.wall_utilities import MazeDimensions, WallConfig
from shared_utils.projection_operation import ProjectionOperation
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

import json

class Mode(Enum):
    IDLE = auto()
    START_EXPERIMENT = auto()
    WAIT_FOR_START = auto()
    START_TRIAL = auto()
    TRIAL_ITI = auto()
    RAT_IS_WANDERING = auto()
    CHOICE_MADE = auto()
    PUBLISH_CUES = auto()
    CHOICE_INCORRECT = auto()
    CHOICE_CORRECT = auto()
    REWARD_TIME = auto()
    WRONG_REWARD = auto()
    WRONG_REWARD_TIME = auto()
    RAT_TO_CENTRE = auto()
    RAT_IN_CENTRE = auto()
    REWARD_FOR_CENTRE = auto()
    END_TRIAL = auto()
    END_EXPERIMENT = auto()
    PAUSE_EXPERIMENT = auto()
    RESUME_EXPERIMENT = auto()
    TEST_MODE = auto()
    OTHER_MODE = auto()


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

        self.scene = QGraphicsScene()

        # Start ProjectionOperation
        self.proj_op = ProjectionOperation()

        # Defining buttons in the scene
        self._widget.startBtn.clicked.connect(
            self._handle_startBtn_clicked)
        
        # put buttons in groups
        self.TESTING_PHASE_ID = 0
        self.HABITUATION1_PHASE_ID = 1
        self.HABITUATION2_PHASE_ID = 2
        self.FULLTASK_PHASE_ID = 3
        self._widget.phaseBtnGroup = QButtonGroup()
        self._widget.phaseBtnGroup.addButton(
            self._widget.testingPhaseBtn, id=self.TESTING_PHASE_ID)
        self._widget.phaseBtnGroup.addButton(
            self._widget.habituation1PhaseBtn, id=self.HABITUATION1_PHASE_ID)
        self._widget.phaseBtnGroup.addButton(
            self._widget.habituation2PhaseBtn, id=self.HABITUATION2_PHASE_ID)
        self._widget.phaseBtnGroup.addButton(
            self._widget.fullTaskPhaseBtn, id=self.FULLTASK_PHASE_ID)

        self._widget.phaseBtnGroup.setExclusive(True)
        for button in self._widget.phaseBtnGroup.buttons():
            button.setEnabled(True)
        
        self._widget.fullTaskPhaseBtn.setChecked(True)  # Default phase

        self._widget.phaseBtnGroup.buttonClicked.connect(
            self._handle_phaseBtnGroup_clicked)
        
        self.RADIAL_MAZE_ID = 0
        self.LOWER_ALL_DOORS_ID = 1
        self._widget.wallBtnGroup = QButtonGroup()
        self._widget.wallBtnGroup.addButton(
            self._widget.radialMazeBtn, id=self.RADIAL_MAZE_ID)
        self._widget.wallBtnGroup.addButton(
            self._widget.lowerAllDoorsBtn, id=self.LOWER_ALL_DOORS_ID)
        
        self._widget.wallBtnGroup.setExclusive(True)
        for button in self._widget.wallBtnGroup.buttons():
            button.setEnabled(True)

        self._widget.radialMazeBtn.setChecked(True)  # Default wall configuration

        self._widget.wallBtnGroup.buttonClicked.connect(
            self._handle_wallBtnGroup_clicked)

        # Setting rospy publishers
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
        # self.trial_sub = rospy.Subscriber(
        #     '/selected_trial', String, self.trial_callback)
        self.experiment_pub = rospy.Publisher(
            '/experiment', String, queue_size=1)

        # Define all Subscribers
        rospy.Subscriber('/mode', String, self.mode_callback, queue_size=1)
        rospy.Subscriber('/rat_head_chamber', Int8,
                         self.rat_head_chamber_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/rat_body_chamber', Int8,
                         self.rat_body_chamber_callback, queue_size=1, tcp_nodelay=True)

        # Time for setting up publishers and subscribers
        rospy.sleep(1.0)

        self.experiment_pub.publish("radial_maze_experiment")

        # setting variables
        
        self.common_functions = CommonFunctions()

        #phase variables
        self.is_testing_phase = False
        self.is_habituation1_phase = False
        self.is_habituation2_phase = False
        self.is_fullTask_phase = True
        self.start = False
        
        # rat variables
        self.rat_head_chamber = -1
        self.rat_body_chamber = -1
        self.previous_rat_chamber = -1

        self.maze_dim = MazeDimensions()

        # Trial parameters
        self.currentTrial = 0
        self.trials = []
        self.prevTrial = -1
        self.nTrials = 16 #a multiple of 8
        self.numBlocks = self.nTrials // 8
        self.correctChambers = []
        self.ITI = rospy.Duration(5)
        self.punishTime = rospy.Duration(10)
        self.gantryTime = rospy.Duration(3) #Time for gantry to move
        self.rewardTime = rospy.Duration(6)
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # Stimuli variables
        self.wall_blue = [6, 7, 8] # index values of the wall images
        self.wall_green = [9, 10, 11]
        self.wall_teal = [12, 13, 14]
        self.wall_black = [0, 0, 0]
        self.blue_sound = '1KHz' # name of sound clip to be played
        self.green_sound = '5KHz'
        self.teal_sound = '8KHz'
        self.correct_sound = 'Error'

        # Maze variables
        # key is chamber number, value is chamber wall pointing to middle
        self.goalChambers = {
            0 : 5,
            1 : 6,
            2 : 7,
            3 : 4,
            5 : 0,
            6 : 3,
            7 : 2,
            8 : 1
        }
        
        # key is chamber number, value is chamber wall pointing to middle
        self.diagonalChambers = {
            0 : 5,
            2 : 7,
            6 : 3,
            8 : 1
        }

        # key is goal chamber number, value is centre chamber wall pointing to goal chamber
        self.goalChambersFromCentre = {
            0 : 1,
            1 : 2,
            2 : 3,
            3 : 0,
            5 : 4,
            6 : 7,
            7 : 6,
            8 : 5
        }

        # key is goal chamber number, value is the 3 walls on the opposite side of the entrance from left to right
        self.projectionWalls = {
            0 : [0, 1, 2],
            1 : [1, 2, 3],
            2 : [2, 3, 4],
            3 : [7, 0, 1],
            5 : [3, 4, 5],
            6 : [6, 7, 0],
            7 : [5, 6, 7],
            8 : [4, 5, 6]
        }

        # key is correct chamber number, value is 2 bit chamber number
        self.twoBitChambers = {
            0 : 2,
            1 : 3,
            2 : 0,
            3 : 1,
            5 : 7,
            6 : 8,
            7 : 5,
            8 : 6
        }

        # key is goal chamber number, value is the stimuli to be projected
        self.stimuli = {
            0 : (self.wall_blue, self.blue_sound),
            1 : (self.wall_blue, self.blue_sound),
            2 : (self.wall_blue, self.blue_sound),
            3 : (self.wall_blue, self.blue_sound),
            4 : (self.wall_blue, self.blue_sound),
            5 : (self.wall_blue, self.blue_sound),
            6 : (self.wall_blue, self.blue_sound),
            7 : (self.wall_blue, self.blue_sound),
            8 : (self.wall_blue, self.blue_sound)
        }

        # Mode parameters
        self.mode = Mode.START_EXPERIMENT
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)

        # Common functions
        self.common_functions = CommonFunctions()

    # Defining functions of each button
    
    def _handle_phaseBtnGroup_clicked(self):
        if self._widget.phaseBtnGroup.checkedId() == self.TESTING_PHASE_ID:
            self.is_testing_phase = True
            self.is_habituation1_phase = False
            self.is_habituation2_phase = False
            self.is_fullTask_phase = False
            self.mode = Mode.TEST_MODE
            rospy.loginfo("Testing phase selected")
        elif self._widget.phaseBtnGroup.checkedId() == self.HABITUATION1_PHASE_ID:
            self.is_testing_phase = False
            self.is_habituation1_phase = True
            self.is_habituation2_phase = False
            self.is_fullTask_phase = False
            self.mode = Mode.START_EXPERIMENT
            rospy.loginfo("Habituation 1 phase selected")
        elif self._widget.phaseBtnGroup.checkedId() == self.HABITUATION2_PHASE_ID:
            self.is_testing_phase = False
            self.is_habituation1_phase = False
            self.is_habituation2_phase = True
            self.is_fullTask_phase = False
            self.mode = Mode.START_EXPERIMENT
            rospy.loginfo("Habituation 2 phase selected")
        elif self._widget.phaseBtnGroup.checkedId() == self.FULLTASK_PHASE_ID:
            self.is_testing_phase = False
            self.is_habituation1_phase = False
            self.is_habituation2_phase = False
            self.is_fullTask_phase = True
            self.mode = Mode.START_EXPERIMENT
            rospy.loginfo("Full task phase selected")
        
    def _handle_wallBtnGroup_clicked(self):
        if self._widget.wallBtnGroup.checkedId() == self.RADIAL_MAZE_ID:
            rospy.loginfo("Assuming radial maze configuration")
            self.raiseAllWalls()
            self.lowerWalls(self.diagonalChambers)
        elif self._widget.wallBtnGroup.checkedId() == self.LOWER_ALL_DOORS_ID:
            rospy.loginfo("Lowering all doors")
            self.lowerAllWalls()
    
    def _handle_startBtn_clicked(self):
        rospy.loginfo("Start clicked")
        self.start = True
    
    # function definitions

    def generateTrials(self):
        goalChambers = list(self.goalChambers.keys())
        self.correctChambers = []
        for i in range(self.numBlocks):
            random.shuffle(goalChambers)
            self.correctChambers += goalChambers
        rospy.loginfo(self.correctChambers)

    def lowerWalls(self, walls):
        for chamber in walls.keys():
            self.common_functions.lower_wall(Wall(chamber, walls[chamber]), False)
        self.common_functions.activateWalls()

    def lowerCentreWalls(self):
        for i in range(8):
            self.common_functions.lower_wall(Wall(4, i), False)
        self.common_functions.activateWalls()

    def raiseCentreWalls(self):
        for i in range(8):
            self.common_functions.raise_wall(Wall(4, i), False)
        self.common_functions.activateWalls()

    def raiseWalls(self, walls):
        rospy.loginfo(type(walls))
        #raise designated central walls
        for chamber in walls.keys():
            self.common_functions.raise_wall(Wall(chamber, walls[chamber]), False)
        self.common_functions.activateWalls()

    def raiseAllWalls(self):
        for i in range(9):
            for j in range(8):
                self.common_functions.raise_wall(Wall(i, j), False)
        self.common_functions.activateWalls()

    def lowerAllWalls(self):
        for i in range(8):
            for j in range(8):
                self.common_functions.lower_wall(Wall(i, j), False)
        self.common_functions.activateWalls()

    # Define data revieved from gantry
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

    def run_experiment(self):
    # This funtion loops
        self.current_time = rospy.Time.now()
        current_rat_chamber = self.rat_body_chamber

        if (self.start) and (current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1):
        #if (current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1):
            # The rat has moved to a different chamber, update the gantry position
            self.common_functions.move_gantry_to_chamber(current_rat_chamber)

            # Update the previous_rat_chamber for the next iteration
            self.previous_rat_chamber = current_rat_chamber

        if self.mode == Mode.TEST_MODE:
            self.mode_start_time = rospy.Time.now()
            rospy.loginfo("TESTING MODE")

        elif self.mode == Mode.START_EXPERIMENT:
            rospy.loginfo("STARTING EXPERIMENT")
            #generate and print all trials
            self.generateTrials()
            #raise all walls but 4 corner walls
            self.raiseAllWalls()
            self.lowerWalls(self.diagonalChambers)
    
            self.mode = Mode.WAIT_FOR_START

        elif self.mode == Mode.WAIT_FOR_START:
            #wait for user to press start button on UI
            if self.start == True:
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TRIAL
        
        elif self.mode == Mode.PAUSE_EXPERIMENT:
            rospy.loginfo("PAUSE_EXPERIMENT")
            self.button_pub.publish("Pause_button_disabled")
            self.mode_start_time = rospy.Time.now()

        elif self.mode == Mode.RESUME_EXPERIMENT:
            rospy.loginfo("RESUME_EXPERIMENT")
            self.button_pub.publish("Pause_button_enabled")
            self.mode_start_time = rospy.Time.now()
            self.mode = self.mode_before_pause

        elif self.mode == Mode.START_TRIAL:
            # End experiment if there are no more trials from the predefined number of total trials
            if self.currentTrial >= self.nTrials:
                self.mode = Mode.END_EXPERIMENT

            self.prevTrial = self.currentTrial
            self.currentTrial += 1
            
            rospy.loginfo(f"STARTING TRIAL {self.currentTrial}")

            # reset end chambers
            self.endChambers = self.goalChambers.copy()

            self.correctChamber = self.correctChambers[self.currentTrial-1]

            # key is goal chamber number, value is the stimuli to be projected
            self.stimuli = {
                0 : (self.wall_blue, self.blue_sound),
                1 : (self.wall_blue, self.blue_sound),
                2 : (self.wall_blue, self.blue_sound),
                3 : (self.wall_blue, self.blue_sound),
                4 : (self.wall_blue, self.blue_sound),
                5 : (self.wall_blue, self.blue_sound),
                6 : (self.wall_blue, self.blue_sound),
                7 : (self.wall_blue, self.blue_sound),
                8 : (self.wall_blue, self.blue_sound)
            }

            # set stimuli to be projected for the chambers with more than one bit of information
            self.stimuli[8 - self.correctChamber] = (self.wall_green, self.green_sound)
            self.stimuli[self.twoBitChambers[self.correctChamber]] = (self.wall_teal, self.teal_sound)
            self.stimuli[8 - self.twoBitChambers[self.correctChamber]] = (self.wall_teal, self.teal_sound)
            self.stimuli[self.correctChamber] = (self.wall_black, self.correct_sound)

            self.proj_op.blank_maze(publish=True)  # Start with a blank maze

            self.mode_start_time = rospy.Time.now()
            rospy.loginfo("Waiting for ITI")
            self.mode = Mode.TRIAL_ITI

        elif self.mode == Mode.TRIAL_ITI:
            #wait for ITI
            if (self.current_time - self.mode_start_time).to_sec() >= self.ITI.to_sec():
                #When the ITI is over,
                rospy.loginfo("ITI over")
                #lower centre walls
                self.lowerCentreWalls()

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.RAT_IS_WANDERING
        
        elif self.mode == Mode.RAT_IS_WANDERING:
                #wait for rat to go to a end chamber
                if current_rat_chamber in self.endChambers.keys():
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE_MADE
        
        elif self.mode == Mode.CHOICE_MADE:
            rospy.loginfo("CHOICE MADE")
            #raise wall corresponding to end chamber
            self.common_functions.raise_wall(Wall(current_rat_chamber, self.endChambers[current_rat_chamber]), False)
            self.common_functions.activateWalls()

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.PUBLISH_CUES

        elif self.mode == Mode.PUBLISH_CUES:
            #publish visual and auditory cues
            rospy.loginfo("projection mode entered")

            if (self.is_fullTask_phase or self.is_habituation2_phase):
                if (current_rat_chamber != self.correctChamber):
                    index = 0
                    for wall in self.projectionWalls[current_rat_chamber]:
                            self.proj_op.set_wall_image(chamber=current_rat_chamber,
                                                        wall=wall,
                                                        image_index=self.stimuli[current_rat_chamber][0][index],
                                                        publish=False)
                            index += 1
                    self.proj_op.publish_image_message()
                self.sound_pub.publish(self.stimuli[current_rat_chamber][1])
                rospy.loginfo("Projecting images and playing sound")

            if (current_rat_chamber == self.correctChamber) or self.is_habituation1_phase:
                rospy.loginfo(f"Trial {self.currentTrial}: correct. Chamber: {current_rat_chamber}")
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.CHOICE_CORRECT
            else:
                rospy.loginfo(f"Trial {self.currentTrial}: incorrect. Chamber: {current_rat_chamber}")
                if (self.is_habituation2_phase):
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.WRONG_REWARD
                else:
                    rospy.loginfo("CHOICE INCORRECT, waiting for punish time")
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE_INCORRECT
        
        elif self.mode == Mode.CHOICE_INCORRECT:
            if (self.current_time - self.mode_start_time).to_sec() >= self.punishTime.to_sec():
                #punish time over
                rospy.loginfo("punishment over")
                # remove chamber from list of possible end chambers and lower wall
                self.common_functions.lower_wall(Wall(current_rat_chamber, self.endChambers.pop(current_rat_chamber)), False)
                self.common_functions.activateWalls()

                self.mode_start_time = rospy.Time.now()
                rospy.loginfo("rat is wandering")
                self.mode = Mode.RAT_IS_WANDERING
            
        
        elif self.mode == Mode.CHOICE_CORRECT:
            #wait for gantry to be over the rat
            if (self.current_time - self.mode_start_time).to_sec() >= self.gantryTime.to_sec():
                rospy.loginfo("REWARDING FOR CORRECT CHOICE")
                # have gantry feed rat. Check phase for how much it should be fed
                if self.is_habituation1_phase:
                    self.gantry_pub.publish("deliver_reward", [0.5]) # Send with pump duration (sec)
                elif self.is_habituation2_phase or self.is_fullTask_phase:
                    self.gantry_pub.publish("deliver_reward", [1]) # Send with pump duration (sec)

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.REWARD_TIME

        elif self.mode == Mode.REWARD_TIME:
            #wait for reward time
            if (self.current_time - self.mode_start_time).to_sec() >= self.rewardTime.to_sec():
                rospy.loginfo("REWARD TIME OVER")
                
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.RAT_TO_CENTRE

        elif self.mode == Mode.WRONG_REWARD:
            #wait for gantry to be over the rat
            if (self.current_time - self.mode_start_time).to_sec() >= self.gantryTime.to_sec():
                rospy.loginfo("GIVING HALF REWARD FOR INCORRECT CHOICE")
                self.gantry_pub.publish("deliver_reward", [0.5])

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.WRONG_REWARD_TIME
        
        elif self.mode == Mode.WRONG_REWARD_TIME:
            #wait for punish time
            if (self.current_time - self.mode_start_time).to_sec() >= self.punishTime.to_sec():
                rospy.loginfo("PUNISH REWARD TIME OVER")
                # remove chamber from list of possible end chambers and lower wall
                self.common_functions.lower_wall(Wall(current_rat_chamber, self.endChambers.pop(current_rat_chamber)), False)
                self.common_functions.activateWalls()

                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.RAT_IS_WANDERING

        elif self.mode == Mode.RAT_TO_CENTRE:
            rospy.loginfo("MOVING RAT TO CENTRE")
            self.proj_op.blank_maze(publish=True)  # Start with a blank maze
            self.raiseCentreWalls()
            self.common_functions.lower_wall(Wall(4, self.goalChambersFromCentre.get(current_rat_chamber)), False)
            # remove chamber from list of possible end chambers and lower wall
            self.common_functions.lower_wall(Wall(current_rat_chamber, self.endChambers.pop(current_rat_chamber)), False)
            self.common_functions.activateWalls()

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_IN_CENTRE

        elif self.mode == Mode.RAT_IN_CENTRE:
            #if the rat is in the centre and we're giving rewards for that, give reward
            if current_rat_chamber == 4:
                rospy.loginfo("RAT IN CENTRE")
                self.raiseCentreWalls()
                if self.is_habituation1_phase or self.is_habituation2_phase:
                    rospy.loginfo(f"yes {self.is_habituation1_phase}")
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.REWARD_FOR_CENTRE
                else:
                    #check to see if there are still trials left
                    if self.currentTrial >= self.nTrials:
                        self.mode_start_time = rospy.Time.now()
                        self.mode = Mode.END_EXPERIMENT
                    else:
                        self.mode_start_time = rospy.Time.now()
                        self.mode = Mode.START_TRIAL

        elif self.mode == Mode.REWARD_FOR_CENTRE:
            #wait for gantry to be over the rat
            if (self.current_time - self.mode_start_time).to_sec() >= self.gantryTime.to_sec():
                rospy.loginfo("REWARDING FOR RETURNING TO CENTRE")
                #give half reward
                self.gantry_pub.publish("deliver_reward", [0.5])
                
                if ((self.is_habituation2_phase) and (current_rat_chamber != 4)):
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.CHOICE_INCORRECT
                else:
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.END_TRIAL
        
        elif self.mode == Mode.END_TRIAL:
            #check to see if there are still trials left
                if self.currentTrial >= self.nTrials:
                    rospy.loginfo("EXPERIMENT END")
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.END_EXPERIMENT
                else:
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.START_TRIAL

        elif self.mode == Mode.END_EXPERIMENT:
            rospy.loginfo("EXPERIMENT END")







if __name__ == '__main__':
    rospy.init_node('radial maze')
    Interface()
    rospy.spin()