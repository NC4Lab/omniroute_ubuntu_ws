#!/usr/bin/env python
from shared_utils.ui_utilities import UIUtilities
from shared_utils.maze_debug import MazeDB
from shared_utils.wall_utilities import MazeDimensions, WallConfig
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
    START_TRIAL = auto()
    CHOICE_MADE = auto()
    CHOICE_INCORRECT = auto()
    CHOICE_CORRECT = auto()
    RAT_TO_CENTRE = auto()
    EXPERIMENT_END = auto()
    PAUSE_EXPERIMENT = auto()
    RESUME_EXPERIMENT = auto()
    TEST_MODE = auto()


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

        # Defining buttons in the scene
        # TODO change names
        self._widget.testingPhaseBtn.clicked.connect(
            self._handle_testingPhaseBtn_clicked)
        self._widget.trialGeneratorBtn.clicked.connect(
            self._handle_trialGeneratorBtn_clicked)
        self._widget.lowerAllDoorsBtn.clicked.connect(
            self._handle_lowerAllDoorsBtn_clicked)
        
        
        # buttons for each of the phases
        self._widget.phaseBtnGroup = QButtonGroup()
        self._widget.phaseBtnGroup.addButton(
            self._widget.testingBtn, id=0)
        self._widget.phaseBtnGroup.addButton(
            self._widget.habituation1Btn, id=1)
        self._widget.phaseBtnGroup.addButton(
            self._widget.habituatoin2Btn, id=2)
        self._widget.phaseBtnGroup.addButton(
            self._widget.fullTaskBtn, id=3)

        self._widget.phaseBtnGroup.setExclusive(True)
        for button in self._widget.phaseBtnGroup.buttons():
            button.setEnabled(True)

        self._widget.phaseBtnGroup.buttonClicked.connect(
            self._handle_phaseBtnGroup_clicked)

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
        self.trial_sub = rospy.Subscriber(
            '/selected_trial', String, self.trial_callback)

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
        # TODO change these
        
        self.common_functions = CommonFunctions()

        #phase variables
        self.is_testing_phase = False
        self.trial_generator = False
        self.is_habituation1_phase = False
        self.is_habituation2_phase = False
        self.is_fullTask_phase = False
        self.end_task = False
        self.start_task = False
        self.pause_task = False
        self.start_at_trial = False
        
        # rat variables
        self.rat_head_chamber = -1
        self.rat_body_chamber = -1
        self.previous_rat_chamber = -1
        self.rat_position = -1

        self.maze_dim = MazeDimensions()

        # Trial parameters
        self.save = {
            "trialNumber": [],
            "SelectedChamber": [],
            "correctChamber": [],
            "correct?": [],
            "startTime": [],
            "endTIme": []
        }

        self.currentTrial = 0
        self.prevTrial = -1
        self.numTrials = 32
        self.numBlocks = 4
        self.correctChambers = []
        self.ITI = 5
        self.punishTime = 10

        # Stimuli variables
        self.wall_blue_right = 8
        self.wall_img_green = 2
        self.choice_sound_cue = '1kHz_120s'

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

        # key is chamber number, value is chamber wall
        self.centreWalls = {
            4 : 0,
            4 : 1,
            4 : 2,
            4 : 3,
            4 : 4,
            4 : 5,
            4 : 6,
            4 : 7
        }

        # Mode parameters
        self.mode = Mode.TEST_MODE
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run_experiment)
        self.timer.start(10)

        # Common functions
        self.common_functions = CommonFunctions()

    # Defining functions of each button
    # TODO define these
    # phase buttons
    def _handle_testingPhaseBtn_clicked(self):
        self.is_testing_phase = True
        self.mode = Mode.TEST_MODE
        rospy.loginfo("Testing phase selected")
    def _handle_habituation1PhaseBtn_clicked(self):
        self.is_habituation1_phase = True
        rospy.loginfo("Habituation 1 phase selected")
    def _handle_habituation2PhaseBtn_clicked(self):
        self.is_habituation2_phase = True
        rospy.loginfo("Habituation 2 phase selected")
    def _handle_fullTaskPhaseBtn_clicked(self):
        self.is_fullTask_phase = True
        rospy.loginfo("Full task phase selected")
    def _handle_trialGeneratorBtn_clicked(self):
        rospy.loginfo("Generating trials")
        self.generateTrials()
    def _handle_lowerAllDoorsBtn_clicked(self):
        rospy.loginfo("Lowering all doors")
        self.lowerAllWalls()

    def generateTrials(self):
        goalChambers = self.goalChambers.keys()
        self.correctChambers = []
        for i in self.numBlocks:
            random.shuffle(goalChambers)
            self.correctChambers += goalChambers
        rospy.loginfo(self.correctChambers)

    def lowerWalls(self, walls):
        #lower designated central walls
        for chamber in walls.keys():
            for wall in walls[chamber]:
                self.common_functions.lower_wall(Wall(chamber, wall), False)
        self.common_functions.activateWalls()

    def raiseWalls(self, walls):
        #raise designated central walls
        for chamber in walls.keys():
            for wall in walls[chamber]:
                self.common_functions.raise_wall(Wall(chamber, wall), False)
        self.common_functions.activateWalls()

    def raiseAllWalls(self):
        for i in range(8):
            for j in range(8):
                self.common_functions.raiseWall(Wall(i, j), False)
        self.common_functions.activateWalls()

    def lowerAllWalls(self):
        for i in range(8):
            for j in range(8):
                self.common_functions.lowerWall(Wall(i, j), False)
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
        rospy.loginfo("EXPERIMENT RUNNING")
        self.current_time = rospy.Time.now()
        current_rat_chamber = self.rat_head_chamber

        if current_rat_chamber != self.previous_rat_chamber and current_rat_chamber != -1:
            # The rat has moved to a different chamber, update the gantry position
            self.common_functions.move_gantry_to_chamber(current_rat_chamber)

            # Update the previous_rat_chamber for the next iteration
            self.previous_rat_chamber = current_rat_chamber

        if self.mode == Mode.TEST_MODE:
            rospy.loginfo("TESTING MODE")
            self.raiseAllWalls()
            self.projection_wall_img_pub.publish(self.wall_blue_right)
            self.projection_pub.publish(json.dumps(Wall(3, 0).to_dict()))
            rospy.loginfo("Waiting 3 seconds")
            rospy.sleep(3)
            self.lowerAllWalls()

        elif self.mode == Mode.START_EXPERIMENT:
            rospy.loginfo("STARTING EXPERIMENT")
            # TODO generate and print all trials
            #raise all walls but 4 corner walls
            self.raiseAllWalls()
            self.lowerWalls(self.diagonalChambers)
            # TODO if something like a start button or ready button is clicked, then start
            self.mode = Mode.START_TRIAL
        
        elif self.mode == Mode.START_TRIAL:
            rospy.loginfo("STARTING TRIAL")

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

            #wait for ITI
            rospy.sleep(self.ITI)
            #lower centre walls
            self.lowerWalls(self.centreWalls)
            if current_rat_chamber in self.goalChambers.keys():
                self.mode = Mode.CHOICE_MADE
        
        elif self.mode == Mode.CHOICE_MADE:
            rospy.loginfo("CHOICE MADE")
            #raise wall corresponding to goal chamber
            self.raiseWalls(dict.fromkeys(current_rat_chamber, self.goalChambers[current_rat_chamber]))
            #visual and auditory cues
            #TODO check to see if chamber is correct or not each trial
            self.projection_wall_img_pub.publish(self.wall_img_green)
            self.sound_pub.publish(self.choice_sound_cue)

            #TODO check if choice correct or incorrect
            self.mode = Mode.CHOICE_INCORRECT
        
        elif self.mode == Mode.CHOICE_INCORRECT:
            rospy.loginfo("CHOICE INCORRECT")
            rospy.sleep(self.punishTime)
            # remove chamber from list of possible goal chambers and lower wall
            self.lowerWalls(dict.fromkeys(current_rat_chamber, self.goalChambers.pop(current_rat_chamber)))
            if current_rat_chamber in self.goalChambers.keys():
                self.mode = Mode.CHOICE_MADE
        
        elif self.mode == Mode.CHOICE_CORRECT:
            rospy.loginfo("CHOICE CORRECT")
            # TODO have gantry feed rat

        elif self.mode == Mode.RAT_TO_CENTRE:
            rospy.loginfo("MOVING RAT TO CENTRE")
            #remove centre wall facing rat from the list so we don't raise and lower the same wall in a row (might make more noise than necessary)
            self.goalChambersFromCentre.pop(current_rat_chamber)
            self.raiseWalls(self.goalChambersFromCentre)
            # remove chamber from list of possible goal chambers and lower wall
            self.lowerWalls(dict.fromkeys(current_rat_chamber, self.goalChambers.pop(current_rat_chamber)))
            #if the rat is in the centre and we're giving rewards for that, give reward
            if current_rat_chamber == 4:
                self.raiseWalls(self.centreWalls)
                if self.is_habituation1_phase or self.is_habituation2_phase:
                    rospy.loginfo("REWARDING FOR RETURNING TO CENTRE")
                    #TODO have gantry feed rat

                #TODO check if there are still trials left
                self.mode = Mode.START_TRIAL
                #if there aren't any, end the experiment
                self.mode = Mode.EXPERIMENT_END






if __name__ == '__main__':
    rospy.init_node('radial maze')
    Interface()
    rospy.spin()