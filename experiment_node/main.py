#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pandas as pd
from enum import Enum
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin


class Mode(Enum):
    START_EXPERIMENT = 0
    START_TRIAL = 1
    RAT_IN_START_CHAMBER = 2
    START_TO_CHOICE = 3
    CHOICE = 4
    CHOICE_TO_GOAL = 5
    END_TRIAL = 6
    END_EXPERIMENT = 7
    PAUSE_EXPERIMENT = 8
    RESUME_EXPERIMENT = 9


class Trial:
    def __init__(self):
        self.file_path = 'Training_Trials.xlsx'
        self.df = pd.read_excel(self.file_path)
        self.column_lists = [self.df[col].tolist() for col in self.df.columns]
        self.left_visual_cue = self.column_lists[0]
        self.right_visual_cue = self.column_lists[1]
        self.sound_cue = self.column_lists[2]


class Interface(Plugin):
    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None
        self.setObjectName('Interface')

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self.SCREEN_WIDTH = 720
        self.SCREEN_HEIGHT = 72

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'experiment.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names

        rospy.logerr('Test Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.experimentView.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.scene = QGraphicsScene()
        self._widget.experimentViewView.setScene(self.scene)

        # Set the size of the scene and the view
        self._widget.experimentView.setFixedSize(self.SCREEN_WIDTH, self.SCREEN_HEIGHT)
        self._widget.experimentView.setSceneRect(0, 0, self.SCREEN_WIDTH, self.SCREEN_HEIGHT)

        # Set the background color of the scene to gray
        self._widget.experimentView.setBackgroundBrush(QColor(0, 0, 0))

        self._widget.experimentView.setViewport(QtOpenGL.QGLWidget())
        # self._widget.mazeView.update()

        self._widget.pathBrowseBtn.clicked.connect(self._handle_pathBrowseBtn_clicked)
        # self._widget.pathPreviousBtn.clicked.connect(self._handle_pathPreviousBtn_clicked)
        # self._widget.pathNextBtn.clicked.connect(self._handle_pathNextBtn_clicked)

        self._widget.pathDirEdit.setText(
            os.path.expanduser(os.path.join('~', 'catkin_ws', 'src', 'maze_interface', 'src', 'maze_interface')))


class MazeExperimentController:
    def __init__(self):
        rospy.init_node('maze_experiment_node', anonymous=True)

        self.sound_pub = rospy.Publisher('sound_command', String, queue_size=1)
        self.door_pub = rospy.Publisher('door_command', String, queue_size=1)
        self.projector_pub = rospy.Publisher('projector_command', String, queue_size=1)

        # Experiment parameters
        self.start_wait_duration = rospy.Duration(5.0)  # Duration of delay in the beginning of the trial
        self.choice_wait_duration = rospy.Duration(15.0)  # Duration to wait for rat to move to the choice point
        self.reward_duration = rospy.Duration(20.0)  # Duration to dispense reward if the rat made the right choice
        self.wrong_choice_duration = rospy.Duration(40.0)  # Duration to wait if the rat made the wrong choice

        self.mode = Mode.START_EXPERIMENT
        self.mode_start_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        self.currentTrial = 0
        self.nTrials

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.run_experiment()
            r.sleep()

    def run_experiment(self):
        self.current_time = rospy.Time.now()

        if self.mode == Mode.START_EXPERIMENT:
            rospy.loginfo("START OF THE EXPERIMENT")
            # Load trial file
            # populate a list of trials
            # populate nTrials
            self.currentTrial = -1

            # Load starting maze config
            # Wait for experimenter signal
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.START_TRIAL

        elif self.mode == Mode.START_TRIAL:
            rospy.loginfo(f"START OF TRIAL {i}")
            self.currentTrial = self.currentTrial+1

            if self.currentTrial >= self.nTrials:
                self.mode = Mode.END_EXPERIMENT

            # Load maze config according to animal location
            # Project cues

            # Play sound cue
            self.play_sound_cue()

            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.RAT_IN_START_CHAMBER

        elif self.mode == Mode.RAT_IN_START_CHAMBER:
            if (self.current_time - self.mode_start_time) >= self.start_sound_duration.to_sec():
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.START_TO_CHOICE

        elif self.mode == Mode.START_TO_CHOICE:
            rospy.loginfo("START TO CHOICE")
            # Wait for the rat to move to the choice point
            self.mode_start_time = rospy.Time.now()
            self.mode = Mode.CHOICE

        elif self.mode == Mode.CHOICE:
            rospy.loginfo("CHOICE")
            self.door_pub.publish("close_start_door")

            if (self.current_time - self.mode_start_time) >= self.choice_wait_duration.to_sec():
                self.door_pub.publish("open_chosen_door")
                self.stop_sound_cue()
                self.mode_start_time = rospy.Time.now()
                self.mode = Mode.CHOICE_TO_GOAL

        elif self.mode == Mode.CHOICE_TO_GOAL:
            rospy.loginfo("CHOICE TO GOAL")
            if rat_made_right_choice:
                self.door_pub.publish("close_goal_door")
                self.reward_dispense()
                if (self.current_time - self.start_time) == self.reward_duration.to_sec():
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.END_TRIAL
            else:
                self.door_pub.publish("close_goal_door")
                if (self.current_time - self.start_time) == self.wrong_choice_duration.to_sec():
                    self.mode_start_time = rospy.Time.now()
                    self.mode = Mode.END_TRIAL

        elif self.mode == Mode.END_TRIAL:
            rospy.loginfo("END TRIAL")
            self.mode_start_time = rospy.Time.now()
            self.mode == Mode.START_TRIAL

    def play_sound_cue(self):
        self.sound_pub.publish("play_sound")

    def stop_sound_cue(self):
        self.sound_pub.publish("stop_sound")


if __name__ == '__main__':
    rospy.init_node('maze_experiment_controller')
    MazeExperimentController()
    rospy.spin()




