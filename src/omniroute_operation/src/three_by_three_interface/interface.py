#!/usr/bin/env python
import os
import rospy

from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding import loadUi
from python_qt_binding import QtOpenGL

from PyQt5 import QtWidgets, uic
from qt_gui.plugin import Plugin
import numpy as np
from scipy.io import loadmat
import math

from std_msgs.msg import *
from omniroute_operation.msg import *

# GLOBAL VARS
norm = np.linalg.norm
wall_clicked_pub = rospy.Publisher('/wall_state', WallState, queue_size=1)

NUM_ROWS_COLS = 3
CHAMBER_WIDTH = 120
WALL_WIDTH = 10
WALL_MAP = {
    0: [0, 1, 2, 3, 4, 5, 6, 7],
    1: [1, 3, 5, 7, 2],
    2: [0, 1, 2, 3, 4, 5, 6, 7],
    3: [1, 3, 5, 7, 0],
    4: [0, 1, 2, 3, 4, 5, 6, 7],
    5: [1, 3, 5, 7, 4],
    6: [0, 1, 2, 3, 4, 5, 6, 7],
    7: [1, 3, 5, 7, 6],
    8: [0, 1, 2, 3, 4, 5, 6, 7]
}


def in_current_folder(file_name: str):
    # Get the absolute path of the current script file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Create the path to the "config" directory four levels up
    return os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..', 'config', 'paths'))
    # return os.path.join(os.path.dirname(os.path.realpath(__file__)), file_name)


class Wall(QGraphicsItemGroup):
    def __init__(self, p0=(0, 0), p1=(1, 1), wall_width=WALL_WIDTH,
                 chamber_num=-1, wall_num=-1, state=False, label_pos=None, parent=None):
        super().__init__(parent)

        self.chamber_num = chamber_num
        self.wall_num = wall_num
        self.state = state

        self.line = QGraphicsLineItem(QLineF(p0[0], p0[1], p1[0], p1[1]))
        self.upPen = QPen(Qt.red)
        self.upPen.setWidth(wall_width)
        self.downPen = QPen(Qt.gray)
        self.downPen.setWidth(wall_width)

        self.setState(state)

        self.addToGroup(self.line)

        # Plot wall numbers
        self.label = QGraphicsTextItem(str(wall_num))
        self.label.setFont(QFont("Arial", 10, QFont.Bold))

        if label_pos == None:
            label_pos = ((p0[0]+p1[0])/2, (p0[1]+p1[1])/2)
        self.label.setPos(label_pos[0], label_pos[1])
        self.addToGroup(self.label)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            rospy.loginfo("Chamber %d wall %d clicked!" % (self.chamber_num, self.wall_num))
            self.setState(not self.state)
            wall_clicked_pub.publish(self.chamber_num, self.wall_num, self.state)

    def setState(self, state: bool):
        if state:
            self.line.setPen(self.upPen)
        else:
            self.line.setPen(self.downPen)

        self.state = state


class Chamber(QGraphicsItemGroup):
    def __init__(self, center_x=0, center_y=0, chamber_width=CHAMBER_WIDTH,
                 chamber_num=-1, wall_width=WALL_WIDTH, parent=None):
        super().__init__(parent)
        self.center_x = center_x
        self.center_y = center_y
        self.chamber_width = chamber_width
        self.chamber_num = chamber_num

        # Plot backround chamber octogons
        octagon_vertices = self.get_octagon_vertices(center_x, center_y, chamber_width/2, -math.pi/WALL_WIDTH)
        octagon_points = [QPointF(i[0], i[1]) for i in octagon_vertices]
        self.octagon = QGraphicsPolygonItem(QPolygonF(octagon_points))
        self.octagon.setBrush(QBrush(QColor(180, 180, 180)))
        self.addToGroup(self.octagon)

        # Plot cahamber numbers
        self.label = QGraphicsTextItem(str(chamber_num))
        self.label.setFont(QFont("Arial", 20, QFont.Bold))
        self.label.setPos(center_x-10, center_y-20)
        self.addToGroup(self.label)

        wall_angular_offset = 2*math.pi/32  # This decides the angular width of the wall
        wall_vertices_0 = self.get_octagon_vertices(
            center_x, center_y, chamber_width/2, -math.pi/8+wall_angular_offset)
        wall_vertices_1 = self.get_octagon_vertices(
            center_x, center_y, chamber_width/2, -math.pi/8-wall_angular_offset)
        wall_label_pos = self.get_octagon_vertices(
            center_x-8, center_y-10, chamber_width/3, 0)

        self.walls = [Wall(p0=wall_vertices_0[k], p1=wall_vertices_1[k+1], chamber_num=chamber_num,
                           wall_num=k, wall_width=wall_width, label_pos=wall_label_pos[k])
                      for k in range(8)]

    def get_octagon_vertices(self, x, y, w, offset):
        vertices_list = [(round(x + w*math.cos(k)), round(y+w*math.sin(k)))
                         for k in np.linspace(math.pi, 3*math.pi, 9) + offset]
        return vertices_list

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            rospy.loginfo("Chamber %d clicked!" % self.chamber_num)


class Maze:
    def __init__(self, num_rows=2, num_cols=2, chamber_width=CHAMBER_WIDTH,
                 x_offset=0, y_offset=0):

        self.num_rows = num_rows
        self.num_cols = num_cols
        self.chamber_width = chamber_width

        maze_width = self.chamber_width * self.num_cols
        maze_height = self.chamber_width * self.num_rows
        half_width = chamber_width/2
        x_pos = x_offset + \
            np.linspace(half_width, int(maze_width - half_width),  num_cols)
        y_pos = y_offset + \
            np.linspace(half_width, int(maze_height - half_width),  num_rows)

        self.chambers = []
        k = 0
        for y in y_pos:
            for x in x_pos:
                self.chambers.append(
                    Chamber(center_x=x, center_y=y, chamber_num=k, chamber_width=chamber_width))
                k = k+1


class Interface(Plugin):
    def __init__(self, context):
        super(Interface, self).__init__(context)

        self._joint_sub = None

        # Give QObjects reasonable names
        self.setObjectName('Interface')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Pixel measurements
        self.SCREEN_WIDTH = 720
        self.SCREEN_HEIGHT = 720

        # Load maze config
        # maze_config = loadmat(in_current_folder('maze_config.mat'))['involved_cd']
        # separating first (indicates number of polygons) and second column (indicates up and down walls).
        # self.cell_number = [c[0] for c in maze_config]
        # self.wall_binary_code = [bin(c[1])[2:].zfill(8)[::-1] for c in maze_config]

        # Create QWidget
        self._widget = QWidget()
        # Extend the widget with all attributes and children from UI file
        loadUi(os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'interface.ui'), self._widget)

        rospy.loginfo('Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Retrieve the width and height from the UI file
        width = self._widget.width()
        height = self._widget.height()
        # Set the fixed size of the QWidget to match the designed UI size
        # self._widget.setFixedSize(width, height)
        # print("Figure size: ", width, height)

        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.mazeView.setViewportUpdateMode(
            QGraphicsView.FullViewportUpdate)
        self.scene = QGraphicsScene()
        self._widget.mazeView.setScene(self.scene)

        # Set the fixed size of the main window based on the dimensions from the UI file
        main_window_width = self._widget.geometry().width()
        main_window_height = self._widget.geometry().height()
        self._widget.setFixedSize(main_window_width, main_window_height)

        # Set the size hint of the main window to match the size of the _widget
        self._widget.window().setMinimumSize(main_window_width, main_window_height)
        self._widget.window().setMaximumSize(main_window_width, main_window_height)

        # Get the size of the mazeView QGraphicsView
        maze_view_size = self._widget.mazeView.width()
        self.CHAMBER_WIDTH = self._widget.mazeView.width()*0.75/NUM_ROWS_COLS
        #print("MazeView size: ", maze_view_size.width(), maze_view_size.height())

        # # Set the size of the scene and the view
        # sceneWidth = 500
        # sceneHeight = 500
        # scale = 1
        # self._widget.mazeView.setFixedSize(sceneWidth*scale, sceneHeight*scale)
        # self._widget.mazeView.setSceneRect(0, 0, sceneWidth, sceneHeight)

        # Set the background color of the scene to white
        self._widget.mazeView.setBackgroundBrush(QColor(255, 255, 255))
        self._widget.mazeView.setViewport(QtOpenGL.QGLWidget())

        # CSV browser
        self._widget.pathBrowseBtn.clicked.connect(
            self._handle_pathBrowseBtn_clicked)
        self._widget.pathPreviousBtn.clicked.connect(
            self._handle_pathPreviousBtn_clicked)
        self._widget.pathNextBtn.clicked.connect(
            self._handle_pathNextBtn_clicked)
        self._widget.pathDirEdit.setText(in_current_folder('.'))

        # Create Maze and populate walls according to WALL_MAP
        self.maze = Maze(num_rows=NUM_ROWS_COLS, num_cols=NUM_ROWS_COLS, chamber_width=CHAMBER_WIDTH,
                         x_offset=CHAMBER_WIDTH/4, y_offset=CHAMBER_WIDTH/4)

        # Add chambers and disable walls not connected
        for k, c in enumerate(self.maze.chambers):
            self.scene.addItem(c)
            for j, w in enumerate(c.walls):
                if j not in WALL_MAP[k]:
                    w.setEnabled(False)
                    w.setVisible(False)

        # Add walls - this is a new loop so that they are drawn above the chambers
        for c in self.maze.chambers:
            for w in c.walls:
                self.scene.addItem(w)

        # Start timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateScene)
        self.timer.start(20)

    def updateScene(self):
        self.scene.update()
        self._widget.mazeView.update()

    def _handle_pathBrowseBtn_clicked(self):
        # Change this to the file type you want to allow
        filter = "CSV Files (*.csv)"
        files, _ = QFileDialog.getOpenFileNames(
            None, "Select files to add", in_current_folder('.'), filter)

        if files:
            # Clear the list widget to remove any previous selections
            self._widget.pathListWidget.clear()

            # Extract the file names from the full paths and store them in self.files
            self.files = [os.path.basename(file) for file in files]

            # Add the selected file names to the list widget
            self._widget.pathListWidget.addItems(self.files)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(self.files) > 1:
                self._widget.pathNextBtn.setEnabled(True)
                self._widget.pathPreviousBtn.setEnabled(True)
            else:
                self._widget.pathNextBtn.setEnabled(False)
                self._widget.pathPreviousBtn.setEnabled(False)

            # Save the list of selected files as an attribute of the class
            self.current_file_index = 0

            # Connect the "Next" and "Previous" buttons to their respective callback functions
            self._widget.pathPreviousBtn.clicked.connect(
                self._handle_pathPreviousBtn_clicked)
            self._widget.pathNextBtn.clicked.connect(
                self._handle_pathNextBtn_clicked)
            
    def _handle_pathNextBtn_clicked(self):
        # Decrement the current file index
        self.current_file_index -= 1

        # If we've reached the beginning of the list, loop back to the end
        if self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1

        # Set the current file in the list widget
        self._widget.pathListWidget.setCurrentRow(self.current_file_index)
    
    def _handle_pathPreviousBtn_clicked(self):
        # Increment the current file index
        self.current_file_index += 1

        # If we've reached the end of the list, loop back to the beginning
        if self.current_file_index >= len(self.files):
            self.current_file_index = 0

        # Set the current file in the list widget
        self._widget.pathListWidget.setCurrentRow(self.current_file_index)