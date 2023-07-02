#!/usr/bin/env python
import os,sys,time
import rospy
from sensor_msgs.msg import Joy

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

norm = np.linalg.norm

## GLOBAL VARS
NUM_ROWS = 3
NUM_COLS = 3
CHAMBER_WIDTH = 100
WALL_MAP = {
    0: [0,1,2,3],
    1: [0,1,2],
    2: [3,4,5],
    3: [3,4,5],
    4: [3,4,5],
    5: [3,4,5],
    6: [3,4,5],
    7: [3,4,5],
    8: [3,4,5]
}

def in_current_folder(file_name: str):
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), file_name)

## Returns vertices of an octagon
# def old_octagon(x, y, w):
#     n = np.arange(-7, 9, 2) / 8
#     x_cor = [w * math.sin(math.pi * i) + x for i in n]
#     y_cor = [w * math.cos(math.pi * i) + y for i in n]
#     x_shift = x_cor[1:] + x_cor[:1]
#     y_shift = y_cor[-3:] + y_cor[:-3]
#     vertices_list = [(round(x_shift[k], 2), round(y_shift[k], 2)) for k in range(0, 8)]
#     return vertices_list



class Wall(QGraphicsLineItem):
    def __init__(self, num=-1, p0=(0,0), p1=(1,1), w=6, parent=None):
        super().__init__(parent)

        self.num=num
        self.setLine(QLineF(p0[0], p0[1], p1[0], p1[1]))
        pen = QPen(Qt.red)
        pen.setWidth(w)
        self.setPen(pen)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            print("Wall %d clicked!" % self.num)

class Chamber(QGraphicsItemGroup):
    def __init__(self, num=-1, center_x=0, center_y=0, chamber_width=100, parent=None):
        super().__init__(parent)
        self.x = center_x
        self.y = center_y
        self.w = chamber_width
        self.num = num

        octagon_vertices = self.get_octagon_vertices(center_x, center_y, chamber_width/2)
        octagon_points = [QPointF(i[0], i[1]) for i in octagon_vertices]
        self.octagon = QGraphicsPolygonItem(QPolygonF(octagon_points))
        self.octagon.setBrush(QBrush(QColor(180, 180, 180)))
        self.addToGroup(self.octagon)

        self.label = QGraphicsTextItem(str(num))
        self.label.setFont(QFont("Arial", 20, QFont.Bold))
        self.label.setPos(center_x-10, center_y-20)
        self.addToGroup(self.label)
        
        self.walls = [Wall(k, octagon_vertices[k], octagon_vertices[k+1]) for k in range(8)]

    def get_octagon_vertices(self, x, y, w):
        vertices_list = [(round(x + w*math.cos(k)), round(y+w*math.sin(k))) for k in np.linspace(0, 2*math.pi, 9)+math.pi/8]
        return vertices_list

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            print("Chamber %d clicked!" % self.num)

class Maze:
    def __init__(self, num_rows=2, num_cols=2, chamber_width=100,
                  x_offset=0, y_offset=0):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.chamber_width = chamber_width
        maze_width = self.chamber_width * self.num_cols
        maze_height = self.chamber_width * self.num_rows
        half_width = chamber_width/2
        x_pos = x_offset + np.linspace(half_width, int(maze_width - half_width),  num_cols)
        y_pos = y_offset + np.linspace(half_width, int(maze_height - half_width),  num_rows)
        
        self.chambers = []
        k = 0
        for x in x_pos:
            for y in y_pos:
                self.chambers.append(Chamber(num = k, center_x=x, center_y=y, chamber_width=chamber_width))
                k = k+1

class Interface(Plugin):
    # update_robot_position_signal = Signal()

    def __init__(self, context):
        super(Interface, self).__init__(context)
        
        self._joint_sub = None

        ## Give QObjects reasonable names
        self.setObjectName('Interface')
               
        ## Process standalone plugin command-line arguments
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
        self.chamber_width = 100           ## Chamber width (in pixels)
       
        ## Load maze config
        # maze_config = loadmat(in_current_folder('maze_config.mat'))['involved_cd']
        # separating first (indicates number of polygons) and second column (indicates up and down walls).
        # self.cell_number = [c[0] for c in maze_config]
        # self.wall_binary_code = [bin(c[1])[2:].zfill(8)[::-1] for c in maze_config]  
        
        # Create QWidget
        self._widget = QWidget()
        # Extend the widget with all attributes and children from UI file
        loadUi(in_current_folder('interface.ui'), self._widget)
        # Give QObjects reasonable names

        rospy.logerr('Interface started')

        self._widget.setObjectName('InterfacePluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self._widget.mazeView.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.scene = QGraphicsScene()
        self._widget.mazeView.setScene(self.scene)
         
        # Set the size of the scene and the view
        self._widget.mazeView.setFixedSize(self.SCREEN_WIDTH, self.SCREEN_HEIGHT)
        self._widget.mazeView.setSceneRect(0, 0, self.SCREEN_WIDTH, self.SCREEN_HEIGHT)

        # Set the background color of the scene to gray
        self._widget.mazeView.setBackgroundBrush(QColor(0, 0, 0)) 
        
        self._widget.mazeView.setViewport(QtOpenGL.QGLWidget())

        self._widget.pathBrowseBtn.clicked.connect(self._handle_pathBrowseBtn_clicked)
        # self._widget.pathPreviousBtn.clicked.connect(self._handle_pathPreviousBtn_clicked)
        # self._widget.pathNextBtn.clicked.connect(self._handle_pathNextBtn_clicked)

        self._widget.pathDirEdit.setText(in_current_folder('.'))

        # Make all chambers and walls
        # self.all_vertices_list = []
        # for i in range(0, self.chambers_per_side):
        #     for j in range(0, self.chambers_per_side):
        #         x = self.chamber_x_pos[i]
        #         y = self.chamber_x_pos[j]
        #         self.all_vertices_list.append(old_octagon(x,y, self.chamber_width/2))              
                
        self.maze = Maze(num_rows=NUM_ROWS, num_cols=NUM_COLS)

        for k,c in enumerate(self.maze.chambers):
            self.scene.addItem(c)
            for j,w in enumerate(c.walls):
                if j not in WALL_MAP[k]:
                    w.setEnabled(False)
                    w.setVisible(False)
                self.scene.addItem(w)

        # self.wall_coord = self.up_walls(self.all_vertices_list)
        # for w in self.wall_coord:
        #     self.line = QGraphicsLineItem(w[0][0], w[0][1], w[1][0], w[1][1]) 
        #     self.pen = QPen(Qt.red)
        #     self.pen.setWidth(self.wall_width)
        #     self.line.setPen(self.pen)
        #     self.scene.addItem(self.line)   

        robotMap = QPixmap(in_current_folder("robot.jpg"))
        robot_dimension = round(self.chamber_width/2)
        self.robot_pos_x = 50.0
        self.robot_pos_y = 50.0
        self.robot_vel_x = 0.0
        self.robot_vel_y = 0.0
        self.robot = QGraphicsPixmapItem(robotMap)
        self.robot.setPixmap(robotMap.scaled(robot_dimension,robot_dimension))
        self.robot.setPos(self.robot_pos_x, self.robot_pos_y)
        self.scene.addItem(self.robot)   

        self._widget.mazeView.update()
        
        self.joystick_velocity_scale = 1
        rospy.Subscriber("joy", Joy, self.ros_joystick_callback)
        
        # Start timer
        self.timer=QTimer()
        self.timer.timeout.connect(self.updateScene)
        self.timer.start(20)
    
    # def _handle_update_robot_position(self):
    #     self.imageItem.setPos(self.px, self.py)
    
    def updateScene(self):
        thresh_dist = 10
        spring_k = 2.5
        max_vel = 10
        force_ave_x = 0
        force_ave_y = 0

        self.scene.update()
        self._widget.mazeView.update()

        self.imageCenter = self.robot.mapToScene(self.robot.boundingRect().center())
        self.imageCenterx = self.imageCenter.x()
        self.imageCentery = self.imageCenter.y()   
      
        # for i in self.wall_coord:
        #     force, dist = self.force_from_line(i[0], i[1], (self.imageCenterx, self.imageCentery),
        #                             thresh_dist, spring_k)
        #     force_ave_x += force[0]
        #     force_ave_y += force[1]
        
        self.robot_vel_x = (self.robot_vel_x + force_ave_x)
        self.robot_vel_y = (self.robot_vel_y + force_ave_y)

        self.robot_pos_x += self.robot_vel_x
        self.robot_pos_y += self.robot_vel_y

        if self.robot_pos_x < 0:
            self.robot_pos_x = 0
        if self.robot_pos_x > self.SCREEN_WIDTH:
            self.robot_pos_x = self.SCREEN_WIDTH
        if self.robot_pos_y < 0:
            self.robot_pos_y = 0
        if self.robot_pos_y >= self.SCREEN_HEIGHT:
            self.robot_pos_y = self.SCREEN_HEIGHT   

        self.robot.setPos(self.robot_pos_x, self.robot_pos_y)

    def _handle_pathBrowseBtn_clicked(self):
        filter = "Text Files (*.py)"  # Change this to the file type you want to allow
        files, _ = QFileDialog.getOpenFileNames(None, "Select files to add", in_current_folder('.'), filter)

        if files:
            # Clear the list widget to remove any previous selections
            self._widget.listWidget.clear()

            # Add the selected files to the list widget
            self._widget.listWidget.addItems(files)

            # Enable the "Next" and "Previous" buttons if there is more than one file
            if len(files) > 1:
                self._widget.pathNextBtn.setEnabled(True)
                self._widget.pathPreviousBtn.setEnabled(True)
            else:
                self._widget.pathNextBtn.setEnabled(False)
                self._widget.pathPreviousBtn.setEnabled(False)

            # Save the list of selected files as an attribute of the class
            self.files = files
            self.current_file_index = 0

            # Connect the "Next" and "Previous" buttons to their respective callback functions
            self._widget.pathPreviousBtn.clicked.connect(self._handle_pathPreviousBtn_clicked)
            self._widget.pathNextBtn.clicked.connect(self._handle_pathNextBtn_clicked)


    def _handle_pathNextBtn_clicked(self):
        # Increment the current file index
        self.current_file_index += 1

         # If we've reached the end of the list, loop back to the beginning
        if self.current_file_index >= len(self.files):
            self.current_file_index = 0

        # Set the current file in the list widget
        self._widget.listWidget.setCurrentRow(self.current_file_index)


    def _handle_pathPreviousBtn_clicked(self):
        # Decrement the current file index
        self.current_file_index -= 1

        # If we've reached the beginning of the list, loop back to the end
        if self.current_file_index < 0:
            self.current_file_index = len(self.files) - 1

        # Set the current file in the list widget
        self._widget.listWidget.setCurrentRow(self.current_file_index)
    
    def up_walls(self, vertices):
        wall_coordinates = []
        for i in range(len(self.cell_number)):
            vertices_path = vertices[self.cell_number[i] - 1]
            binary_code = self.wall_binary_code[i]
            for c in range(len(binary_code)):
                if binary_code[c] == '1':
                    wall_coordinates.append([vertices_path[c % 8], vertices_path[(c + 1) % 8]])
        return wall_coordinates
    
       
    def ros_joystick_callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + 'maze_interface = %s', msg.axes)

        self.vel_x = (-1) * msg.axes[0] * self.joystick_velocity_scale
        self.vel_y = (-1) * msg.axes[1] * self.joystick_velocity_scale
          
        # self.update_robot_position_signal.emit()


    def force_from_line(self, line_start, line_end, point, thresh, spring_k):
        p1 = np.array(line_start)
        p2 = np.array(line_end)
        p3 = np.array(point)
        r1 = p3 - p1
        r2 = p3 - p2
        r1_magnitude = norm(r1)
        r2_magnitude = norm(r2)
        direction_line = p2 - p1
        n = 1 / norm(direction_line) * (
            np.array((direction_line[1], -direction_line[0])))
        d = np.dot(r1, n)
        x3, y3 = self.intersecting_point(line_start, line_end, point)
        force = np.zeros(2)
        if abs(d) < thresh:
            if self.point_pos(line_start, line_end, (x3, y3)):
                force = spring_k * d * n
            elif r1_magnitude < r2_magnitude:
                if r1_magnitude < thresh:
                    force = spring_k * r1
            elif r2_magnitude < thresh:
                force = spring_k * r2
        return force, d

    def intersecting_point(self, line_start, line_end, point):
        p1 = np.array(line_start)
        p2 = np.array(line_end)
        p3 = np.array(point)
        direction_line = p2 - p1
        a1 = direction_line[1]
        b1 = direction_line[0]
        c1 = a1 * p1[0] - b1 * p1[1]
        a2 = -direction_line[0]
        b2 = direction_line[1]
        c2 = a2 * p3[0] - b2 * p3[1]
        x0 = (b1 * c2 - b2 * c1) / -(a1 * b2 - a2 * b1)
        y0 = (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1)
        return round(x0, 2), round(y0, 2)

    def point_pos(self, line_start, line_end, point):
        p1 = line_start
        p2 = line_end
        p3 = point
        return bool(min(p1[0], p2[0]) <= p3[0] <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= p3[1] <= max(p1[1], p2[1]))

               
        
           


    
    

        
