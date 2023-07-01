#!/usr/bin/env python
import os,sys,time
import rospy
import roslib
from sensor_msgs.msg import Joy
# from PyQt5.QtCore import *
# from PyQt5.QtGui import *
# from PyQt5.QtWidgets import *

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
            
	
        self.SCREEN_WIDTH = 720
        self.SCREEN_HEIGHT = 720
        self.line_width = 6
        self.cell_wd = 100
        self.n_cell = 49
        xy_max = self.cell_wd * math.sqrt(self.n_cell)
        self.octagon_side_length = self.cell_wd * math.sqrt(2) / (2 + math.sqrt(2))
        self.x_pos_ind = np.linspace(int(self.cell_wd / 2), int(xy_max - self.cell_wd / 2),  int(math.sqrt(self.n_cell)))
        self.px = 50.0
        self.py = 50.0
        self.vx = 0.0
        self.vy = 0.0
        norm = np.linalg.norm
       
        # Custom signal connections
        # self.update_robot_position_signal.connect(self._handle_update_robot_position)
	
        os.chdir('catkin_ws/src/maze_interface/src/maze_interface/')
       
        maze_config = loadmat('maze_config.mat')['involved_cd']

        # separating first (indicates number of polygons) and second column (indicates up and down walls).
        self.cell_number = [c[0] for c in maze_config]
        self.wall_binary_code = [bin(c[1])[2:].zfill(8)[::-1] for c in maze_config]  
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'maze_interface.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names

        rospy.logerr('Test Interface started')

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
        #self._widget.mazeView.update()

        self._widget.pathBrowseBtn.clicked.connect(self._handle_pathBrowseBtn_clicked)
        # self._widget.pathPreviousBtn.clicked.connect(self._handle_pathPreviousBtn_clicked)
        # self._widget.pathNextBtn.clicked.connect(self._handle_pathNextBtn_clicked)

        self._widget.pathDirEdit.setText(os.path.expanduser(os.path.join('~','catkin_ws','src','maze_interface', 'src', 'maze_interface')))
                
        self.vertices = []
        for i in range(0, 7):
            for j in range(0, 7):
                x = self.x_pos_ind[j]
                y = self.x_pos_ind[i]
                vertices_list = self.octagon(x, y, self.cell_wd / 2)
                self.vertices.append(vertices_list)              
                self.polygon_vertices = [QPointF(i[0], i[1]) for i in vertices_list]
                self.polygon = QPolygonF(self.polygon_vertices)
                self.polygon_item = QGraphicsPolygonItem(self.polygon)
                self.polygon_item.setBrush(QBrush(QColor(180, 180, 180)))
                self.scene.addItem(self.polygon_item)                
                
        self.wall_coord = self.up_walls(self.vertices)
        for w in self.wall_coord:
            self.line = QGraphicsLineItem(w[0][0], w[0][1], w[1][0], w[1][1]) 
            self.pen = QPen(Qt.red)
            self.pen.setWidth(self.line_width)
            self.line.setPen(self.pen)
            self.scene.addItem(self.line)   

        self.image = QPixmap("robot.jpg")
        self.imageItem = QGraphicsPixmapItem(self.image)
        self.image_dimension = round(self.octagon_side_length - 30)
        self.imageItem.setPixmap(self.image.scaled(self.image_dimension,self.image_dimension))
        self.imageItem.setPos(self.px, self.py)
        self.scene.addItem(self.imageItem)   
        
        self.joystick_velocity_scale = 1
        rospy.Subscriber("joy", Joy, self.ros_joystick_callback)
        
        #self._widget.mazeView.update()
        
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

        self.imageCenter = self.imageItem.mapToScene(self.imageItem.boundingRect().center())
        self.imageCenterx = self.imageCenter.x()
        self.imageCentery = self.imageCenter.y()   
      
        for i in self.wall_coord:
            force, dist = self.force_from_line(i[0], i[1], (self.imageCenterx, self.imageCentery),
                                    thresh_dist, spring_k)
            force_ave_x += force[0]
            force_ave_y += force[1]
        
        self.vx = (self.vel_x + force_ave_x)
        self.vy = (self.vel_y + force_ave_y)

        self.px += self.vx
        self.py += self.vy

        if self.px < 0:
            self.px = 0
        if self.px > self.SCREEN_WIDTH:
            self.px = self.SCREEN_WIDTH
        if self.py < 0:
            self.py = 0
        if self.py >= self.SCREEN_HEIGHT:
            self.py = self.SCREEN_HEIGHT   

        self.imageItem.setPos(self.px, self.py)


    def _handle_pathBrowseBtn_clicked(self):
        pathDir = os.path.expanduser(os.path.join('~','catkin_ws','src','maze_interface', 'src', 'maze_interface'))
        filter = "Text Files (*.py)"  # Change this to the file type you want to allow
        files, _ = QFileDialog.getOpenFileNames(None, "Select files to add", pathDir, filter)

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

        
    def octagon(self, x, y, w):
        n = np.arange(-7, 9, 2) / 8
        x_cor = [w * math.sin(math.pi * i) + x for i in n]
        y_cor = [w * math.cos(math.pi * i) + y for i in n]
        x_shift = x_cor[1:] + x_cor[:1]
        y_shift = y_cor[-3:] + y_cor[:-3]
        vertices_list = [(round(x_shift[k], 2), round(y_shift[k], 2)) for k in range(0, 8)]
        return vertices_list
        
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
        r1_magnitude = np.linalg.norm(r1)
        r2_magnitude = np.linalg.norm(r2)
        direction_line = p2 - p1
        n = 1 / np.linalg.norm(direction_line) * (
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

               
        
           


    
    

        
