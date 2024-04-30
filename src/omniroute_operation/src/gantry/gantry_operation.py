#!/usr/bin/env python

import time
import rospy
from omniroute_operation.msg import *
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
from enum import Enum

# Importing Gantry library
from gantry.gcodeclient import Client as GcodeClient
# from three_by_three_interface.find_port port find_port

class GantryState(Enum):
    IDLE = 0
    TRACK_HARNESS = 1
    TARGET = 2

class GantryFeeder:
    # Initialize the GantryFeeder class
    def __init__(self):
        rospy.loginfo("[GantryFeeder]: INITAILIZING...")

        # Initialize gantry coordinate class variables
        self.gantry_x = 0.0
        self.gantry_y = 0.0
        self.harness_x = 0.0
        self.harness_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0

        # Specify the x and y offset from the gantry tracking marker to the gantry center (m)
        self.gantry_marker_to_gantry_center = np.array([-0.285, -0.178])

        # Paramters for positioning gantry
        self.chamber_wd = 0.3 # Chamber width (m)
        self.n_chamber_side = 3 
        self.chamber_centers = [] # List of chamber centers 
        self.threshold = 0.06    # Threshold distance for chamber entry from center(m)

        # Compute the chamber centers
        for i in range(0, self.n_chamber_side**2):
            row = i//self.n_chamber_side
            col = i%self.n_chamber_side
            chamber_center = np.array([self.chamber_wd/2 + col*self.chamber_wd, self.chamber_wd/2 + (self.n_chamber_side-1-row)*self.chamber_wd])
            self.chamber_centers.append(chamber_center)

        # Initialize the subsrciber for reading in the gantry commands
        rospy.Subscriber('/gantry_cmd', GantryCmd, self.gantry_cmd_callback, queue_size=1, tcp_nodelay=True)
        # Initialize the subsrciber for reading in the tracker position data
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/gantry_pose_in_maze', PoseStamped, self.gantry_pose_callback, queue_size=1, tcp_nodelay=True)

        # Initialize the gantry mode used for state machine
        self.gantry_mode = GantryState.TRACK_HARNESS

        # ................ GCode Client Setup ................
        self.gcode_client = GcodeClient('/dev/ttyUSB0', 115200)
        ## TODO: Automatically determine the port

        # Wait for a few secs
        time.sleep(1)
        self.home() 
        time.sleep(1)

        ## TODO: What is this for???
        self.gcode_client.raw_command("M3 S1000")

        # Initialize the ROS rate 
        r = rospy.Rate(30)

        # Loop until the node is shutdown
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):
        self.current_time = rospy.Time.now()

        if self.gantry_mode == GantryState.TARGET:
            # Unit vector from gantry to target
            x_offset = 0.0
            y_offset = 0.0
            gantry_to_target = np.array([self.target_x + x_offset - self.gantry_x, self.target_y + y_offset - self.gantry_y])
            
            # Slow on approach
            # distance = np.linalg.norm(gantry_to_target)
            # gantry_to_target = gantry_to_target / distance

             # Speed of gantry movement
            k = 15.0

            # X component of the target movement vector
            y = k* gantry_to_target[0]
            # Y component of the target movement vector
            x = k* gantry_to_target[1]
                    
            # Move the gantry to the target
            if ~np.isnan(x) and ~np.isnan(y):
                self.move_gantry_rel(x, y)         

        elif self.gantry_mode == GantryState.TRACK_HARNESS:
            # Unit vector from gantry to harness
            gantry_to_harness = np.array([self.harness_x - self.gantry_x, self.harness_y - self.gantry_y])
            distance = np.linalg.norm(gantry_to_harness)

            if distance > 0.04:
                
                # Speed of gantry movement
                k = 25.0

                # X component of the harness movement vector
                y = k*gantry_to_harness[0]
                # Y component of the harness movement vector
                x = k*gantry_to_harness[1] 

                if ~np.isnan(x) and ~np.isnan(y):
                    self.move_gantry_rel(x, y)      
            
    def home(self):
        rospy.loginfo("[GantryFeeder]: Homing...")
        self.gcode_client.raw_command("$25=5000")
        self.gcode_client.raw_command("$H")
        self.gcode_client.raw_command("G10 P0 L20 X0 Y0 Z0")
           
    def move_gantry_rel(self, x, y):
        cmd = "$J=G91 G21 X{:.1f} Y{:.1f} F25000".format(x,y)
        self.gcode_client.raw_command(cmd)
        #rospy.loginfo("[GantryFeeder]: TEMP Moving Gantry: ({:.2f}, {:.2f})".format(x, y)) 

    def move_gantry_abs(self, x, y):
        self.gcode_client.raw_command("$J=G90 G21 X{:.1f} Y{:.1f} F25000".format(x,y))
            
    def gantry_cmd_callback(self, msg):
        if msg.cmd == "HOME":
            rospy.loginfo("[GantryFeeder]: Homing Command Received")
            self.home()

        elif msg.cmd == "MOVE":
            self.target_x = msg.args[0]
            self.target_y = msg.args[1]
            self.gantry_mode = GantryState.TARGET
            rospy.loginfo("[GantryFeeder]: Move Command Received: ({:.2f}, {:.2f})".format(self.target_x, self.target_y)  )

        elif msg.cmd == "MOVE_TO_CHAMBER":
            chamber_num = int(msg.args[0])
            self.target_x = self.chamber_centers[chamber_num][0]
            self.target_y = self.chamber_centers[chamber_num][1]
            self.gantry_mode = GantryState.TARGET
            rospy.loginfo("[GantryFeeder]: Move to Chamber Command Received: Chamber({}) Target({:.2f}, {:.2f})".format(chamber_num, self.target_x, self.target_y))

        elif msg.cmd == "STOP_TRACKING_HARNESS":
            rospy.loginfo("[GantryFeeder]: Stop Tracking Harness Command Received")
            self.gantry_mode = GantryState.IDLE

    def gantry_pose_callback(self, msg):
        self.gantry_x = msg.pose.position.x + self.gantry_marker_to_gantry_center[0]
        self.gantry_y = msg.pose.position.y + self.gantry_marker_to_gantry_center[1]
        #rospy.loginfo("Gantry pose: "f'x: {self.gantry_x}, y: {self.gantry_y}')

    def harness_pose_callback(self, msg):
        self.harness_x = msg.pose.position.x
        self.harness_y = msg.pose.position.y
        #rospy.loginfo("Harness pose: "f'x: {self.harness_x}, y: {self.harness_y}')
        

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'gantry_operation'
    rospy.init_node('gantry_operation')
    GantryFeeder()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown