#!/usr/bin/env python

import time
import rospy
from omniroute_operation.msg import *
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np

# Importing Gantry library
from gantry.gcodeclient import Client as GcodeClient
# from three_by_three_interface.find_port port find_port

class GantryFeeder:
    # @brief Initialize the GantryFeeder class
    def __init__(self):
        self.gantry_x = 0.0
        self.gantry_y = 0.0
        self.harness_x = 0.0
        self.harness_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0

        self.gantry_marker_to_gantry_center = np.array([-0.285, -0.178])

        # @brief Initialize the subsrciber for reading from '/csv_file_name' topic
        rospy.Subscriber('/gantry_cmd', GantryCmd, self.gantry_cmd_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/gantry_pose_in_maze', PoseStamped, self.gantry_pose_callback, queue_size=1, tcp_nodelay=True)

        self.track_mode = 'NONE'

        # ................ GCode Client Setup ................
        self.gcode_client = GcodeClient('/dev/ttyUSB0', 115200)
        ## TODO: Automatically determine the port

        # Wait for a few secs
        # time.sleep(1)
        self.home()

        time.sleep(1)

        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()
        
    def loop(self):
        if self.track_mode == 'TARGET':
            gantry_to_target = np.array([self.target_x - self.gantry_x, self.target_y - self.gantry_y])

            k = 15.0

            # X component of the harness movement vector
            y = k*gantry_to_target[0]
            # # # Y component of the harness movement vector
            x = k*gantry_to_target[1]

            move_flag = True

        elif self.track_mode == 'HARNESS':
            # Vector from gantry to harness
            gantry_to_harness = np.array([self.harness_x - self.gantry_x, self.harness_y - self.gantry_y])

            k = 10.0

            # X component of the harness movement vector
            y = k*gantry_to_harness[0]
            # # # Y component of the harness movement vector
            x = k*gantry_to_harness[1]
            
            move_flag = True
        else:
            move_flag = False


        if move_flag:
            rospy.loginfo("Gantry Move: {}, {}".format(x, y))

            # # # Move the gantry to the specified location
            # # # TODO: Check if the gantry is within the maze boundary
            if ~np.isnan(x) and ~np.isnan(y):
                self.move_gantry_rel(x, y)
    
    def home(self):
        # self.gcode_client.raw_command("$X")
        self.gcode_client.raw_command("$25=5000")
        self.gcode_client.raw_command("$H")
        self.gcode_client.raw_command("G10 P0 L20 X0 Y0 Z0")
        
    def move_gantry_rel(self, x, y):
        cmd = "$J=G91 G21 X{:.1f} Y{:.1f} F25000".format(x,y)
    
        print(cmd)
        self.gcode_client.raw_command(cmd)

    def move_gantry_abs(self, x, y):
        self.gcode_client.raw_command("$J=G90 G21 X{:.1f} Y{:.1f} F25000".format(x,y))
    
    def cancel_jog(self):
        self.gcode_client.raw_command("0x85")

    def run_pump(self, duration):
        self.gcode_client.raw_command("M3 S127")
        time.sleep(duration)
        self.gcode_client.raw_command("M5")

    def gantry_cmd_callback(self, msg):
        if msg.cmd == "HOME":
            self.track_mode = 'NONE'
            self.home()

        elif msg.cmd == "MOVE":
            self.track_mode = 'TARGET'
            # Move the gantry to the specified location
            self.target_x = msg.args[0]
            self.target_y = msg.args[1]

        elif msg.cmd == "PUMP":
            self.track_mode = 'NONE'
            self.run_pump(msg.args[0])

        elif msg.cmd == "TRACK_HARNESS":
            self.track_mode = 'HARNESS'

    def gantry_pose_callback(self, msg):
        self.gantry_x = msg.pose.position.x + self.gantry_marker_to_gantry_center[0]
        self.gantry_y = msg.pose.position.y + self.gantry_marker_to_gantry_center[1]

    def harness_pose_callback(self, msg):
        self.harness_x = msg.pose.position.x
        self.harness_y = msg.pose.position.y

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'gantry_operation'
    rospy.init_node('gantry_operation')
    GantryFeeder()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown