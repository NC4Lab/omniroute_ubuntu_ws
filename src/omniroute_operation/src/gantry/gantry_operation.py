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

        # @brief Initialize the publisher for writing to '/Esmacat_write_maze_ard0_ease' topic
        # self.maze_ard0_pub = rospy.Publisher('/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)

        # @brief Initialize the subsrciber for reading from '/csv_file_name' topic
        rospy.Subscriber('/gantry_cmd', GantryCmd, self.gantry_cmd_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/Gantry/pose', PoseStamped, self.gantry_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/Harness/pose', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)

        rospy.Subscriber('/natnet_ros/MazeBoundary/marker0/pose', PointStamped, self.mazeboundary_marker0_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker1/pose', PointStamped, self.mazeboundary_marker1_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker2/pose', PointStamped, self.mazeboundary_marker2_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker3/pose', PointStamped, self.mazeboundary_marker3_callback, queue_size=1, tcp_nodelay=True)

        self.gantry_pose = PoseStamped()
        self.harness_pose = PoseStamped()

        self.mazeboundary_marker0 = np.zeros(3)
        self.mazeboundary_marker1 = np.zeros(3)
        self.mazeboundary_marker2 = np.zeros(3)
        self.mazeboundary_marker3 = np.zeros(3)

        self.track_harness = True

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
        # Define gantry xhat and yhat according to the maze boundary markers
        self.xhat = self.mazeboundary_marker0 - self.mazeboundary_marker3
        self.yhat = self.mazeboundary_marker1 - self.mazeboundary_marker0

        # print("Length of xhat: ", np.linalg.norm(self.xhat))
        # print("Length of yhat: ", np.linalg.norm(self.yhat))

        self.xhat = self.xhat / np.linalg.norm(self.xhat)
        self.yhat = self.yhat / np.linalg.norm(self.yhat)
        
        if self.track_harness:
            # Vector from gantry to harness
            self.gantry_to_harness = np.array([self.harness_pose.pose.position.x - self.gantry_pose.pose.position.x, 
                                               self.harness_pose.pose.position.y - self.gantry_pose.pose.position.y, 
                                               self.harness_pose.pose.position.z - self.gantry_pose.pose.position.z])
            
            gh_dist = np.linalg.norm(self.gantry_to_harness)

            k = 10.0

            # X component of the harness movement vector
            x = k*np.dot(self.gantry_to_harness, self.xhat)
            # Y component of the harness movement vector
            y = k*np.dot(self.gantry_to_harness, self.yhat)

            # print("X: ", x, "Y: ", y)
            # Move the gantry to the specified location
            # TODO: Check if the gantry is within the maze boundary
            if ~np.isnan(x) and ~np.isnan(y):
                self.move_gantry_rel(x, y)
    
    def mazeboundary_marker0_callback(self, msg):
        self.mazeboundary_marker0 = np.array([msg.point.x, msg.point.y, msg.point.z])

    def mazeboundary_marker1_callback(self, msg):
        self.mazeboundary_marker1 = np.array([msg.point.x, msg.point.y, msg.point.z])
    
    def mazeboundary_marker2_callback(self, msg):
        self.mazeboundary_marker2 = np.array([msg.point.x, msg.point.y, msg.point.z])
    
    def mazeboundary_marker3_callback(self, msg):
        self.mazeboundary_marker3 = np.array([msg.point.x, msg.point.y, msg.point.z])
    
    def home(self):
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
            self.track_harness = False
            self.home()
        elif msg.cmd == "MOVE":
            self.track_harness = False
            # Move the gantry to the specified location
            self.move_gantry_abs(msg.args[0], msg.args[1])
        elif msg.cmd == "PUMP":
            self.track_harness = False
            self.run_pump(msg.args[0])
        elif msg.cmd == "TRACK_HARNESS":
            self.track_harness = True


    def gantry_pose_callback(self, msg):
        self.gantry_pose = msg
        # print("Gantry Pose: ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def harness_pose_callback(self, msg):
        self.harness_pose = msg
        # print("Harness Pose: ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'gantry_operation'
    rospy.init_node('gantry_operation')
    GantryFeeder()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown