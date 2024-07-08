#!/usr/bin/env python

import rospy
from omniroute_operation.msg import *

# Custom Imports
from shared_utils.maze_debug import MazeDB

# ROS Imports
from geometry_msgs.msg import PoseStamped

from omniroute_controller.interface import MazeDimensions


class RatDetector:
    # Initialize the RatDetector class
    def __init__(self):
        
        #Initialize the subsrciber for reading from harness and maze boundary markers posistions
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        self.harness_pose = PoseStamped()
        self.harness_x = 0.0
        self.harness_y = 0.0

        self.MazeDim = MazeDimensions()

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()
    
    def harness_pose_callback(self, data):
        self.harness_pose = data
        self.harness_x = data.pose.position.x
        self.harness_y = data.pose.position.y
        
    def loop(self):
        
        rospy.loginfo("Harness x: %f, y: %f", self.harness_x, self.harness_y)
        rospy.loginfo("Maze Dimensions: %f", self.MazeDim.chamber_wd)

        pass

if __name__ == '__main__':
    rospy.init_node('rat_detector')
    rd = RatDetector()
    rospy.spin()