#!/usr/bin/env python

import rospy
from omniroute_operation.msg import *

# Custom Imports
from shared_utils.maze_debug import MazeDB

# ROS Imports
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from omniroute_controller.interface import MazeDimensions

import math


class RatDetector:
    # Initialize the RatDetector class
    def __init__(self):
        
        #Initialize the subsrciber for reading from harness and maze boundary markers posistions
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        self.harness_pose = PoseStamped()
        self.rat_head_chamber_pub = rospy.Publisher('/rat_head_chamber', Int8, queue_size=1, tcp_nodelay=True)
        self.rat_body_chamber_pub = rospy.Publisher('/rat_body_chamber', Int8, queue_size=1, tcp_nodelay=True)

        self.harness_x = 0.0
        self.harness_y = 0.0

        self.MazeDim = MazeDimensions()
        self.threshold = self.MazeDim.chamber_wd/2

        self.current_body_chamber = -1
        while self.current_body_chamber == -1:
            self.current_body_chamber = self.rat_head_chamber()
        # rospy.loginfo(f"Current chamber: {self.current_chamber}")

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    
    def rat_head_chamber(self):
        for i in range(9):
            if self.is_rat_head_in_chamber(i):
                return i
        return -1

    def is_rat_head_in_chamber(self, chamber_num):
        dist_from_center = self.dist(self.harness_x, self.harness_y, self.MazeDim.chamber_centers[chamber_num][0], self.MazeDim.chamber_centers[chamber_num][1])
        return dist_from_center <= self.threshold
    
    def dist(self, x1, y1, x2, y2):
        return ((x1-x2)**2 + (y1-y2)**2)**0.5

    def did_rat_body_move_to(self, new_chamber):

        current_chamber_x = self.MazeDim.chamber_centers[self.current_body_chamber][0]
        current_chamber_y = self.MazeDim.chamber_centers[self.current_body_chamber][1]

        new_chamber_x = self.MazeDim.chamber_centers[new_chamber][0]
        new_chamber_y = self.MazeDim.chamber_centers[new_chamber][1]
        
        # Same x or y position
        if current_chamber_x == new_chamber_x or current_chamber_y == new_chamber_y:
            dist_from_current_chamber = self.dist(self.harness_x, self.harness_y, current_chamber_x, current_chamber_y)
            if dist_from_current_chamber >= (self.MazeDim.chamber_wd * 1.25) and self.is_rat_head_in_chamber(new_chamber):

                return True
        else:
            if self.dist(self.harness_x, self.harness_y, current_chamber_x, current_chamber_y) >= (self.MazeDim.chamber_wd * (math.sqrt(2) + 0.25)) and self.is_rat_head_in_chamber(new_chamber):
                return True
        
        return False
            
    def harness_pose_callback(self, data):
        self.harness_pose = data
        self.harness_x = data.pose.position.x
        self.harness_y = data.pose.position.y
        
    def loop(self):

        for i in range(9):
            if self.is_rat_head_in_chamber(i):
                self.rat_head_chamber_pub.publish(i)
                break

        for i in range(9):
            if self.did_rat_body_move_to(i):
                # rospy.loginfo(f"Rat moved to chamber {i}")
                self.current_body_chamber = i
                self.rat_body_chamber_pub.publish(i)
                break


if __name__ == '__main__':
    rospy.init_node('rat_detector')
    rd = RatDetector()
    rospy.spin()