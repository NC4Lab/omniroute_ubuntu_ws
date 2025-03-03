#!/usr/bin/env python

# Custom Imports
from shared_utils.maze_debug import MazeDB
from shared_utils.wall_utilities import MazeDimensions

# ROS Imports
import rospy
from omniroute_operation.msg import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

# Other Imports
import math

class RatDetector:
    # Initialize the RatDetector class
    def __init__(self):
        MazeDB.printMsg('ATTN', "RatDetector Node Started")

        # Subscriber to track the rat's harness OR headstage position within the maze
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped,
                         self.ros_callback_rat_marker_pose, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('headstage_pose_in_maze', PoseStamped,
                         self.ros_callback_rat_marker_pose, queue_size=1, tcp_nodelay=True)

        # Publishers to broadcast the detected chamber positions of the rat's head and body
        self.rat_pose_pub = rospy.Publisher(
            '/rat_pose_in_maze', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.rat_head_chamber_pub = rospy.Publisher(
            '/rat_head_chamber', Int8, queue_size=1, tcp_nodelay=True)
        self.rat_body_chamber_pub = rospy.Publisher(
            '/rat_body_chamber', Int8, queue_size=1, tcp_nodelay=True)

        # Variables to store rat's headstage / harness position
        self.rat_marker_x = 0.0
        self.rat_marker_y = 0.0

        # Maze dimensions and threshold distance for chamber boundaries
        self.MazeDim = MazeDimensions()
        self.threshold = self.MazeDim.chamber_wd / 2

        # Initialize the current chamber of the rat's body, retrying until valid
        self.current_body_chamber = -1
        while self.current_body_chamber == -1:
            self.current_body_chamber = self.rat_head_chamber()

        # Main loop rate (100 Hz)
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    # Determine which chamber the rat's head is currently in
    def rat_head_chamber(self):
        for i in range(9):
            if self.is_rat_head_in_chamber(i):
                return i
        return -1

    # Check if the rat's head is within a specific chamber's boundary
    def is_rat_head_in_chamber(self, chamber_num):
        dist_from_center = self.dist(
            self.rat_marker_x, self.rat_marker_y,
            self.MazeDim.chamber_centers[chamber_num][0],
            self.MazeDim.chamber_centers[chamber_num][1]
        )
        return dist_from_center <= self.threshold

    # Calculate Euclidean distance between two points
    def dist(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    # Check if the rat's body has moved to a new chamber
    def did_rat_body_move_to(self, new_chamber):
        current_chamber_x = self.MazeDim.chamber_centers[self.current_body_chamber][0]
        current_chamber_y = self.MazeDim.chamber_centers[self.current_body_chamber][1]

        new_chamber_x = self.MazeDim.chamber_centers[new_chamber][0]
        new_chamber_y = self.MazeDim.chamber_centers[new_chamber][1]

        # Check movement based on x or y alignment with the new chamber
        if current_chamber_x == new_chamber_x or current_chamber_y == new_chamber_y:
            dist_from_current_chamber = self.dist(
                self.rat_marker_x, self.rat_marker_y, current_chamber_x, current_chamber_y)
            # Verify distance threshold for movement
            if dist_from_current_chamber >= (self.MazeDim.chamber_wd * 1.2) and self.is_rat_head_in_chamber(new_chamber):
                return True
        else:
            # Diagonal movement check with distance threshold
            if self.dist(self.rat_marker_x, self.rat_marker_y, current_chamber_x, current_chamber_y) >= (self.MazeDim.chamber_wd * math.sqrt(2) + 0.05) and self.is_rat_head_in_chamber(new_chamber):
                return True

        return False

    # Callback to update rat's marker position
    def ros_callback_rat_marker_pose(self, data):
        self.rat_marker_pose = data
        self.rat_marker_x = data.pose.position.x
        self.rat_marker_y = data.pose.position.y

        # Publish the rat's current position in the maze
        self.rat_pose_pub.publish(self.rat_marker_pose)

    # Main loop to update the rat's head and body positions in the GUI
    def loop(self):
        # Publish the current chamber of the rat's head
        for i in range(9):
            if self.is_rat_head_in_chamber(i):
                self.rat_head_chamber_pub.publish(i)
                break

        # Check and publish if the rat's body has moved to a new chamber
        for i in range(9):
            if self.did_rat_body_move_to(i):
                self.current_body_chamber = i
                self.rat_body_chamber_pub.publish(i)
                break


if __name__ == '__main__':
    rospy.init_node('rat_detector')
    RatDetector()
    rospy.spin()