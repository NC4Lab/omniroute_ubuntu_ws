#!/usr/bin/env python

# Custom Imports
from shared_utils.maze_debug import MazeDB

# ROS Imports
import rospy
from std_msgs.msg import Int32

class ProjectionOperation:
    def __init__(self):
        # Initialize the node (if not already initialized)
        if not rospy.core.is_initialized():
            rospy.init_node('projection_opperation_node', anonymous=True)

        # Create the publisher for 'projection_cmd' topic
        self.projection_pub = rospy.Publisher(
            'projection_cmd', Int32, queue_size=10)

        # Rate for publishing, adjust as needed
        self.rate = rospy.Rate(10)  # 10hz

    def publish_image_cfg_cmd(self, number):
        # Publish the number if it is a single digit between 0 and 9
        if 0 <= number <= 8:
            self.projection_pub.publish(number)
            MazeDB.printMsg(
                'INFO', "Published image config code[%d] to projection_cmd topic", number)
        else:
            MazeDB.printMsg('ERROR', "Image config[%d] is not valid", number)

    def publish_window_mode_cmd(self, number):
        # Can send any number
        self.projection_pub.publish(number)
        MazeDB.printMsg(
            'INFO', "Published window mode code[%d] to projection_cmd topic", number)