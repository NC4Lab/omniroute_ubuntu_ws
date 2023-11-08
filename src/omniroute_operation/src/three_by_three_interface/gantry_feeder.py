#!/usr/bin/env python

import rospy
from omniroute_operation.msg import *

# Importing Gantry library
from three_by_three_interface.gcodeclient import Client as GcodeClient

class GantryFeeder:
    # @brief Initialize the GantryFeeder class
    def __init__(self):

        # @brief Initialize the publisher for writing to '/Esmacat_write_maze_ard0_ease' topic
        # self.maze_ard0_pub = rospy.Publisher('/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)

        # @brief Initialize the subsrciber for reading from '/csv_file_name' topic
        rospy.Subscriber('/feed', FeedState, self.feed_msg_callback, queue_size=1, tcp_nodelay=True)

        # ................ GCode Client Setup ................
        self.gcode_client = GcodeClient('/dev/ttyUSB0', 115200)
        ## TODO: Automatically determine the port

    def move_gantry(self, x, y):
        self.gcode_client.raw_command("G0 X{} Y{}".format(x,y))

    def feed_msg_callback(self, msg):
        # Move the gantry to the specified location
        self.move_gantry(msg.x, msg.y)

        ## TODO: Based on the state of msg.feed, operate the pump


# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'wall_controller'
    rospy.init_node('gantry_feeder')
    GantryFeeder()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown