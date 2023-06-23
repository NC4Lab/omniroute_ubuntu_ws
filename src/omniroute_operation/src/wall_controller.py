#!/usr/bin/env python

import rospy
from omniroute_esmacat_ros.msg import *
import struct
import ctypes
import time
from enum import Enum

# @brief Union class using ctype union for storing ethercat data shareable accross data types


class Union:
    def __init__(self):
        self.b = bytearray(2)  # 2 bytes

    @property
    def i16(self):
        return struct.unpack("<H", self.b)[0]

    @i16.setter
    def i16(self, value):
        self.b = bytearray(struct.pack("<H", value))

# @brief WallController class
##

class WallState(Enum):
        START = 0
        WALL_MOVING_UP = 1
        WALL_UP = 2
        WALL_MOVING_DOWN = 3
        WALL_DOWN = 4
        END = -1

class WallController:

    # @brief Initialize the WallController class
    def __init__(self):

        # @brief Initialize the publisher for writing to '/Esmacat_write_maze_ard0_ease' topic
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)

     
        self.start_time = rospy.Time.now()
        self.wall_state = WallState.START
        self.wall_up_duration = 10
        self.wall_down_duration = 10

        # @brief Set the desired rate for the loop (100 Hz)
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()  # Call the loop function
            r.sleep()   # Sleep to maintain the desired rate

    # @brief Perform the main loop of the WallController
    def loop(self):
        """
        This function is called repeatedly at a fixed rate.
        It publishes a message to the 'maze_ard0_pub' topic.

        The outgoing register is structured arr[8] of 16 bit integers
        with each 16 bit value seperated into bytes 
        [0]:    b1=254 b0=254           header       
        [1:x]:  b1=chamber b0=wall_ind  wall config
        [x+1]:  b1=255 b0=255           footer

        @return: None
        """

        currentTime = rospy.Time.now()

        if self.wall_state == WallState.START:
            # Move walls up
            # Make chamber and byte array lists
            cw_list = [
                [[0], [1,3]],
                [[1], [2,3,5]],
                [[2], [5]]
            ]
            reg_arr = self.make_reg_msg(cw_list) # Create registry message
            self.maze_ard0_pub.publish(*reg_arr) # Publish list to topic
            # for sublist in cw_list:
            #     rospy.loginfo(sublist)

            self.start_time = rospy.Time.now()
            self.wall_state = WallState.WALL_MOVING_UP

        elif self.wall_state == WallState.WALL_MOVING_UP:
            if currentTime >= (self.start_time + self.wall_up_duration):
                self.wall_state = WallState.WALL_UP
        
        elif self.wall_state == WallState.WALL_UP:
            # Move walls back down
            for w_l in cw_list:
                w_l[1] = [0]
            reg_arr = self.make_reg_msg(cw_list) # Create registry message
            self.maze_ard0_pub.publish(*reg_arr) # Publish list to topic
            self.start_time = rospy.Time.now()
            self.wall_state = WallState.WALL_MOVING_DOWN

        elif self.wall_state == WallState.WALL_MOVING_DOWN:
            if currentTime >= (self.start_time + self.wall_down_duration):
                self.wall_state = WallState.WALL_DOWN
        
        elif self.wall_state == WallState.WALL_DOWN:
            self.wall_state = WallState.END

        elif self.wall_state == WallState.END:
            return

        return

    # @brief Create a byte with bits set to 1 based on wall_up_arr
    def set_wall_byte(self, wall_up_arr):
        byte_value = 0  # Initialize the byte value

        # Iterate over the array of values
        for index in wall_up_arr:
            if 0 <= index <= 7:
                # Set the corresponding bit to 1 using bitwise OR
                byte_value |= (1 << index)

        return byte_value

    def make_reg_msg(self, cw_list):
        # Create a list 'reg' with 8 16-bit Union elements
        U_arr = [Union() for _ in range(8)]
        u_ind = 0

        # Set header
        U_arr[u_ind].b[1] = 254
        U_arr[u_ind].b[0] = 254

        # Update U_arr with corresponding chamber and wall byte
        for cw in cw_list:
            u_ind += 1
            chamber = cw[0][0]
            wall_byte = self.set_wall_byte(cw[1])
            U_arr[u_ind].b[1] = chamber
            U_arr[u_ind].b[0] = wall_byte

        # Set footer
        if u_ind < 7:
            U_arr[u_ind + 1].b[1] = 255
            U_arr[u_ind + 1].b[0] = 255

        # Store and return 16-bit values
        reg_arr = [ctypes.c_int16(U.i16).value for U in U_arr] # cast as signed for use with ease_registers
        return reg_arr

        # KEEP THIS AT THE END
        pass


# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'wall_controller'
    rospy.init_node('wall_controller')
    WallController()  # Create an instance of the WallController class
    rospy.spin()  # Keep the program running until it is explicitly shutdown


