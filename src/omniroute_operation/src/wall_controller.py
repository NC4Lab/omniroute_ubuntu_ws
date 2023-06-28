#!/usr/bin/env python

import rospy
from omniroute_esmacat_ros.msg import *
import struct
import ctypes
import time
from enum import Enum
from std_msgs.msg import String

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

# Enum for tracking run state
class RunState(Enum):
    GET_CSV = 0
    START_MOVE = 1
    WALL_MOVING_UP = 2
    WALL_UP = 3
    WALL_MOVING_DOWN = 4
    WALL_DOWN = 5
    CHECK_REPLY = 6

# Enum for message type ID
class MsgTypeID(Enum):
    INITIALIZE = 0
    MOVE_WALLS_UP = 1
    MOVE_WALLS_DOWN = 2

class WallController:

    # @brief Initialize the WallController class
    def __init__(self):

        # @brief Initialize the publisher for writing to '/Esmacat_write_maze_ard0_ease' topic
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)
        
        # @brief Initialize the subsrciber for reading from '/csv_file_name' topic
        #rospy.Subscriber('/Esmacat_read_maze_ard0_ease', ease_registers, self.get_response, queue_size=1, tcp_nodelay=True)

        # @brief Initialize the subsrciber for reading from '?' topic
        rospy.Subscriber('/csv_file_name', String, self.callback_load_csv)

        self.start_time = rospy.Time.now()
        self.run_state = RunState.GET_CSV
        self.wall_up_duration = rospy.Duration(10)
        self.wall_down_duration = rospy.Duration(10)

        # Track outgoing and incoming message id
        self.msg_num_id_out = 0

        # Make chamber and byte array lists
        self.cw_list = [
            [[0], [1,3]],
            [[1], [2,3,5]],
            [[2], [5]]
        ]

        # self.cw_list = [
        #     [[6], [0]],
        #     [[7], [1]],
        #     [[8], [2]],
        #     [[9], [3]],
        #     [[10], [4]],
        #     [[11], [5]]
        # ]

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

        # State: GET_CSV
        if self.run_state == RunState.GET_CSV:
            # Return till csv file loaded
            return

        # State: START_MOVE
        if self.run_state == RunState.START_MOVE:
            # Send empty init message
            reg_arr = self.make_reg_msg(MsgTypeID.INITIALIZE.value, self.cw_list)
            reg_arr = [0] * 8
            self.maze_ard0_pub.publish(*reg_arr)  # Publish list to topic

            # Move walls up
            # Create registry message
            reg_arr = self.make_reg_msg(MsgTypeID.MOVE_WALLS_UP.value, self.cw_list)
            self.maze_ard0_pub.publish(*reg_arr)  # Publish list to topic
            # for sublist in self.cw_list:
            #     rospy.loginfo(sublist)

            self.start_time = rospy.Time.now()
            self.run_state = RunState.WALL_MOVING_UP
            rospy.loginfo("MOVE WALL UP")

        # State: WALL_MOVING_UP
        elif self.run_state == RunState.WALL_MOVING_UP:
            if currentTime >= (self.start_time + self.wall_up_duration):
                self.run_state = RunState.WALL_UP

        # State: WALL_UP
        elif self.run_state == RunState.WALL_UP:

            # Move walls back down
            for w_l in self.cw_list:
                w_l[1] = [0]
            # Create registry message
            reg_arr = self.make_reg_msg(MsgTypeID.MOVE_WALLS_DOWN.value, self.cw_list)
            # self.maze_ard0_pub.publish(*reg_arr) # Publish list to topic
            self.start_time = rospy.Time.now()
            self.run_state = RunState.WALL_MOVING_DOWN
            rospy.loginfo("MOVING WALL DOWN")

        # State: WALL_MOVING_DOWN
        elif self.run_state == RunState.WALL_MOVING_DOWN:
            if currentTime >= (self.start_time + self.wall_down_duration):
                self.run_state = RunState.WALL_DOWN

        # State: CHECK_REPLY
        elif self.run_state == RunState.WALL_DOWN:
            self.run_state = RunState.CHECK_REPLY
            rospy.loginfo("CHECK_REPLY")

        elif self.run_state == RunState.CHECK_REPLY:
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

    def make_reg_msg(self, msg_type_id, cw_list):
        # Create a list 'reg' with 8 16-bit Union elements
        U_arr = [Union() for _ in range(8)]
        u_ind = 0

        # Set message num and type id
        U_arr[0].b[0] = self.msg_num_id_out
        U_arr[0].b[1] = msg_type_id
        self.msg_num_id_out += 1
        

        # Update U_arr with corresponding chamber and wall byte
        for cw in cw_list:
            u_ind += 1
            chamber = cw[0][0]
            wall_byte = self.set_wall_byte(cw[1])
            U_arr[u_ind].b[0] = chamber
            U_arr[u_ind].b[1] = wall_byte

        # Set footer
        if u_ind < 7:
            U_arr[u_ind + 1].b[0] = 255
            U_arr[u_ind + 1].b[1] = 255

        # Store and return 16-bit values
        # cast as signed for use with ease_registers
        reg_arr = [ctypes.c_int16(U.i16).value for U in U_arr]

        # Print reg message
        for index, U in enumerate(U_arr):
            rospy.loginfo("%d %d", U.b[0], U.b[1])

        return reg_arr

        # KEEP THIS AT THE END
        pass

    def callback_load_csv(self, data):
        # Log the received message
        rospy.logerr(rospy.get_caller_id() + 'CSV_DIR: %s', data.data)
        #self.run_state = RunState.START_MOVE
        #rospy.loginfo("START_MOVE")

    def get_response(self, data):
        # Log the received message
        #rospy.loginfo(rospy.get_caller_id() + '!!!!!!!!  I heard %s', data.INT0)
        print("")


# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'wall_controller'
    rospy.init_node('wall_controller')
    WallController()  # Create an instance of the WallController class
    rospy.spin()  # Keep the program running until it is explicitly shutdown
