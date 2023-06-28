#!/usr/bin/env python

import rospy
from omniroute_esmacat_ros.msg import *
import struct
import ctypes
import time
import csv
from enum import Enum
from std_msgs.msg import String
from colorama import Fore, Style

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
    INITIALIZE = 1
    START_MOVE = 2
    WALL_MOVING_UP = 3
    WALL_UP = 4
    WALL_MOVING_DOWN = 5
    WALL_DOWN = 6
    CHECK_REPLY = 7

# Enum for message type ID


class MsgTypeID(Enum):
    INITIALIZE = 128
    MOVE_WALLS_UP = 1
    MOVE_WALLS_DOWN = 2


class WallController:

    # Initialize colorama
    Fore.RED, Fore.GREEN, Fore.BLUE, Fore.YELLOW  # Set the desired colors

    # @brief Initialize the WallController class
    def __init__(self):

        # @brief Initialize the publisher for writing to '/Esmacat_write_maze_ard0_ease' topic
        self.maze_ard0_pub = rospy.Publisher(
            '/Esmacat_write_maze_ard0_ease', ease_registers, queue_size=1)

        # @brief Initialize the subsrciber for reading from '/csv_file_name' topic
        #rospy.Subscriber('/Esmacat_read_maze_ard0_ease', ease_registers, self.callback_get_response, queue_size=1, tcp_nodelay=True)

        # @brief Initialize the subsrciber for reading from '?' topic
        rospy.Subscriber('/csv_file_name', String, self.callback_load_csv)

        # Option to automatically load csv
        self.do_auto_load_csv = True;
        self.csv_file_path = "/media/windows/Users/lester/MeDocuments/Research/MadhavLab/CodeBase/omniroute_operation_ws/config/path_1x3_1.csv"

        # State machine variables
        self.last_ts = rospy.Time.now()
        self.run_state = RunState.GET_CSV
        self.start_dt = rospy.Duration(1)
        self.init_dt = rospy.Duration(1)
        self.wall_up_dt = rospy.Duration(5)
        self.wall_down_dt = rospy.Duration(5)

        # Track outgoing and incoming message id
        self.msg_num_id_out = 0

        # # Make chamber and byte array lists
        # cw_list = [
        #     [[0], [1,3]],
        #     [[1], [2,3,5]],
        #     [[2], [5]]
        # ]

        # cw_list = [
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
            if currentTime >= (self.last_ts + self.start_dt):
                self.last_ts = rospy.Time.now()
                # Check for autoload
                if self.do_auto_load_csv:
                    self.run_state = RunState.INITIALIZE

        # State: INITIALIZE
        elif self.run_state == RunState.INITIALIZE:
            self.rosby_log_info(Fore.GREEN,"INITIALIZE")

            # Send empty init message
            reg_arr = self.make_reg_msg(MsgTypeID.INITIALIZE)
            self.maze_ard0_pub.publish(*reg_arr)  # Publish list to topic

            self.last_ts = rospy.Time.now()
            self.run_state = RunState.START_MOVE

        # State: START_MOVE
        elif self.run_state == RunState.START_MOVE:
            if currentTime >= (self.last_ts + self.init_dt):
                self.rosby_log_info(Fore.GREEN,"MOVE WALL UP")
                
                # Parse the csv file
                cw_list = self.parse_csv()

                # Create registry message
                reg_arr = self.make_reg_msg(MsgTypeID.MOVE_WALLS_UP, cw_list)
                self.maze_ard0_pub.publish(*reg_arr)  # Publish list to topic
                # for sublist in cw_list:
                #     rospy.loginfo(sublist)

                self.last_ts = rospy.Time.now()
                self.run_state = RunState.WALL_MOVING_UP

        # State: WALL_MOVING_UP
        elif self.run_state == RunState.WALL_MOVING_UP:
            if currentTime >= (self.last_ts + self.wall_up_dt):
                self.run_state = RunState.WALL_UP

        # State: WALL_UP
        elif self.run_state == RunState.WALL_UP:
            self.rosby_log_info(Fore.GREEN,"MOVING WALL DOWN")

            # Create registry message
            reg_arr = self.make_reg_msg(MsgTypeID.MOVE_WALLS_DOWN)
            self.maze_ard0_pub.publish(*reg_arr) # Publish list to topic
            self.last_ts = rospy.Time.now()
            self.run_state = RunState.WALL_MOVING_DOWN
           

        # State: WALL_MOVING_DOWN
        elif self.run_state == RunState.WALL_MOVING_DOWN:
            if currentTime >= (self.last_ts + self.wall_down_dt):
                self.run_state = RunState.WALL_DOWN

        # State: WALL_DOWN
        elif self.run_state == RunState.WALL_DOWN:
            self.rosby_log_info(Fore.GREEN,"CHECK_REPLY")
            self.run_state = RunState.CHECK_REPLY

        # State: CHECK_REPLY
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

    def make_reg_msg(self, msg_type_id, cw_list=None):

        # Create a list 'reg' with 8 16-bit Union elements
        U_arr = [Union() for _ in range(8)]
        u_ind = 0

        # Set message num and type id
        self.msg_num_id_out = self.msg_num_id_out+1 if self.msg_num_id_out < 255 else 1
        U_arr[u_ind].b[0] = self.msg_num_id_out
        U_arr[u_ind].b[1] = msg_type_id.value
        
        # Update walls to move up
        if msg_type_id == MsgTypeID.MOVE_WALLS_UP and cw_list is not None:
            # Update U_arr with corresponding chamber and wall byte
            for cw in cw_list:
                u_ind += 1
                chamber = cw[0]
                wall_byte = self.set_wall_byte(cw[1])
                U_arr[u_ind].b[0] = chamber
                U_arr[u_ind].b[1] = wall_byte

        # Set footer
        if u_ind < 7:
            U_arr[u_ind + 1].b[0] = 254
            U_arr[u_ind + 1].b[1] = 254

        # Store and return 16-bit values
        # cast as signed for use with ease_registers
        reg_arr = [ctypes.c_int16(U.i16).value for U in U_arr]

        # Print reg message
        for index, U in enumerate(U_arr):
            self.rosby_log_info(Fore.BLUE,"%d %d", U.b[0], U.b[1])

        return reg_arr

    def parse_csv(self):
        """
        Parses the CSV file and returns the chamber and wall configuration list.
        Returns:
            list: The chamber and wall configuration list.
        """
        cw_list = []  # Create an empty chamber and wall configuration list

        if not self.csv_file_path:
            rospy.logerr("CSV file path is not set!")
            return cw_list

        try:
            with open(self.csv_file_path, 'r') as csv_file:
                csv_reader = csv.reader(csv_file)
                for row in csv_reader:
                    if len(row) >= 2:
                        chamber = int(row[0])
                        walls_byte = int(row[1])
                        walls = [i for i in range(8) if walls_byte & (1 << i)]
                        # Add chamber and wall configuration to the list
                        cw_list.append([chamber, walls])
        except IOError as e:
            rospy.logerr("Error reading CSV file: %s", str(e))

        # # Print the cw_list
        # self.rosby_log_info(Fore.BLUE,"Chamber and wall configuration list:")
        # for chamber, walls in cw_list:
        #     self.rosby_log_info(Fore.BLUE,"Chamber %d: Walls %s", chamber, walls)

        return cw_list
    
    def rosby_log_info(self, color, message, *args):
        colored_message = f"{color}{message}{Style.RESET_ALL}"
        formatted_message = colored_message % args
        rospy.loginfo(formatted_message)

    def callback_load_csv(self, data):
        # Store the csv file path
        self.csv_file_path = data.data
        # rospy.logerr(rospy.get_caller_id() + 'CSV_DIR: %s', data.data)
        self.run_state = RunState.INITIALIZE
        rospy.loginfo("INITIALIZE")

    def callback_get_response(self, data):
        # Log the received message
        rospy.loginfo(rospy.get_caller_id() +
                      '!!!!!!!!  I heard %s', data.INT0)


# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'wall_controller'
    rospy.init_node('wall_controller')
    WallController()  # Create an instance of the WallController class
    rospy.spin()  # Keep the program running until it is explicitly shutdown
