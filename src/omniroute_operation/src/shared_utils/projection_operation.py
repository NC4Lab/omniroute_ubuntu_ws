#!/usr/bin/env python
# Class for publishing commands to the projection system

# Custom Imports
from shared_utils.maze_debug import MazeDB

# ROS Imports
import rospy
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension, String

# Other
import csv
import os
import json


class ProjectionOperation:
    # ------------------------ CLASS VARIABLES ------------------------

    # Wall image file names
    # NOTE: This list needs to match that used in:
    # omniroute_windows_ws\src\projection_operation\include\projection_utils.h
    WALL_IMAGE_FILE_NAMES = [
        "w_black",
        "w_square",
        "w_circle",
        "w_triangle",
        "w_star",
        "w_pentagon"
    ]

    # Floor image file names
    # NOTE: This list needs to match that used in:
    # omniroute_windows_ws\src\projection_operation\include\projection_utils.h
    FLOOR_IMAGE_FILE_NAMES = [
        "f_black",
        "f_green",
        "f_pattern_0",
        "f_pattern_1",
        "f_pattern_2",
        "f_white"
    ]

    def __init__(self):

        # Initialize the node (if not already initialized)
        if not rospy.core.is_initialized():
            rospy.init_node('projection_opperation_node', anonymous=True)

        # Create the publisher for 'projection_cmd' topic
        self.projection_pub = rospy.Publisher(
            'projection_cmd', Int32, queue_size=10)

        # Create the publisher for the 'projection_image' topic
        self.image_pub = rospy.Publisher(
            'projection_image', Int32MultiArray, queue_size=10)
        
        rospy.Subscriber('projection_image_floor_num', Int32, self.projection_image_floor_callback)

        rospy.Subscriber('projection_walls', String, self.projection_walls_callback)

        rospy.Subscriber('projection_image_wall_num', Int32, self.projection_image_wall_callback)

        # Initialize image_config as a 10x8 array with default values
        self.image_config = [[0 for _ in range(8)] for _ in range(10)]

        # Rate for publishing set to 30hz
        self.rate = rospy.Rate(30)

    def projection_image_floor_callback(self, msg):
        #rospy.loginfo("Received projection floor image number: number[%d]", msg.data)
        self.floor_img_num = msg.data
        self.set_config('floor', self.floor_img_num)
        self.publish_image_message()
        
    def projection_image_wall_callback(self, msg):
        #rospy.loginfo("Received projection wall image number: number[%d]", msg.data)
        self.wall_image_num = msg.data
        
    def projection_walls_callback(self, msg):
        #rospy.loginfo("Received projection wall number: number[%s]", msg.data)
        wall_num = json.loads(msg.data)
        self.cham_ind = wall_num['chamber_num']
        self.wall_ind = wall_num['wall_num']

        self.set_config('walls', self.wall_image_num, cham_ind=self.cham_ind, wall_ind=self.wall_ind)
        self.publish_image_message()

    
    def setup_layout(self, dim1, dim2):
        """Helper function to set up the layout for a 2-dimensional array."""
        layout = []

        # Define first dimension (rows)
        dim1_layout = MultiArrayDimension()
        dim1_layout.label = "rows"
        dim1_layout.size = dim1
        dim1_layout.stride = dim1 * dim2
        layout.append(dim1_layout)

        # Define second dimension (columns)
        dim2_layout = MultiArrayDimension()
        dim2_layout.label = "columns"
        dim2_layout.size = dim2
        dim2_layout.stride = dim2
        layout.append(dim2_layout)

        return layout

    def set_config(self, data_type, img_ind, cham_ind=None, wall_ind=None):
        """
        Read the CSV and structure the data into either a 10x8 array for 'walls'
        or extract a single value for 'floor' and modify the image_config.

        Args:
            data_type (str): A string that specifies whether to process the data as
                            'walls' or 'floor'. 
                            - 'walls': Updates the 10x8 array for wall configuration.
                            - 'floor': Updates a single value in the last entry of dim1 and first entry of dim2.
            img_ind (int): The index of the image to set.
            cham_ind (int): The index of the chamber to set.
            wall_ind (int): The index of the wall to set.

        Returns:
            list: modified 10x8 list.
        """

        if data_type == "walls":
            self.image_config[cham_ind][wall_ind] = img_ind

        elif data_type == "floor":
            # Store the value in the last entry of dim1 and first entry of dim2
            self.image_config[-1][0] = img_ind

        else:
            MazeDB.printMsg(
                'WARN', "[ProjectionOperation:set_config] Expected 'walls' or 'floor': data_type[%s]", data_type)
            

    def set_config_from_csv(self, file_path, data_type):
        """
        Read the CSV and structure the data into either a 10x8 array for 'walls'
        or extract a single value for 'floor' and modify the image_config.

        Args:
            file_path (str): The path to the CSV file containing the data.
            data_type (str): A string that specifies whether to process the data as
                            'walls' or 'floor'. 
                            - 'walls': Updates the 10x8 array for wall configuration.
                            - 'floor': Updates a single value in the last entry of dim1 and first entry of dim2.

        Returns:
            list: modified 10x8 list.
        """

        with open(file_path, mode='r') as csvfile:
            csv_reader = csv.reader(csvfile)

            if data_type == "walls":
                next(csv_reader)  # Skip the header row
                # For walls, store data as a 10x8 array
                for row_idx, row in enumerate(csv_reader):
                    if row_idx < 10:
                        # Ignore the first column and take columns 1-8 (which are index 1 to 8 in 0-based indexing)
                        data_row = list(map(int, row[1:9]))
                        self.image_config[row_idx] = data_row

            elif data_type == "floor":
                first_row = next(csv_reader)  # Get the first data row
                # Read the value from the first column
                floor_value = int(first_row[0])
                # Store the value in the last entry of dim1 and first entry of dim2
                self.image_config[-1][0] = floor_value

            else:
                MazeDB.printMsg(
                    'WARN', "[ProjectionOperation:set_config] Expected 'walls' or 'floor': data_type[%s]", data_type)

    def publish_image_message(self):
        """
        Send the data from Int32MultiArray image_config.
        """

        # Create the Int32MultiArray message
        projection_data = Int32MultiArray()

        # Set up the layout using the helper function (10x8 array)
        projection_data.layout.dim = self.setup_layout(10, 8)

        # Flatten the 10x8 array into a single list
        flat_data = [self.image_config[i][j]
                     for i in range(10) for j in range(8)]
        projection_data.data = flat_data

        # Publish the CSV data message
        self.image_pub.publish(projection_data)

        # Log the sent message data
        MazeDB.printMsg(
            'INFO', "Published new projection config")

    def publish_command_message(self, number):
        # Can send any number
        self.projection_pub.publish(number)
        MazeDB.printMsg(
            'INFO', "Published projection command: command[%d]", number)
        
    
