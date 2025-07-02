#!/usr/bin/env python
"""
Class for publishing commands to the projection system
"""

# Custom Imports
from shared_utils.maze_debug import MazeDB

# ROS Imports
import rospy
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension, String

# Other
import csv
import json

class ProjectionOperation:
    def __init__(self):
        self.num_chambers = 9  # Number of chambers
        self.num_surfaces = 9  # Number of surfaces (8 walls + 1 floor)
        self.blank_image_config = [[0 for _ in range(self.num_surfaces)] for _ in range(self.num_chambers)]

        # Initialize image_config as a 10x8 array with default values
        self.image_config = self.blank_image_config

        # Create the publisher for 'projection_cmd' topic
        self.projection_pub = rospy.Publisher('projection_cmd', Int32, queue_size=10)

        # Create the publisher for the 'projection_image' topic
        self.image_pub = rospy.Publisher('projection_image', Int32MultiArray, queue_size=10)
        
        # rospy.Subscriber('projection_image_floor_num', Int32, self.projection_image_floor_callback)

        # rospy.Subscriber('projection_walls', String, self.projection_walls_callback)

        # rospy.Subscriber('projection_image_wall_num', Int32, self.projection_image_wall_callback)

        self.wall_image_num = None  
        self.cham_ind = None
        self.wall_ind = None
    
    #TODO Be able to use a string to set the wall image
    def set_wall_image(self, chamber, wall, image_index):
        if (wall < 0 or wall > 7) or (chamber < 0 or chamber > 8):
            rospy.logwarn(f"[projection_sender:set_wall_image] Invalid chamber {chamber} or wall {wall}. Must be in range [0, 8] and [0, 7] respectively.")
            return
        
        # Set the image index for the specified chamber and wall
        self.image_config[chamber][wall] = image_index
    
    def set_floor_image(self, image_index):
        for chamber in range(9):
            self.image_config[chamber][8] = image_index

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

        Arguments:
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

        Arguments:
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

    def publish_image_message(self, image_config):
        """
        Send the data from the CSV as an Int32MultiArray message.
                
        Args:
            image_config (list): A 10x8 list that will be modified in place.
        """
        # Create the Int32MultiArray message
        projection_data = Int32MultiArray()
        projection_data.layout.dim = self.setup_layout(9, 9)  

        # Flatten the 9x9 array into a single list
        flat_data = [image_config[i][j] for i in range(9) for j in range(9)]
        projection_data.data = flat_data

        # Publish the CSV data message
        self.image_pub.publish(projection_data)

        # Log the sent message data
        # rospy.loginfo("[publish_image_message] Sent the following data:")
        # for i in range(9):
        #     rospy.loginfo("Data[%d] = %s", i, str(image_config[i]))

    def publish_command_message(self, number):
        # Can send any number
        self.projection_pub.publish(number)
        # MazeDB.printMsg(
        #     'INFO', "Published projection command: command[%d]", number)
    
if __name__ == '__main__':
    ProjectionOperation()
    rospy.spin()