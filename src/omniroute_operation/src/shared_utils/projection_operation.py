#!/usr/bin/env python
"""
Class for publishing commands to the projection system
"""

# Custom Imports
from shared_utils.maze_debug import MazeDB

# ROS Imports
import rospy, rospkg
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension

from enum import IntEnum

# Other
import csv
import os
import numpy as np

class ProjectionCmd(IntEnum):
    TOGGLE = -1
    FULLSCREEN = -2
    FORCE_FOCUS = -3

class ProjectionOperation:
    def __init__(self):
        self.num_chambers = 9  # Number of chambers
        self.num_surfaces = 9  # Number of surfaces (8 walls + 1 floor)
        self.blank_image_config = np.zeros((self.num_chambers, self.num_surfaces), dtype=int)
        self.unchanged_image_config = np.full((self.num_chambers, self.num_surfaces), -1, dtype=int)

        # Read available images from the ROS parameter server
        self.runtime_wall_images = rospy.get_param('runtime_wall_images', [])
        self.runtime_floor_images = rospy.get_param('runtime_floor_images', [])

        # Create the publisher for 'projection_cmd' topic
        self.projection_pub = rospy.Publisher('projection_cmd', Int32, queue_size=10)
        # Create the publisher for the 'projection_image' topic
        self.image_pub = rospy.Publisher('projection_image', Int32MultiArray, queue_size=10)

        self.reset_image_config()

        MazeDB.printMsg('INFO', "ProjectionOperation initialized with %d chambers and %d surfaces.", self.num_chambers, self.num_surfaces)
        MazeDB.printMsg('DEBUG', "Available wall images: %s", str(self.runtime_wall_images))
        MazeDB.printMsg('DEBUG', "Available floor images: %s", str(self.runtime_floor_images))
    
    def reset_image_config(self):
        """ Reset the image configuration to the initial blank state. """
        self.image_config = np.copy(self.unchanged_image_config)
        MazeDB.printMsg('DEBUG', "[projection_sender:reset_image_config] Image configuration reset to blank values.")

    def blank_maze(self, publish: bool=False):
        """ Reset the image configuration for all chambers to blank values. 
        
        Args:
            publish (bool): If True, publish the blank configuration to the projection system.
        """
        self.image_config = np.copy(self.blank_image_config)
        if publish:
            self.publish_image_message()
        MazeDB.printMsg('DEBUG', "[projection_sender:blank_maze] All chambers reset to blank configuration.")
    
    def blank_chamber(self, chamber: list | int, publish=False):
        """ Reset the image configuration for a specific chamber to blank values.
        
        Args:
            chamber (int or list): The chamber number (0-8) to reset, or a list of chamber numbers.
                If a list is provided, all specified chambers will be reset.
                If an integer is provided, only that chamber will be reset.
            publish (bool): If True, publish the blank configuration to the projection system.
        """
        if isinstance(chamber, list):
            for c in chamber:
                self.blank_chamber(c, publish=False)
        else:
            if chamber < 0 or chamber >= self.num_chambers:
                MazeDB.printMsg('WARN', "[projection_sender:blank_chamber] Invalid chamber %d. Must be in range [0, 8].", chamber)
                return
            
            # Set the specified chamber's configuration to blank
            self.image_config[chamber, :] = np.copy(self.blank_image_config[chamber, :])
        
        
        if publish:
            self.publish_image_message()
        MazeDB.printMsg('DEBUG', "[projection_sender:blank_chamber] Chamber %d reset to blank configuration.", chamber)
    
    def set_wall_image(self, chamber: list|int, wall: list|int, image_index: int|str, publish=False):
        """ Set the image index for a specific wall in a specific chamber.
        Args:
            chamber (list or int): The chamber number (0-8) to set the wall image for, or a list of chamber numbers.
                If a list is provided, the wall image will be set for all specified chambers.
                If an integer is provided, only that chamber will be set.
            wall (list or int): The wall number (0-7) to set the image for, or a list of wall numbers.
                If a list is provided, the wall image will be set for all specified walls.
                If an integer is provided, only that wall will be set.
            image_index (int or str): The index of the image to set, or the name
                of the image as a string. If a string is provided, it will be
                looked up in the runtime_wall_images list.
            publish (bool): If True, publish the updated configuration to the projection system.
        """
        # Update runtime_wall_images
        self.runtime_wall_images = rospy.get_param('runtime_wall_images', [])

        if (wall < 0 or wall > 7) or (chamber < 0 or chamber > 8):
            MazeDB.printMsg('WARN', "[projection_sender:set_wall_image] Invalid chamber %d or wall %d. Must be in range [0, 8] and [0, 7] respectively.", chamber, wall)
            return
        
        if isinstance(image_index, str):
            # If image_index is a string, try to find its index in runtime_wall_images
            if image_index in self.runtime_wall_images:
                image_index = self.runtime_wall_images.index(image_index)
            else:
                MazeDB.printMsg('WARN', "[projection_sender:set_wall_image] Image '%s' not found in runtime_wall_images.", image_index)
                return
        
        if image_index < 0 or image_index >= len(self.runtime_wall_images):
            MazeDB.printMsg('WARN', "[projection_sender:set_wall_image] Invalid image index %d. Must be in range [0, %d].", image_index, len(self.runtime_wall_images) - 1)
            return
        
        # Set the image index for the specified chamber and wall
        self.image_config[chamber][wall] = image_index

        if publish:
            self.publish_image_message()
        MazeDB.printMsg('DEBUG', "[projection_sender:set_wall_image] Set chamber %d, wall %d to image index %d.", chamber, wall, image_index)
    
    def set_floor_image(self, image_index, publish=False):
        """ Set the image index for the floor in all chambers.
        Args:
            image_index (int or str): The index of the image to set, or the name
                of the image as a string. If a string is provided, it will be
                looked up in the runtime_floor_images list.
            publish (bool): If True, publish the updated configuration to the projection system.
        """
        # Update runtime_floor_images
        self.runtime_floor_images = rospy.get_param('runtime_floor_images', [])

        if isinstance(image_index, str):
            # If image_index is a string, try to find its index in runtime_floor_images
            if image_index in self.runtime_floor_images:
                image_index = self.runtime_floor_images.index(image_index)
            else:
                MazeDB.printMsg('WARN', "[projection_sender:set_floor_image] Image '%s' not found in runtime_floor_images.", image_index)
                return

        if image_index < 0 or image_index >= len(self.runtime_floor_images):
            MazeDB.printMsg('WARN', "[projection_sender:set_floor_image] Invalid image index %d. Must be in range [0, %d].", image_index, len(self.runtime_floor_images) - 1)
            return

        for chamber in range(9):
            self.image_config[chamber][8] = image_index
        
        if publish:
            self.publish_image_message()
        MazeDB.printMsg('DEBUG', "[projection_sender:set_floor_image] Set all chambers to floor image index %d.", image_index)

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

    def set_config_from_csv(self, file_name):
        """ Set the image configuration from a CSV file.
        Args:
            image_config (list): A 9x9 list
            file_name (str): The name of the CSV file to read from.
        """
        self.image_config = self.blank_image_config
        with open(file_name, mode='r') as csvfile:
            csv_reader = csv.reader(csvfile)

            data = []
            for row in csv_reader:
                # Skip empty and commented rows
                if not row or row[0].startswith('#'):
                    continue
                data.append([int(value) for value in row])


            if len(data) != self.num_chambers:
                MazeDB.printMsg('WARN', "[projection_sender:set_config_from_csv] Expected %d rows, got %d", self.num_chambers, len(data))
                return
            for i, row in enumerate(data):
                if len(row) != self.num_surfaces:
                    MazeDB.printMsg('WARN', "[projection_sender:set_config_from_csv] Expected %d columns, got %d in row %d", self.num_surfaces, len(row), i)
                    return
                
                for j in range(self.num_surfaces):
                    try:
                        self.image_config[i][j] = int(row[j])
                    except ValueError:
                        MazeDB.printMsg('WARN', "[projection_sender:set_config_from_csv] Invalid value '%s' at row %d, column %d", row[j], i, j)
                        return
        

    def publish_image_message(self, image_config = None):
        """
        Send the data from the CSV as an Int32MultiArray message.
                
        Args:
            image_config (list): A 9x9 list of integers representing the image configuration.
            If None, the class variable image_config will be used.
        """
        if image_config is None:
            image_config = self.image_config

        # Create the Int32MultiArray message
        projection_data = Int32MultiArray()
        projection_data.layout.dim = self.setup_layout(9, 9)  

        # Flatten the 9x9 array into a single list
        projection_data.data = image_config.flatten().tolist()

        # Publish the CSV data message
        self.image_pub.publish(projection_data)

        # Log the sent message data
        MazeDB.printMsg('INFO', "[publish_image_message] Sent data")
        for i in range(9):
            MazeDB.printMsg('DEBUG', "Data[%d] = %s", i, str(image_config[i]))

    def publish_command_message(self, number):
        """
        Publish a command to the projection system.
        Args:
            number (int or str): The command number to publish. If a string is provided,
                it will be looked up in the projection_cmds dictionary.
        """
        self.projection_pub.publish(number)
        MazeDB.printMsg('INFO', "Published projection command: command[%d]", number)
    
if __name__ == '__main__':
    ProjectionOperation()
    rospy.spin()