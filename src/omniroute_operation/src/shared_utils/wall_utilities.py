#!/usr/bin/env python
"""
Classes for common wall operations.
"""

# ROS Imports
import rospy

# Custom Imports
from shared_utils.maze_debug import MazeDB

# Standard Library Imports
import os
import csv
import numpy as np



class MazeDimensions:
    """
    Class for storing the dimensions of the maze
    """
    
    # Wall map for 3x3 maze [chamber_num][wall_num]
    WallMap = {  
        0: [0, 1, 2, 3, 4, 5, 6, 7],
        1: [1, 2, 3, 5, 7],
        2: [0, 1, 2, 3, 4, 5, 6, 7],
        3: [0, 1, 3, 5, 7],
        4: [0, 1, 2, 3, 4, 5, 6, 7],
        5: [1, 3, 4, 5, 7],
        6: [0, 1, 2, 3, 4, 5, 6, 7],
        7: [1, 3, 5, 6, 7],
        8: [0, 1, 2, 3, 4, 5, 6, 7]
    }

    def __init__(self):
        self.chamber_wd = 0.3  # Chamber width (m)
        self.n_chamber_side = 3
        self.chamber_centers = []  # List of chamber centers

        # Compute the chamber centers
        for i in range(0, self.n_chamber_side**2):
            row = i//self.n_chamber_side
            col = i % self.n_chamber_side
            chamber_center = np.array([self.chamber_wd/2 + col*self.chamber_wd,
                                    self.chamber_wd/2 + (self.n_chamber_side-1-row)*self.chamber_wd])
            self.chamber_centers.append(chamber_center)

class WallConfig:
    """ 
    Class to stores the wall configuration of the maze for CSV and Ethercat for the maze.
    """

    # ------------------------ CLASS VARIABLES ------------------------

    # Stores the wall configuration list
    wall_cfg_num_list = []  # cham_num x [wall_num]
    wall_cfg_byte_list = []  # cham_num x wall_byte

    # ------------------------ CLASS METHODS ------------------------

    @classmethod
    def reset(cls):
        """Resets the wall configuration list"""

        cls.wall_cfg_num_list = []
        cls.wall_cfg_byte_list = []

    @classmethod
    def get_len(cls):
        """Returns the number of entries in the wall configuration list"""

        return len(cls.wall_cfg_num_list)

    @classmethod
    def add_wall(cls, chamber_num, wall_num):
        """Adds a wall to the wall configuration list"""
        for item in cls.wall_cfg_num_list:
            if item[0] == chamber_num:
                if wall_num not in item[1]:
                    item[1].append(wall_num)
                return
        cls.wall_cfg_num_list.append([chamber_num, [wall_num]])

    @classmethod
    def remove_wall(cls, chamber_num, wall_num):
        """Removes a wall from the wall configuration list"""
        for item in cls.wall_cfg_num_list[:]:  # Iterate over a copy of the list
            if item[0] == chamber_num:
                if wall_num in item[1]:
                    item[1].remove(wall_num)
                    if not item[1]:  # If the second column is empty, remove the entire row
                        cls.wall_cfg_num_list.remove(item)
                    return

    @classmethod
    def make_byte2num_cfg_list(cls, _wall_cfg_byte_list, do_print=False):
        """
        Used to convert imported CSV with wall byte mask values to a list with wall numbers

        Arguments:
            _wall_cfg_byte_list (list): 2D list: col_1 = chamber number, col_2 = wall byte mask

        Returns:
            2D list: col_1 = chamber number, col_2 = nested wall numbers
        """

        # Clear/reset the existing wall_config_list
        cls.reset()

        # Convert the byte values to arrays and update the wall_config_list
        for row in _wall_cfg_byte_list:
            chamber_num = row[0]
            byte_value = row[1]

            # Convert the byte_value back to an array of wall numbers
            wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

            cls.wall_cfg_num_list.append([chamber_num, wall_numbers])

        # loop thorugh wall_byte_config_list and print each element
        if do_print:
            for row in cls.wall_cfg_num_list:
                MazeDB.printMsg('DEBUG', "[%d][%d]", row[0], row[1])

        return cls.wall_cfg_num_list

    @classmethod
    def make_num2byte_cfg_list(cls):
        """
        Used to covert wall number arrays to byte values for saving to CSV

        Returns:
            2D list: col_1 = chamber number, col_2 = wall byte mask
        """

        cls.wall_cfg_byte_list = []

        for row in cls.wall_cfg_num_list:  # row = [chamber_num, wall_numbers]
            chamber_num = row[0]
            wall_arr = row[1]

            # Initialize the byte value
            byte_value = 0
            # Iterate over the array of values
            for wall_i in wall_arr:
                if 0 <= wall_i <= 7:
                    # Set the corresponding bit to 1 using bitwise OR
                    byte_value |= (1 << wall_i)
            cls.wall_cfg_byte_list.append([chamber_num, byte_value])

        return cls.wall_cfg_byte_list

    @classmethod
    def get_wall_byte_list(cls):
        """
        Used to generate a 1D list with only byte values for each chamber corespoinding to the wall configuration
        For use with the EsmacatCom class

        Returns: 
            1D list with byte values for all chambers
        """

        cls.wall_cfg_byte_list = cls.make_num2byte_cfg_list()

        # Update U_arr with corresponding chamber and wall byte
        _wall_byte_list = [0] * len(MazeDimensions.WallMap)
        for cw in cls.wall_cfg_byte_list:
            _wall_byte_list[cw[0]] = cw[1]

        return _wall_byte_list

    @classmethod
    def _sort_entries(cls):
        """Sorts the entries in the wall configuration list by chamber number and wall numbers"""

        # Sort the rows by the entries in the based on the first chamber number
        cls.wall_cfg_num_list.sort(key=lambda row: row[0])

        # Sort the arrays in the second column
        for row in cls.wall_cfg_num_list:
            row[1].sort()

    @classmethod
    def __iter__(cls):
        """Returns an iterator for the wall configuration list"""
        return iter(cls.wall_cfg_num_list)

    @classmethod
    def __str__(cls):
        """Returns the wall configuration list as a string"""
        return str(cls.wall_cfg_num_list)
    
    @classmethod
    def load_from_csv(cls, dir_path, file_name):
        """ Function to load wall config data from a CSV file and update wall configuration """

        # Join the directory path and file name to get the full path
        file_path = os.path.join(dir_path, file_name)

        # Attempt to open the file and read the data
        try:
            with open(file_path, 'r') as csv_file:
                csv_reader = csv.reader(csv_file)
                # Read each row and convert the data to integers, representing chamber and wall configuration
                wall_byte_config_list = [
                    [int(row[0]), int(row[1])] for row in csv_reader]

                # Update the wall configuration 
                cls.make_byte2num_cfg_list(wall_byte_config_list)
                MazeDB.printMsg(
                    'INFO', "CSV: Data Loaded from File: %s", file_name)
        except Exception as e:
            # Print an error message if loading the CSV file fails
            MazeDB.printMsg('ERROR', "CSV: Loading Data Error: %s", str(e))

    @classmethod
    def save_to_csv(cls, save_file_path, wall_config_list):
        """ Function to save the wall config data to a CSV file """

        try:
            # Open the file for writing and use a CSV writer to write each row of wall configuration data
            with open(save_file_path, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                for row in wall_config_list:
                    csv_writer.writerow(row)

            # Extract the file name from the full path to log it
            save_file_name = os.path.basename(save_file_path)
            MazeDB.printMsg(
                'INFO', "CSV: Data Saved to File: %s", save_file_name)

        except Exception as e:
            # Print an error message if saving the CSV file fails
            MazeDB.printMsg('ERROR', "CSV: Saving Data Error: %s", str(e))
