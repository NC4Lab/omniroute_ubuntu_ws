class WallConfig:
    """ 
    Used to stores the wall configuration of the maze for CSV and Ethercat for the maze.
    """

    #------------------------ CLASS VARIABLES ------------------------

    # Stores the wall configuration list
    wallConfigList = []

    #------------------------ CLASS METHODS ------------------------

    def reset(self):
        """Resets the wall configuration list"""
        self.wallConfigList = []

    def get_len(self):
        """Returns the number of entries in the wall configuration list"""
        return len(self.wallConfigList)

    def add_wall(self, chamber_num, wall_num):
        """Adds a wall to the wall configuration list"""
        for item in self.wallConfigList:
            if item[0] == chamber_num:
                item[1].append(wall_num)
                return
        self.wallConfigList.append([chamber_num, [wall_num]])

    def remove_wall(self, chamber_num, wall_num):
        """Removes a wall from the wall configuration list"""
        for item in self.wallConfigList:
            if item[0] == chamber_num:
                item[1].remove(wall_num)
                if not item[1]:  # If the second column is empty, remove the entire row
                    self.wallConfigList.remove(item)
                return

    def make_byte_2_wall_list(self, wall_byte_config_list):
        """
        Used to convert imported CSV with wall byte mask values to a list with wall numbers

        Args:
            wall_byte_config_list (list): 2D list: col1 = chamber number, col2 = wall byte mask
        
        Returns:
            2D list: col1 = chamber number, col2 = nested wall numbers
        """

        # Clear the existing wall_config_list
        self.wallConfigList = []

        # Convert the byte values to arrays and update the wall_config_list
        for row in wall_byte_config_list:
            chamber_num = row[0]
            byte_value = row[1]

            # Convert the byte_value back to an array of wall numbers
            wall_numbers = [i for i in range(8) if byte_value & (1 << i)]

            self.wallConfigList.append([chamber_num, wall_numbers])

            return self.wallConfigList

    def make_wall_2_Byte_list(self):
        """
        Used to covert wall number arrays to byte values for saving to CSV
        
        Returns:
            2D list: col1 = chamber number, col2 = wall byte mask"""  

        _wall_byte_config_list = []
        for row in self.wallConfigList:  # row = [chamber_num, wall_numbers]
            chamber_num = row[0]
            wall_arr = row[1]
            # Initialize the byte value
            byte_value = 0
            # Iterate over the array of values
            for wall_i in wall_arr:
                if 0 <= wall_i <= 7:
                    # Set the corresponding bit to 1 using bitwise OR
                    byte_value |= (1 << wall_i)
            _wall_byte_config_list.append([chamber_num, byte_value])

        return _wall_byte_config_list

    def get_wall_byte_list(self):
        """
        Used to generate a 1D list with only byte values for each chamber corespoinding to the wall configuration
        For use with the EsmacatCom class
        
        Returns: 
            1D list with byte values for all chambers"""

        wall_byte_config_list = self.make_wall_2_Byte_list()

        # Update U_arr with corresponding chamber and wall byte
        _wall_byte_arr = [0] * N_CHAMBERS
        #wall_arr = [0] * len(self.wallConfigList)
        for cw in wall_byte_config_list:
            _wall_byte_arr[cw[0]] = cw[1]

        return _wall_byte_arr

    def _sort_entries(self):
        """Sorts the entries in the wall configuration list by chamber number and wall numbers"""

        # Sort the rows by the entries in the based on the first chamber number
        self.wallConfigList.sort(key=lambda row: row[0])

        # Sort the arrays in the second column
        for row in self.wallConfigList:
            row[1].sort()

    def __iter__(self):
        """Returns an iterator for the wall configuration list"""
        return iter(self.wallConfigList)

    def __str__(self):
        """Returns the wall configuration list as a string"""
        return str(self.wallConfigList)