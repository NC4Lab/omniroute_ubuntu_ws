#!/usr/bin/env python
import os
import rospy


def initialize_shared_state():
    """
    Initialize shared state variables using the ROS Parameter Server.
    This function will set up the default values on the ROS Parameter Server.
    """

    # Get the absolute path of the current script file
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Specify the default wall path directory
    wall_cfg_dir_default = os.path.abspath(os.path.join(
        script_dir, '..', '..', '..', '..', 'data', 'wall_config', 'experiments'))

    # Specify the default wall projection config directory
    proj_cfg_dir_default = os.path.abspath(os.path.join(
        script_dir, '..', '..', '..', '..', 'data', 'projection', 'image_config'))

    # Set parameters on the ROS parameter server
    rospy.set_param('/shared_state/wall_cfg_dir_default', wall_cfg_dir_default)
    rospy.set_param('/shared_state/proj_cfg_dir_default', proj_cfg_dir_default)
    rospy.set_param('/shared_state/is_arduinos_connected', False)
    rospy.set_param('/shared_state/is_maze_initialized', False)

    rospy.loginfo("Shared state initialized on the ROS Parameter Server.")


if __name__ == "__main__":
    # Initialize the ROS node (for parameter management)
    rospy.init_node('shared_state_initializer', anonymous=True)

    # Call function to initialize shared state
    initialize_shared_state()
