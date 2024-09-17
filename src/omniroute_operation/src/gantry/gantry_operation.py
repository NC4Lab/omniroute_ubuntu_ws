#!/usr/bin/env python

# Custom Imports
from shared_utils.maze_debug import MazeDB
from shared_utils.esmacat_com import EsmacatCom
from gantry.gcodeclient import Client as GcodeClient

# ROS Imports
import rospy
from omniroute_operation.msg import *

# Other Imports
import time
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
from enum import Enum
import math


# Track the state for state machine
class GantryState(Enum):
    IDLE = 0
    TRACK_HARNESS = 1
    MOVE_TO_TARGET = 2
    LOWER_FEEDER = 3
    RAISE_FEEDER = 4
    START_PUMP = 5
    STOP_PUMP = 6
    REWARD = 7


class GantryOperation:
    # Initialize the GantryOperation class
    def __init__(self):
        MazeDB.printMsg('ATTN', "GANTRY_OPERATION NODE STARTED")

        # ................ Gantry Tracking Setup ................

        # PID values for normal operation
        self.Kp = 42.0
        self.Ti = 0.0
        self.Td = 0.0

        # For storing the PID control terms
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        # Initialize gantry coordinate class variables
        self.gantry_x = 0.0
        self.gantry_y = 0.0
        self.harness_x = 0.0
        self.harness_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0

        # Track if movement is in progress
        self.movement_in_progress = False

        # Specify the x and y offset from the gantry tracking marker to the gantry center (m)
        # self.gantry_marker_to_gantry_center = np.array([-0.285, -0.178])
        self.gantry_marker_to_gantry_center = np.array([-0.317, -0.185])

        # Paramters for positioning gantry
        self.chamber_wd = 0.3  # Chamber width (m)
        self.n_chamber_side = 3
        # Threshold distance for chamber entry from center(m)
        self.threshold = 0.06
        self.chamber_centers = []  # List of chamber centers

        # Compute the chamber centers
        for i in range(0, self.n_chamber_side**2):
            row = i//self.n_chamber_side
            col = i % self.n_chamber_side
            chamber_center = np.array([self.chamber_wd/2 + col*self.chamber_wd,
                                      self.chamber_wd/2 + (self.n_chamber_side-1-row)*self.chamber_wd])
            self.chamber_centers.append(chamber_center)

        # ................ ROS Setup ................

        # Initialize the subsrciber for reading in the gantry commands
        rospy.Subscriber('/gantry_cmd', GantryCmd,
                         self.gantry_cmd_callback, queue_size=1, tcp_nodelay=True)

        # Initialize the subsrciber for reading in the tracker position data
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped,
                         self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/gantry_pose_in_maze', PoseStamped,
                         self.gantry_pose_callback, queue_size=1, tcp_nodelay=True)

        # Initialize the gantry mode used for state machine
        self.gantry_mode = GantryState.IDLE

        # ................ Ecat Setup ................

        # Create EsmacatCom object for gantry_ease
        self.EsmaCom = EsmacatCom('gantry_ease')

        # Wait for 1 second
        time.sleep(1)

        # Send handshake command
        self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.HANDSHAKE)

        # Wait for 1 second
        time.sleep(1)

        # Send command to initialize GRBL for gantry
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_INITIALIZE_GRBL)

        # TEMP
        time.sleep(1)
        self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.GANTRY_HOME)
        time.sleep(5)
        # self.gantry_mode = GantryState.TRACK_HARNESS

        # ................ Run node ................

        # Initialize the ROS rate
        r = rospy.Rate(180)

        # Loop until the node is shutdown
        MazeDB.printMsg(
            'INFO', "[GantryOperation]: Initialzed gantry_operation_node")
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):

        # # Check for new message
        # TODO: Fix round trip timing
        # if not self.EsmaCom.rcvEM.isNew:
        #     return
        # self.procEcatMessage()

        if self.gantry_mode == GantryState.TRACK_HARNESS:

            # Get the current distance between the gantry and the harness
            gantry_to_harness = np.array(
                [self.harness_x - self.gantry_x, self.harness_y - self.gantry_y])
            distance = np.linalg.norm(gantry_to_harness)

            # # Use proportional control with deceleration
            # if distance > 0.15:
            #     x, y = self.proportional_control(
            #         gantry_to_harness, base_speed=35.0, slow_on_approach=False)

            #     # Send the movement command
            #     if ~np.isnan(x) and ~np.isnan(y):
            #         self.move_gantry_rel(x, y)

            #     self.movement_in_progress = True

            # Use PID values (tuned or hardcoded) for control
            if distance > 0.01:
                x, y = self.pid_control(
                    gantry_to_harness, self.Kp, self.Ti, self.Td)

                # Send the movement command
                if ~np.isnan(x) and ~np.isnan(y):
                    self.move_gantry_rel(x, y)

                self.movement_in_progress = True

            # Stop the gantry when it reaches the harness
            elif self.movement_in_progress:
                self.jog_cancel()
                self.movement_in_progress = False

        if self.gantry_mode == GantryState.MOVE_TO_TARGET:
            # Unit vector from gantry to target
            gantry_to_target = np.array(
                [self.target_x - self.gantry_x, self.target_y - self.gantry_y])

            # Use proportional control with deceleration
            x, y = self.proportional_control(
                gantry_to_target, base_speed=25.0, slow_on_approach=False)

            # Move the gantry to the target
            if ~np.isnan(x) and ~np.isnan(y):
                self.move_gantry_rel(x, y)

    def proportional_control(self, gantry_to_target, base_speed, slow_on_approach=True):
        """
        Proportional control to calculate movement based on the current distance and direction to the target.

        Parameters:
        - gantry_to_target: Array containing the difference between the gantry and target positions in X and Y.
        - base_speed: The base speed to scale the movement by.
        - slow_on_approach: A flag to indicate if the gantry should decelerate when approaching the target.
        """

        # Calculate the distance between the gantry and target
        distance = np.linalg.norm(gantry_to_target)

        # Avoid division by zero or small distances by checking the threshold
        if distance < 0.001:
            return 0, 0

        # Apply the speed, with optional deceleration as it approaches the target
        if slow_on_approach:
            # Scale speed by distance, slowing down as it approaches the target
            distance = np.linalg.norm(gantry_to_target)
            gantry_to_target = gantry_to_target / distance

        # Y component of the target movement vector
        output_y = base_speed * gantry_to_target[0]
        # X component of the target movement vector
        output_x = base_speed * gantry_to_target[1]

        return output_x, output_y

    def pid_control(self, gantry_to_harness, Kp, Ti, Td):
        """PID control to calculate movement based on current error with integral windup protection."""
        # Get the errors in x and y directions
        error_y = gantry_to_harness[0]
        error_x = gantry_to_harness[1]

        # Set limits for integral to prevent windup
        integral_limit = 100.0  # You can adjust this value based on system scale

        # Calculate the proportional term
        P_x = Kp * error_x
        P_y = Kp * error_y

        # Calculate the integral term with clamping to prevent windup
        self.integral_x += error_x
        self.integral_y += error_y
        self.integral_x = max(
            min(self.integral_x, integral_limit), -integral_limit)
        self.integral_y = max(
            min(self.integral_y, integral_limit), -integral_limit)

        I_x = (Kp / Ti) * self.integral_x if Ti != 0 else 0
        I_y = (Kp / Ti) * self.integral_y if Ti != 0 else 0

        # Calculate the derivative term (rate of change of error)
        D_x = Kp * Td * (error_x - self.prev_error_x)
        D_y = Kp * Td * (error_y - self.prev_error_y)

        # Update previous errors for the next cycle
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        # Compute final control outputs for x and y
        output_x = P_x + I_x + D_x
        output_y = P_y + I_y + D_y

        return output_x, output_y

    def jog_cancel(self):
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_JOG_CANCEL, do_print=False)

    def home(self):
        # Send command to home gantry
        self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.GANTRY_HOME)

    def move_gantry_rel(self, x, y):
        # Convert x and y to a list
        xy_list = [x, y]

        # Send command to move gantry
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_MOVE_REL, msg_arg_data_f32=xy_list, do_print=False)

    def gantry_cmd_callback(self, msg):
        if msg.cmd == "HOME":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Homing command received")
            self.home()

        elif msg.cmd == "MOVE_TO_COORDINATE":
            self.target_x = msg.args[0]
            self.target_y = msg.args[1]
            self.gantry_mode = GantryState.MOVE_TO_TARGET
            self.move_gantry_rel(0, 0)
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Move command received: target(%0.2f, %0.2f)", self.target_x, self.target_y)

        elif msg.cmd == "MOVE_TO_CHAMBER":
            chamber_num = int(msg.args[0])
            self.target_x = self.chamber_centers[chamber_num][0]
            self.target_y = self.chamber_centers[chamber_num][1]
            self.gantry_mode = GantryState.MOVE_TO_TARGET
            self.move_gantry_rel(0, 0)
            MazeDB.printMsg('DEBUG', "[GantryOperation]: Move to chamber command received: chamber(%d) target(%0.2f, %0.2f)",
                            chamber_num, self.target_x, self.target_y)

        elif msg.cmd == "TRACK_HARNESS":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Track Hhrness command received")
            self.gantry_mode = GantryState.TRACK_HARNESS

        elif msg.cmd == "IDLE":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Idle command received")
            self.gantry_mode = GantryState.IDLE

        elif msg.cmd == "LOWER_FEEDER":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Lower Feeder command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_SET_FEEDER, 1)

        elif msg.cmd == "RAISE_FEEDER":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Raise Feeder command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_SET_FEEDER, 0)

        elif msg.cmd == "START_PUMP":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Start Pump command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_RUN_PUMP, 1)

        elif msg.cmd == "STOP_PUMP":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Stop Pump command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_RUN_PUMP, 0)

        elif msg.cmd == "REWARD":
            duration = msg.args[0]  # Duration in seconds
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Reward command received: duration(%d)", duration)
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_REWARD, msg_arg_data_f32=duration)

    def gantry_pose_callback(self, msg):
        self.gantry_x = msg.pose.position.x + \
            self.gantry_marker_to_gantry_center[0]
        self.gantry_y = msg.pose.position.y + \
            self.gantry_marker_to_gantry_center[1]
        # rospy.loginfo("Gantry pose: "f'x: {self.gantry_x}, y: {self.gantry_y}')

    def harness_pose_callback(self, msg):
        self.harness_x = msg.pose.position.x
        self.harness_y = msg.pose.position.y
        # rospy.loginfo("Harness pose: "f'x: {self.harness_x}, y: {self.harness_y}')

    def procEcatMessage(self):
        """ Used to parse new incoming ROS ethercat msg data. """

        # ................ Process Ack Error First ................

        if self.EsmaCom.rcvEM.errTp != EsmacatCom.ErrorType.ERR_NONE:

            MazeDB.printMsg('ERROR', "(%d)ECAT ERROR: %s",
                            self.EsmaCom.rcvEM.msgID, self.EsmaCom.rcvEM.errTp.name)

        # ................ Process Ack Message ................

        # HANDSHAKE
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.HANDSHAKE:
            MazeDB.printMsg('INFO', "Gantry Handshake Confirmed")

            # Set the handshake flag
            self.EsmaCom.isEcatConnected = True

        # GANTRY_MOVE_REL
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.GANTRY_MOVE_REL:
            pass

        # Reset new message flag
        self.EsmaCom.rcvEM.isNew = False


# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'gantry_operation'
    rospy.init_node('gantry_operation')
    GantryOperation()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown
