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

        # ................ GRBL Runtime Parameters ................
        self.max_feed_rate = 1000  # Maxiumum feed rate (mm/min)
        self.max_acceleration = 300  # Maximum acceleration (mm/sec^2)

        # ................ Serial Setup ................

        # Flag if using serial communication
        self.use_serial = False

        # Setup serial communication
        if self.use_serial:

            # GCode client setup
            self.gcode_client = GcodeClient('/dev/ttyUSB0', 115200)
            time.sleep(1)

            # Set Units (mm)
            self.gcode_client.raw_command("G21")

            # Set Mode (G90 = Absolute, G91 = Relative)
            self.gcode_client.raw_command("G91")

            # Feed Rate (mm/min)
            feed_rate_command = f"F{self.max_feed_rate}"
            self.gcode_client.raw_command(feed_rate_command)

            # Max Acceleration for X and Y axes (mm/sec^2)
            # X-axis acceleration
            acceleration_command_x = f"$120={self.max_acceleration}"
            # Y-axis acceleration
            acceleration_command_y = f"$121={self.max_acceleration}"
            self.gcode_client.raw_command(acceleration_command_x)
            self.gcode_client.raw_command(acceleration_command_y)

        # ................ Gantry Tracking Setup ................

        # Flag to run auto-tuning
        self.track_method = "jog"  # Choose between "prop", "pid", or 'jog'

        # Limit to prevent integral windup
        self.integral_limit = 100.0

        # PID parameters
        self.Kp = 40.0
        self.Kd = 5.0
        self.Ki = 0.0

        # PID tracking parameters
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_time = time.time()

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
        self.gantry_marker_to_gantry_center = np.array([-0.317, -0.185])

        # Paramters for positioning gantry
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
        init_list = [self.max_feed_rate, self.max_acceleration]
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_INITIALIZE_GRBL, msg_arg_data_f32=init_list)

        # # TEMP
        # time.sleep(1)
        # self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.GANTRY_HOME)
        # time.sleep(5)

        # ................ Run node ................

        # Initialize the ROS rate
        r = rospy.Rate(50)

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

            # Use PID values (tuned or hardcoded) for control
            if self.track_method == 'pid':
                if distance > 0.01:

                    # Calculate time step
                    current_time = time.time()
                    dt = current_time - self.prev_time

                    # Compute PID for x
                    x, self.integral_x, self.prev_error_x = self.pid_control(
                        error=gantry_to_harness[0],
                        Kp=self.Kp,
                        Ki=self.Ki,
                        Kd=self.Kd,
                        integral=self.integral_x,
                        prev_error=self.prev_error_x,
                        dt=dt,
                        integral_limit=self.integral_limit)

                    # Compute PID for y
                    y, self.integral_y, self.prev_error_y = self.pid_control(
                        error=gantry_to_harness[1],
                        Kp=self.Kp,
                        Ki=self.Ki,
                        Kd=self.Kd,
                        integral=self.integral_y,
                        prev_error=self.prev_error_y,
                        dt=dt,
                        integral_limit=self.integral_limit)

                    # Send the movement command
                    if ~np.isnan(x) and ~np.isnan(y):
                        self.move_gantry_rel(x, y, self.max_feed_rate)
                        self.movement_in_progress = True

                    # Update the time
                    self.prev_time = current_time

                # Stop the gantry when it reaches the harness
                elif self.movement_in_progress:
                    self.jog_cancel()
                    self.movement_in_progress = False

            # Use proportional control
            elif self.track_method == 'prop':
                if distance > 0.1:
                    x, y = self.proportional_control(
                        gantry_to_harness, Kp=25.0, slow_on_approach=False)

                    # Send the movement command
                    if ~np.isnan(x) and ~np.isnan(y):
                        self.move_gantry_rel(x, y, self.max_feed_rate)
                        self.movement_in_progress = True

                # Stop the gantry when it reaches the harness
                elif self.movement_in_progress:
                    self.jog_cancel()
                    self.movement_in_progress = False

            # Use jog computation
            elif self.track_method == 'jog':

                # Calculate time step
                current_time = time.time()
                dt = current_time - self.prev_time

                if distance > 0.1:

                    # Compute jog distance
                    jog_distance = self.compute_jog(
                        gantry_to_harness, self.max_feed_rate, dt, 1)

                    # Send the movement command
                    self.move_gantry_rel(
                        jog_distance[0], jog_distance[1], self.max_feed_rate)

                    # Update the time
                    self.prev_time = current_time

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
                gantry_to_target, Kp=25.0, slow_on_approach=False)

            # Move the gantry to the target
            if ~np.isnan(x) and ~np.isnan(y):
                self.move_gantry_rel(x, y, self.max_feed_rate)

    def compute_jog(self, gantry_to_target, max_feed_rate, dt_sec, Kp):
        """
        Compute jog increments for the gantry based on position error.

        Args:
        - gantry_to_target: numpy array [dx, dy] in mm.
        - max_feed_rate: scalar max feed rate in mm/min.
        - dt_sec: time interval between control loop iterations in seconds.
        - Kp: Proportional gain.

        Returns:
        - jog_distance: numpy array [jog_x, jog_y] in mm.
        """

        # Convert gantry_to_target from m to mm
        gantry_to_target_mm = gantry_to_target * 1000.0  # mm

        # Compute desired velocity (mm/sec)
        desired_velocity = Kp * gantry_to_target_mm  # mm/sec

        # Convert max feed rate to mm/sec
        max_feed_rate_mm_per_sec = max_feed_rate / 60.0  # mm/sec

        # Limit the desired velocity to the max feed rate
        desired_velocity = np.clip(desired_velocity,
                                   -max_feed_rate_mm_per_sec,
                                   max_feed_rate_mm_per_sec)

        # Compute jog distance (mm)
        jog_distance = desired_velocity * dt_sec

        # TEMP print all the parameters
        MazeDB.printMsg('INFO', "gantry_to_target_mm[%0.2f, %0.2f] desired_velocity[%0.2f, %0.2f] jog_distance[%0.2f, %0.2f] dt_sec[%0.2f]",
                        gantry_to_target_mm[0], gantry_to_target_mm[1], desired_velocity[0], desired_velocity[1], jog_distance[0], jog_distance[1], dt_sec)

        return jog_distance

    def pid_control(self, error, Kp, Ki, Kd, integral, prev_error, dt, integral_limit=None):
        """
        PID control to calculate movement based on current error with integral windup protection for a single axis.

        Args:
        - error: The difference between the gantry and target position for a single axis.
        - Kp: Proportional gain.
        - Ki: Integral gain.
        - Kd: Derivative gain.
        - integral: The integral term for the given axis.
        - prev_error: The previous error for the given axis.
        - dt: Time step between current and previous error measurements.
        - integral_limit: Optional maximum value to limit integral windup.

        Returns:
        - output (float): Updated output for the given axis.
        - integral (float): Updated integral term for the given axis.
        - prev_error (float): Updated previous error for the given axis.
        """

        # Proportional term
        P = Kp * error

        # Integral term with windup protection
        integral += error * dt
        if integral_limit is not None:
            integral = max(min(integral, integral_limit), -integral_limit)

        I = Ki * integral

        # Derivative term (preventing divide by zero in case dt is very small)
        D = Kd * (error - prev_error) / dt if dt > 0 else 0

        # Calculate the output
        output = P + I + D

        # Update previous error
        prev_error = error

        return output, integral, prev_error

    def proportional_control(self, gantry_to_setpoint, Kp, slow_on_approach=True):
        """
        Proportional control to calculate movement based on the current distance and direction to the target.

        Args:
        - gantry_to_setpoint: Array containing the difference between the gantry and target positions in X and Y.
        - Kp: Proportional gain.
        - slow_on_approach: A flag to indicate if the gantry should decelerate when approaching the target.
        """

        # Calculate the distance between the gantry and target
        distance = np.linalg.norm(gantry_to_setpoint)

        # Avoid division by zero or small distances by checking the threshold
        if distance < 0.001:
            return 0, 0

        # Apply the speed, with optional deceleration as it approaches the target
        if slow_on_approach:
            gantry_to_setpoint = gantry_to_setpoint / distance

        # Y component of the target movement vector
        output_x = Kp * gantry_to_setpoint[0]
        # X component of the target movement vector
        output_y = Kp * gantry_to_setpoint[1]

        # Cap output to 10
        max_move = 10
        output_x = min(max(output_x, -max_move), max_move)
        output_y = min(max(output_y, -max_move), max_move)

        return output_x, output_y

    def jog_cancel(self):
        # Use serial to cancel jog
        if self.use_serial:
            # Cancel the jog
            self.gcode_client.raw_command(bytes([0x85]))
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_JOG_CANCEL, do_print=False)

    def home(self):

        # Use serial for homing
        if self.use_serial:
            # Home the gantry
            self.gcode_client.raw_command("$25=7500")  # Set homing speed
            self.gcode_client.raw_command("$H")  # Home the gantry
            # Set the current position to 0
            self.gcode_client.raw_command("G10 P0 L20 X0 Y0 Z0")

        # Send command to home gantry
        else:
            self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.GANTRY_HOME)

    def move_gantry_rel(self, x, y, max_feed_rate):

        # Flip x and y to account for gantry orientation and store to a list
        xy_list = [y, x]

        # Use serial for move command
        if self.use_serial:
            
            # Format the command string
            cmd = "$J=G91 G21 X{:.1f} Y{:.1f} F{}".format(xy_list[0], xy_list[1], max_feed_rate) 
            
            # Send the command
            self.gcode_client.raw_command(cmd)

        # Send command to move gantry
        else:
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_MOVE_REL, msg_arg_data_f32=xy_list, do_print=False)

    def gantry_pose_callback(self, msg):
        # Store x
        self.gantry_x = msg.pose.position.x + \
            self.gantry_marker_to_gantry_center[0]
        # Store y
        self.gantry_y = msg.pose.position.y + \
            self.gantry_marker_to_gantry_center[1]

    def harness_pose_callback(self, msg):
        # Store x
        self.harness_x = msg.pose.position.x
        # Store y
        self.harness_y = msg.pose.position.y

    def gantry_cmd_callback(self, msg):
        if msg.cmd == "HOME":
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Homing command received")
            self.home()

        elif msg.cmd == "MOVE_TO_COORDINATE":
            self.target_x = msg.args[0]
            self.target_y = msg.args[1]
            self.gantry_mode = GantryState.MOVE_TO_TARGET
            self.move_gantry_rel(0, 0, self.max_feed_rate)
            MazeDB.printMsg(
                'DEBUG', "[GantryOperation]: Move command received: target(%0.2f, %0.2f)", self.target_x, self.target_y)

        elif msg.cmd == "MOVE_TO_CHAMBER":
            chamber_num = int(msg.args[0])
            self.target_x = self.chamber_centers[chamber_num][0]
            self.target_y = self.chamber_centers[chamber_num][1]
            self.gantry_mode = GantryState.MOVE_TO_TARGET
            self.move_gantry_rel(0, 0, self.max_feed_rate)
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
