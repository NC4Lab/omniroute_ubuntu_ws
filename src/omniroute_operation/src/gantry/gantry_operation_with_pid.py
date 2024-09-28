#!/usr/bin/env python

# Custom Imports
from shared_utils.maze_debug import MazeDB
from shared_utils.esmacat_com import EsmacatCom
from gantry.gcodeclient import Client as GcodeClient

# ROS Imports
import rospy
from omniroute_operation.msg import *

# Other Imports
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

        # ................ Serial Setup ................
        
        # Flag if using serial communication
        self.use_serial = False

        # Setup serial communication
        if self.use_serial:

            # GCode client setup
            self.gcode_client = GcodeClient('/dev/ttyUSB0', 115200)
            rospy.sleep(1)

            # Set Units (mm)
            self.gcode_client.raw_command("G21")

            # Set Mode (G90 = Absolute, G91 = Relative)
            self.gcode_client.raw_command("G91")

            # Feed Rate (mm/min)
            self.gcode_client.raw_command("F30000")

        # ................ Gantry Tracking Setup ................

        # Flag to run auto-tuning
        self.track_method = "prop"  # Choose between "prop", "pid", or 'tune'

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
        self.prev_time = rospy.get_time().to_sec()

        # Relay feedback parameters
        self.amplitude = 20  # This can be adjusted based on system needs
        self.err_threshold = 0.01  # Threshold for relay feedback
        self.min_oscillations = 5  # Minimum occillations to test

        # Relay feedback for Ziegler-Nichols (separate for X and Y axes)
        self.relay_output_x = self.amplitude
        self.relay_output_y = self.amplitude
        self.last_error_sign_x = 1
        self.last_error_sign_y = 1
        self.oscillations_x = []
        self.oscillations_y = []
        self.last_cross_time_x = None
        self.last_cross_time_y = None

        # PID auto-tuned parameters (separate for X and Y axes)
        self.Ku_x = None
        self.Tu_x = None
        self.Ku_y = None
        self.Tu_y = None

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
        rospy.sleep(1)

        # Send handshake command
        self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.HANDSHAKE)

        # Wait for 1 second
        rospy.sleep(1)

        # Send command to initialize GRBL for gantry
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_INITIALIZE_GRBL)

        # # TEMP
        # rospy.sleep(1)
        # self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.GANTRY_HOME)
        # rospy.sleep(5)

        # ................ Run node ................

        # Initialize the ROS rate
        r = rospy.Rate(30)
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

            # #TEMP - Print the distance and the gantry position for debugging
            # MazeDB.printMsg('INFO', "X[%0.2f] Y[%0.2f] D[%0.2f]", gantry_to_harness[0], gantry_to_harness[1], distance)

            # Run PID auto-tuning
            if self.track_method == 'tune':

                if distance > 0.05:

                    # Relay feedback for X axis
                    self.relay_output_x, self.last_error_sign_x, self.last_cross_time_x, self.oscillations_x = self.relay_feedback_control(
                        gantry_to_harness[0],
                        self.relay_output_x,
                        self.last_error_sign_x,
                        self.last_cross_time_x,
                        self.amplitude,
                        self.oscillations_x,
                        self.err_threshold,
                        axis_name="x"
                    )

                    # Relay feedback for Y axis
                    self.relay_output_y, self.last_error_sign_y, self.last_cross_time_y, self.oscillations_y = self.relay_feedback_control(
                        gantry_to_harness[1],
                        self.relay_output_y,
                        self.last_error_sign_y,
                        self.last_cross_time_y,
                        self.amplitude,
                        self.oscillations_y,
                        self.err_threshold,
                        axis_name="y"
                    )

                    #  # TEMP - Print the relay feedback for debugging
                    # MazeDB.printMsg(
                    #     'INFO', "Relay feedback: x[%0.2f] y[%0.2f]", self.relay_output_x, self.relay_output_y)
                    
                    # Send the movement command
                    self.move_gantry_rel(
                        self.relay_output_x, self.relay_output_y)

                    self.movement_in_progress = True

                    # Calculate and apply PID tuning for X axis once enough oscillations have occurred
                    if len(self.oscillations_x) >= self.min_oscillations:
                        self.calculate_pid_parameters(self.oscillations_x, axis_name="x")

                    # Calculate and apply PID tuning for Y axis once enough oscillations have occurred
                    if len(self.oscillations_y) >= self.min_oscillations:
                        self.calculate_pid_parameters(self.oscillations_y, axis_name="y")

                # Stop the gantry when it reaches the harness
                elif self.movement_in_progress:
                    self.jog_cancel()
                    self.movement_in_progress = False

            # Use PID values (tuned or hardcoded) for control
            elif self.track_method == 'pid':
                if distance > 0.01:

                    # Calculate time step
                    current_time = rospy.get_time().to_sec()
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
                        self.move_gantry_rel(x, y)
                        self.movement_in_progress = True

                    # Update the time
                    self.prev_time = current_time

                # Stop the gantry when it reaches the harness
                elif self.movement_in_progress:
                    self.jog_cancel()
                    self.movement_in_progress = False

            # Use proportional control
            elif self.track_method == 'prop':
                if distance > 0.075:
                    x, y = self.proportional_control(
                        gantry_to_harness, base_speed=25.0, slow_on_approach=False)
                    
                    # x, y = self.compute_jog(gantry_to_harness)
                    if x==0 and y==0 and self.movement_in_progress:
                        self.jog_cancel()
                        self.movement_in_progress = False

                    # Send the movement command
                    elif ~np.isnan(x) and ~np.isnan(y):
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

            if x==0 and y==0 and self.movement_in_progress:
                self.jog_cancel()
                self.movement_in_progress = False

            # Move the gantry to the target
            elif ~np.isnan(x) and ~np.isnan(y):
                self.move_gantry_rel(x, y)
                self.movement_in_progress = True

    def compute_jog(self, gantry_to_setpoint, delta_t=0.02, max_feed_rate=30000):
        """
        Compute relative jog values based on the distance to the target (error).
        gantry_to_setpoint: tuple (x_error, y_error), distance from target position
        delta_t: sampling time interval in seconds (50 Hz = 0.02 s)
        max_feed_rate: max feed rate in mm/min
        Returns: relative (x_jog, y_jog) distances to jog
        """
        # The feed rate is in mm/min, so convert to mm/s
        feed_rate_mm_per_s = max_feed_rate / 60.0

        # Determine how far we can move in delta_t seconds
        max_move_per_sample = feed_rate_mm_per_s * delta_t

        # Compute the magnitude of the current error vector (distance to target)
        distance_to_target = (gantry_to_setpoint[0]**2 + gantry_to_setpoint[1]**2) ** 0.5

        if distance_to_target > max_move_per_sample:
            # Scale the jog values to fit within the maximum move distance
            scale = max_move_per_sample / distance_to_target
            x_jog = gantry_to_setpoint[0] * scale
            y_jog = gantry_to_setpoint[1] * scale
        else:
            # If the error is small, jog directly to reduce it to zero
            x_jog = gantry_to_setpoint[0]
            y_jog = gantry_to_setpoint[1]

        return x_jog, y_jog
    
    def relay_feedback_control(self, error, relay_output, last_error_sign, last_cross_time, amplitude, oscillations, err_threshold, axis_name):
        """
        Relay feedback control to induce oscillations for a given axis (X or Y).

        Args:
        - error: The current error in the selected axis.
        - relay_output: The current relay output for the selected axis.
        - last_error_sign: Last sign of the error in the selected axis.
        - last_cross_time: Time of the last crossing in the selected axis.
        - amplitude: Amplitude of the relay signal.
        - oscillations: List to track oscillation periods and amplitudes for the selected axis.
        - err_threshold: Threshold to determine if the error is significant.
        - axis_name: String to identify the axis ('x' or 'y') for debugging purposes.

        Returns:
        - relay_output: Updated relay output for the selected axis.
        - last_error_sign: Updated last error sign for the selected axis.
        - last_cross_time: Updated last cross time for the selected axis.
        - oscillations: Updated oscillation tracking for the selected axis.
        """

        # Determine the current sign of the error
        current_sign = np.sign(error)
        current_time = rospy.get_time().to_sec()

        # #TEMP print everything
        # MazeDB.printMsg('INFO', "[GantryOperation] Axis[%s] error[%0.2f] current_sign[%0.1f] last_error_sign[%0.1f] last_cross_time[%s] current_time[%0.2f]", 
        #                 axis_name, error, current_sign, last_error_sign, 
        #                 str(last_cross_time) if last_cross_time is not None else "None", current_time)


        # Check if the error has crossed zero (sign change)
        if current_sign != last_error_sign and abs(error) > err_threshold:
            # Switch the relay relay_output based on error sign
            relay_output = amplitude if error > 0 else -amplitude

            # If this is not the first crossing, record the oscillation period
            if last_cross_time:
                oscillation_period = current_time - last_cross_time
                oscillations.append((oscillation_period, abs(relay_output)))

            # Update the time of the last crossing
            last_cross_time = current_time

            # For debugging, you can add print statements to monitor relay feedback per axis
            MazeDB.printMsg(
                'INFO', "[GantryOperation] Axis crossing detected: axis[%s] relay_output[%0.2f]", axis_name, relay_output)

            # Update the last error sign for the next iteration
            last_error_sign = current_sign

        return relay_output, last_error_sign, last_cross_time, oscillations

    def calculate_pid_parameters(self, oscillations, axis_name):
        """
        Calculate PID parameters using Ziegler-Nichols tuning rules.
        This is done separately for each axis.
        """
        if len(oscillations) < 2:
            return None, None, None  # Wait until at least 2 oscillations

        # Calculate ultimate period (Tu) and ultimate gain (Ku) from oscillations
        periods, amplitudes = zip(*oscillations[-2:])
        Tu = np.mean(periods)
        Ku = 4 * self.amplitude / (np.pi * Tu)  # From Ziegler-Nichols method

        # Ziegler-Nichols tuning rules for PID
        Kp = 0.6 * Ku
        Ki = 2 * Kp / Tu
        Kd = Kp * Tu / 8

        # Log the PID parameters for debugging
        MazeDB.printMsg(
            'INFO', "[GantryOperation] PID tuning updated: axis[%s] Kp=%0.2f, Ki=%0.2f, Kd=%0.2f", axis_name, Kp, Ki, Kd)

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

    def proportional_control(self, gantry_to_setpoint, base_speed, slow_on_approach=True):
        """
        Proportional control to calculate movement based on the current distance and direction to the target.

        Args:
        - gantry_to_setpoint: Array containing the difference between the gantry and target positions in X and Y.
        - base_speed: The base speed to scale the movement by.
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
        output_x = base_speed * gantry_to_setpoint[0]
        # X component of the target movement vector
        output_y = base_speed * gantry_to_setpoint[1]

        # Cap output to 10 
        # max_move = 8
        # output_x = min(max(output_x, -max_move), max_move)
        # output_y = min(max(output_y, -max_move), max_move)


        # Move at least jog_dist
        jog_dist = 1.0
        if abs(output_x) < jog_dist:
            output_x = 0
        else:
            output_x = jog_dist * np.sign(output_x)
        
        if abs(output_y) < jog_dist:
            output_y = 0
        else:
            output_y = jog_dist * np.sign(output_y)

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
            self.gcode_client.raw_command("$25=7500") # Set homing speed
            self.gcode_client.raw_command("$H") # Home the gantry
            self.gcode_client.raw_command("G10 P0 L20 X0 Y0 Z0") # Set the current position to 0

        # Send command to home gantry
        else:
            self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.GANTRY_HOME)

    def move_gantry_rel(self, x, y):

        # Flip x and y to account for gantry orientation and store to a list
        xy_list = [y, x]

        # Use serial for move command
        if self.use_serial:
            # Move the gantry
            cmd = "$J=G91 G21 X{:.1f} Y{:.1f} F25000".format(xy_list[0], xy_list[1])
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
