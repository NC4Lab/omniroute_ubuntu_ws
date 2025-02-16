#!/usr/bin/env python

# Custom Imports
from shared_utils.maze_debug import MazeDB
from shared_utils.esmacat_com import EsmacatCom
from shared_utils.wall_utilities import MazeDimensions

# ROS Imports
import rospy
from omniroute_operation.msg import *

# Other Imports
import time
from geometry_msgs.msg import PoseStamped
import numpy as np
from enum import Enum
import math


# Track the state for state machine
class GantryState(Enum):
    IDLE = 0
    INITIALIZE_GANTRY = 1
    TRACK_HARNESS = 2


class GantryOperation:
    # Initialize the GantryOperation class
    def __init__(self):
        MazeDB.printMsg('ATTN', "GantryOperation Node Started")

        # ................ GRBL Runtime Parameters ................
        self.max_feed_rate = 28500  # Maxiumum feed rate (mm/min)
        self.max_acceleration = 500  # Maximum acceleration (mm/sec^2)
        self.home_speed = 7500  # Homing speed (mm/min)

        # ................ Gantry Tracking Setup ................

        # Specity the proportionality constant for the gantry tracking
        self.Kp = 1.25

        # Specify the x and y offset from the gantry tracking marker to the gantry center (m)
        self.gantry_marker_to_gantry_center = np.array([-0.317, -0.185])

        # Track the loop time
        self.prev_time = time.time()

        # Last time G92 command was sent
        self.last_g92_time = time.time()

        # Initialize gantry coordinate class variables
        self.gantry_x = 0.0
        self.gantry_y = 0.0
        self.prev_gantry_x = 0.0
        self.prev_gantry_y = 0.0
        self.harness_x = 0.0
        self.harness_y = 0.0
        self.prev_harness_x = 0.0
        self.prev_harness_y = 0.0

        self.harness_vel = np.array([0.0, 0.0])
        self.gantry_vel = np.array([0.0, 0.0])

        self.distance_threshold = 0.15  # Distance threshold for stopping the gantry

        # Track if movement is in progress
        self.movement_in_progress = False

        self.maze_dim = MazeDimensions()

        # ................ ROS Setup ................

        # Initialize the subsrciber for reading in the gantry commands
        rospy.Subscriber('/gantry_cmd', GantryCmd,
                         self.ros_callback_gantry_cmd, queue_size=1, tcp_nodelay=True)

        # Initialize the subsrciber for reading in the tracker position data
        rospy.Subscriber('/harness_pose_in_maze', PoseStamped,
                         self.ros_callback_harness_pose, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/gantry_pose_in_maze', PoseStamped,
                         self.ros_callback_gantry_pose, queue_size=1, tcp_nodelay=True)
        
        # ROS event publisher
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)

        # Initialize the gantry mode used for state machine
        self.gantry_mode = GantryState.IDLE

        # ................ Ecat Setup ................

        # Create EsmacatCom object for gantry_ease
        self.EsmaCom = EsmacatCom('gantry_ease')

        # Initialize flags for gantry initialization
        self.is_handshake_sent = False
        self.is_handshake_confirmed = False
        self.is_grbl_initialize_confirmed = False
        self.is_gantry_home_confirmed = False

        # Initialize time tracking variables for initialization
        self.ts_handshake_sent = time.time()
        self.ts_grbl_initialize_sent = time.time()
        self.ts_gantry_home_sent = time.time()

        # Define timeouts for initialization steps (sec)
        self.dt_handshake_timeout = 1.0
        self.dt_grbl_initialize_timeout = 2.5
        self.dt_gantry_home_timeout = 20.0

        # ................ Run node ................

        # Initialize the ROS rate
        r = rospy.Rate(50)

        # Loop until the node is shutdown
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):

        # Store current time
        current_time = time.time()

        # ................ Handle Gantry Initialization ................

        if self.gantry_mode == GantryState.INITIALIZE_GANTRY:

            # Process incoming Ecat messages
            if self.EsmaCom.rcvEM.isNew:
                self.procEcatMessage()

            # Check if the handshake is sent
            if not self.is_handshake_sent:

                # Send handshake command
                self.EsmaCom.writeEcatMessage(EsmacatCom.MessageType.HANDSHAKE)

                # Update send time and flag
                self.ts_handshake_sent = time.time()
                self.is_handshake_sent = True

                return


            # Check for handshake timeout
            elif not self.is_handshake_confirmed:
                if time.time() - self.ts_handshake_sent >= self.dt_handshake_timeout:
                    
                    # Log error and kill the node
                    MazeDB.printMsg(
                        'ERROR', "Gantry Arduino Handshake Failed: Shutting Down Node")
                    rospy.signal_shutdown("Gantry Arduino Handshake Failed")
                
                return

            # Check for GRBL initialization timeout
            elif not self.is_grbl_initialize_confirmed:
                if time.time() - self.ts_grbl_initialize_sent >= self.dt_grbl_initialize_timeout:
                    
                    # Log error and kill the node
                    MazeDB.printMsg(
                        'ERROR', "Gantry GRBL Initialization Failed: Shutting Down Node")
                    rospy.signal_shutdown("Gantry GRBL Initialization Failed")
                
                return

            # Check for homing timeout
            elif not self.is_gantry_home_confirmed:
                if time.time() - self.ts_gantry_home_sent >= self.dt_gantry_home_timeout:
                
                    # Log error and kill the node
                    MazeDB.printMsg(
                        'ERROR', "Gantry Home Failed: Shutting Down Node")
                    rospy.signal_shutdown("Gantry Home Failed")
                
                return


            # Set the gantry mode to idle
            else:
                self.gantry_mode = GantryState.IDLE


        # ................ Handle GRBL Position Reset ................

        # if self.EsmaCom.isEcatConnected:

        #     # Check if the G92 command has not been sent in the last 2 seconds
        #     if current_time - self.last_g92_time >= 2.0:
                    
        #         # Convert gantry positions to mm
        #         gantry_x_mm = self.gantry_x * 1000.0
        #         gantry_y_mm = self.gantry_y * 1000.0

        #         # Send the reset origin command
        #         self.reset_origin(gantry_x_mm, gantry_y_mm)

        #         # Update the last G92 time
        #         self.last_g92_time = current_time

        # ................ Handle Haness Tracking ................

        if self.gantry_mode == GantryState.TRACK_HARNESS:

            # Compute the harness velocity
            self.harness_vel = np.array(
                [self.harness_x - self.prev_harness_x, self.harness_y - self.prev_harness_y])
            
            self.gantry_vel = np.array(
                [self.gantry_x - self.prev_gantry_x, self.gantry_y - self.prev_gantry_y])

            # Get the current distance between the gantry and the harness
            gantry_to_harness = np.array(
                [self.harness_x - self.gantry_x, self.harness_y - self.gantry_y])
            distance = np.linalg.norm(gantry_to_harness)

            # Calculate time step
            current_time = time.time()
            dt = current_time - self.prev_time
            dt = min(dt, 0.1)  # Limit dt to 0.1 sec

            if distance > self.distance_threshold:
                # Compute angle between harness velocity and gantry to harness vector
                # angle = math.acos(np.dot(self.harness_vel, gantry_to_harness) /
                #                   (np.linalg.norm(self.harness_vel) * np.linalg.norm(gantry_to_harness)))

                angle = math.acos(np.dot(self.gantry_vel, gantry_to_harness) /
                                  (np.linalg.norm(self.gantry_vel) * np.linalg.norm(gantry_to_harness)))

                # Print angle
                # MazeDB.printMsg('INFO', "Angle: %.2f", angle*180/math.pi)

                # Stop the gantry if it is moving in the wrong direction
                if angle > math.pi/4:
                    if self.movement_in_progress:
                        self.jog_cancel()
                        self.movement_in_progress = False

                # Compute jog distance
                jog_distance = self.compute_jog(
                    gantry_to_harness, self.max_feed_rate, dt, self.Kp)

                # Send the movement command
                self.move_gantry_rel(
                    jog_distance[0], jog_distance[1], self.max_feed_rate)

            # Stop the gantry when it reaches the harness
            elif self.movement_in_progress:
                self.jog_cancel()
                self.movement_in_progress = False

            self.prev_harness_x = self.harness_x
            self.prev_harness_y = self.harness_y

            self.prev_gantry_x = self.gantry_x
            self.prev_gantry_y = self.gantry_y

            # Update the time
            self.prev_time = current_time

    def compute_jog(self, gantry_to_target, max_feed_rate, dt_sec, Kp):
        """
        Compute jog increments for the gantry based on position error, ensuring
        movement is along the direct vector to the target without exceeding max feed rate.

        Arguments:
        - gantry_to_target: numpy array [dx, dy] in mm.
        - max_feed_rate: scalar max feed rate in mm/min.
        - dt_sec: time interval between control loop iterations in seconds.
        - Kp: Proportional gain.

        Returns:
        - jog_distance: numpy array [jog_x, jog_y] in mm.
        """

        # Convert gantry_to_target from m to mm (if it's not already in mm)
        gantry_to_target_mm = gantry_to_target * 1000.0  # mm

        # Compute desired velocity vector (mm/sec)
        desired_velocity = Kp * gantry_to_target_mm  # mm/sec

        # Compute the magnitude of the desired velocity vector
        velocity_magnitude = np.linalg.norm(desired_velocity)

        # Convert max feed rate to mm/sec
        max_feed_rate_mm_per_sec = max_feed_rate / 60.0  # mm/sec

        # Determine the maximum allowable velocity (feed rate limit)
        max_velocity = max_feed_rate_mm_per_sec

        # If the desired velocity magnitude exceeds the maximum, scale the velocity vector
        if velocity_magnitude > max_velocity:
            scaling_factor = max_velocity / velocity_magnitude
            desired_velocity = desired_velocity * scaling_factor

        # Compute jog distance (mm)
        jog_distance = desired_velocity * dt_sec

        return jog_distance

    def jog_cancel(self):
        """
        Cancel the current jog movement.
        """

        # Send command to cancel jog
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_JOG_CANCEL, do_print=False)

    def home(self, home_speed):

        # Send command to home gantry
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_HOME, msg_arg_data_i16=[home_speed])

    def move_gantry_rel(self, x, y, feed_rate=-1):
        """
        Move the gantry to a relative position.

        Arguments:
        - x: relative x position in mm.
        - y: relative y position in mm.
        - feed_rate: max feed rate in mm/min.
        """

        if feed_rate == -1:
            feed_rate = self.max_feed_rate

        if self.is_handshake_confirmed == False or self.is_grbl_initialize_confirmed == False or self.is_gantry_home_confirmed == False:
            MazeDB.printMsg(
                'WARN', "Gantry not initialized. Cannot move gantry.")
            return

        # Flip x and y to account for gantry orientation and store to a list
        xy_list = [y, x, feed_rate]
        xy_list = [int(i) for i in xy_list]

        # Print the move command
        MazeDB.printMsg(
            'DEBUG', "Move gantry rel: x[%0.2f] y[%0.2f] at rate %d", xy_list[0], xy_list[1], xy_list[2])

        # Send command to move gantry
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_MOVE_REL, msg_arg_data_i16=xy_list, do_print=False)

        # Set flag that movement is in progress
        if x != 0 or y != 0:
            self.movement_in_progress = True
    
    def move_gantry_abs(self, x, y, feed_rate=-1):
        """
        Move the gantry to an absolute position.

        Arguments:
        - x: absolute x position in mm.
        - y: absolute y position in mm.
        - feed_rate: max feed rate in mm/min.
        """

        if feed_rate == -1:
            feed_rate = self.max_feed_rate

        if self.is_handshake_confirmed == False or self.is_grbl_initialize_confirmed == False or self.is_gantry_home_confirmed == False:
            MazeDB.printMsg(
                'WARN', "Gantry not initialized. Cannot move gantry.")
            return

        # Flip x and y to account for gantry orientation and store to a list
        xy_list = [y, x, feed_rate]
        xy_list = [int(i) for i in xy_list]

        # Print the move command
        MazeDB.printMsg(
            'DEBUG', "Move gantry abs: x[%0.2f] y[%0.2f] at rate %d", xy_list[0], xy_list[1], xy_list[2])

        # Send command to move gantry
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.GANTRY_MOVE_ABS, msg_arg_data_i16=xy_list, do_print=False)

        # Set flag that movement is in progress
        if x != 0 or y != 0:
            self.movement_in_progress = True

    def reset_origin(self, x, y):
        """
        Reset the origin of the gantry.

        Arguments:
        - x: new x position in mm.
        - y: new y position in mm.
        """

        # Flip x and y to account for gantry orientation and store to a list
        xy_list = [y, x]

        # Print the move command
        MazeDB.printMsg(
            'DEBUG', "Reset gantry origin: x[%0.2f] y[%0.2f]", xy_list[0], xy_list[1])

        # Send command to reset origin
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.RESET_ORIGIN, msg_arg_data_f32=xy_list, do_print=False)

    def ros_callback_gantry_pose(self, msg):
        # Store x
        self.gantry_x = msg.pose.position.x + \
            self.gantry_marker_to_gantry_center[0]
        # Store y
        self.gantry_y = msg.pose.position.y + \
            self.gantry_marker_to_gantry_center[1]

    def ros_callback_harness_pose(self, msg):
        # Store x
        self.harness_x = msg.pose.position.x
        # Store y
        self.harness_y = msg.pose.position.y

    def ros_callback_gantry_cmd(self, msg):
        """
        Callback function for handling ROS gantry commands from GantryCmd message.
        """

        if msg.cmd == "initialize_gantry":
            MazeDB.printMsg(
                'DEBUG', "Initialize System command received")
            self.gantry_mode = GantryState.INITIALIZE_GANTRY

        elif msg.cmd == "home_gantry":
            MazeDB.printMsg(
                'DEBUG', "Homing command received")
            self.home(self.home_speed)

        elif msg.cmd == "move_to_coordinate":

            # Get the target x and y
            target_x = msg.args[0]
            target_y = msg.args[1]
            target_x_mm = target_x * 1000.0  # convert to mm
            target_y_mm = target_y * 1000.0  # convert to mm

            # Move the gantry to the target
            self.move_gantry_rel(
                target_x_mm, target_y_mm, self.max_feed_rate)

            # Set back to idle
            self.gantry_mode = GantryState.IDLE

            MazeDB.printMsg(
                'DEBUG', "Move to coordinate command received: target[%0.2fm, %0.2fm]", target_x, target_y)

        elif msg.cmd == "move_to_chamber":
            self.jog_cancel()

            # Get the target x and y
            chamber_num = int(msg.args[0])
            target_x = self.maze_dim.chamber_centers[chamber_num][0] * 1000.0 + 225
            target_y = self.maze_dim.chamber_centers[chamber_num][1] * 1000.0

            # Send the move command
            self.move_gantry_abs(
                target_x, target_y, 15000)

            # Set back to idle
            self.gantry_mode = GantryState.IDLE

            MazeDB.printMsg('INFO', "Move to chamber command received: chamber[%d] target[%0.2fm, %0.2fm]",
                            chamber_num, target_x, target_y)

        elif msg.cmd == "start_harness_tracking":
            MazeDB.printMsg(
                'DEBUG', "Start tracking harness command received")
            self.gantry_mode = GantryState.TRACK_HARNESS

        elif msg.cmd == "stop_harness_tracking":
            MazeDB.printMsg(
                'DEBUG', "Stop tracking harness command received")
            self.gantry_mode = GantryState.IDLE

        elif msg.cmd == "lower_feeder":
            MazeDB.printMsg(
                'DEBUG', "Lower Feeder command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_SET_FEEDER, 1)

        elif msg.cmd == "raise_feeder":
            MazeDB.printMsg(
                'DEBUG', "Raise Feeder command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_SET_FEEDER, 0)

        elif msg.cmd == "start_pump":
            MazeDB.printMsg(
                'DEBUG', "Start Pump command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_RUN_PUMP, 1)

        elif msg.cmd == "stop_pump":
            MazeDB.printMsg(
                'DEBUG', "Stop Pump command received")
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_RUN_PUMP, 0)

        elif msg.cmd == "deliver_reward":
            duration = msg.args[0]  # Duration in seconds
            MazeDB.printMsg(
                'DEBUG', "Deliver reward command received: duration(%d)", duration)
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_REWARD, msg_arg_data_f32=duration)

    def procEcatMessage(self):
        """ 
        Used to parse new incoming ROS ethercat msg data. 
        """

        # ................ Process Ack Error First ................

        if self.EsmaCom.rcvEM.errTp != EsmacatCom.ErrorType.ERR_NONE:

            MazeDB.printMsg('ERROR', "Ecat Message [id=%d]: %s",
                            self.EsmaCom.rcvEM.msgID, self.EsmaCom.rcvEM.errTp.name)

        # ................ Process Ack Message ................

        # HANDSHAKE
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.HANDSHAKE:
            MazeDB.printMsg(
                'ATTN', "Gantry Arduino Handshake Confirmed")
            
            # Publish to ROS event
            self.event_pub.publish("gantry_ease_connected", rospy.Time.now())

            # Set the handshake flag
            self.is_handshake_confirmed = True

            # Set the Ecat connected flag
            self.EsmaCom.isEcatConnected = True

            # Send command to initialize GRBL for gantry
            init_list = [self.max_feed_rate, self.max_acceleration]
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_INITIALIZE_GRBL, msg_arg_data_f32=init_list)

            # Store the send time
            self.ts_grbl_initialize_sent = time.time()

        # GANTRY_INITIALIZE_GRBL
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.GANTRY_INITIALIZE_GRBL:
            MazeDB.printMsg('ATTN', "Gantry GRBL Initialization Confirmed")

            # Set the GRBL initialization flag
            self.is_grbl_initialize_confirmed = True

            # Send command to home the gantry
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.GANTRY_HOME, msg_arg_data_i16=[self.home_speed])

            # Store the send time
            self.ts_gantry_home_sent = time.time()

        # GANTRY_HOME
        if self.EsmaCom.rcvEM.msgTp == EsmacatCom.MessageType.GANTRY_HOME:
            MazeDB.printMsg('ATTN', "Gantry Homing Confirmed")

            # Set the gantry home flag
            self.is_gantry_home_confirmed = True

            # Store the send time
            self.ts_gantry_home_sent = time.time()

        # Reset new message flag
        self.EsmaCom.rcvEM.isNew = False


# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'gantry_operation'
    rospy.init_node('gantry_operation')
    GantryOperation()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown
