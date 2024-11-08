#!/usr/bin/env python

# Custom Imports
from shared_utils.maze_debug import MazeDB
from shared_utils.esmacat_com import EsmacatCom
from gantry.gcodeclient import Client as GcodeClient
from omniroute_operation.msg import *
from omniroute_esmacat_ros.msg import *

# Other Imports
import time
import rospy
from std_msgs.msg import *
import random


class SyncSender:
    def __init__(self):
        MazeDB.printMsg('ATTN', "SyncSender Node Started")

        # Create EsmacatCom object for sync_ease
        self.EsmaCom = EsmacatCom('sync_ease')

        # ROS sync command subscriber
        self.sync_sub = rospy.Subscriber(
            '/sync_cmd', SyncCmd, self.ros_callback_sync_cmd)
        
        # ROS event subscriber and publisher
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)

        # Specify sync signal parameters
        self.dt_sync_sig = 10.0  # Interval between sync signals (sec)
        self.width_sync_sig = 1.0  # Width of the sync signal (sec)

        # Initialize timing variables for handshake message handling
        self.dt_handshake_timeout = 1.0  # (sec)
        self.dt_handshake_callback = 0.01  # (sec)
        self.ts_handshake_sent = time.time()

        # Initialize flags for handshake message handling
        self.do_handshake = False
        self.is_handshake_sent = False
        self.is_handshake_confirmed = False

        # ROS timer for for SpikeGadgets sync signal callback
        self.timer_spikeGadgetsSync = rospy.Timer(rospy.Duration(self.dt_sync_sig),
                                                  self.timer_callback_spikeGadgetsSync)

        # ROS timer for for handshake handeling callback
        self.timer_handshakeHandling = rospy.Timer(rospy.Duration(self.dt_handshake_callback),
                                                   self.timer_callback_handshakeHandling)

    def ros_callback_sync_cmd(self, msg):
        """ Callback function for handeling ROS sync commands """

        # Check for start sync command
        if msg.cmd == 'initialize_sync_sender':

            # Set flag to start handshake
            self.do_handshake = True

        # Check for start optitrack sync event
        elif msg.cmd == 'start_optitrack_sync':

            # Send Ecat message to set sync pin high
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.SYNC_SET_OPTITRACK_PIN, 1)

            # Publish cmd to start optitrack recording
            self.event_pub.publish(
                "start_optitrack_recording", rospy.Time.now())

            MazeDB.printMsg('INFO', "Starting Optitrack Recording")

        # Check for stop optitrack sync cmd
        elif msg.cmd == 'stop_optitrack_sync':

            # Send Ecat message to set sync pin low
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.SYNC_SET_OPTITRACK_PIN, 0)

            # Publish event to stop optitrack recording
            self.event_pub.publish(
                "stop_optitrack_recording", rospy.Time.now())

            MazeDB.printMsg('INFO', "Stopping Optitrack Recording")

    def timer_callback_handshakeHandling(self, event):
        """ Callback function for handling handshake messages """

        # Wait for command to send handshake message
        if not self.do_handshake:
            return

        # Check if handshake message needs to be sent
        if not self.is_handshake_sent:

            # Send handshake message
            self.EsmaCom.writeEcatMessage(
                EsmacatCom.MessageType.HANDSHAKE)

            # Store time and set flag
            self.ts_handshake_sent = time.time()
            self.is_handshake_sent = True

        # Check for handshake message confirmation
        else:
            # Process incoming Ecat messages
            if self.EsmaCom.rcvEM.isNew:
                self.procEcatMessage()

        # Check if handshake is confirmed
        if self.is_handshake_confirmed:

            # Shutdown the timer
            self.timer_handshakeHandling.shutdown()

        # Check for handshake timeout
        elif time.time() - self.ts_handshake_sent >= self.dt_handshake_timeout:

            # Log error and kill the node
            MazeDB.printMsg(
                'ERROR', "Sync Arduino Handshake Failed: Shutting Down Node")
            rospy.signal_shutdown("Sync Arduino Handshake Failed")

    def timer_callback_spikeGadgetsSync(self, event):
        """ Callback function for sending sync signal to SpikeGadgets """

        # Wait for Ecat coms to be established
        if not self.EsmaCom.isEcatConnected:
            return

        # Add small random delay to the sync signal
        sleep_time = random.random()*(self.dt_sync_sig-self.width_sync_sig)
        rospy.sleep(sleep_time)

        # Publish current time to event topic
        self.event_pub.publish("sync_spikegadgets", rospy.Time.now())

        # Set the spike gadgets sync signal high
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.SYNC_SET_SPIKEGADGETS_PIN, 1, do_print=False)
        # MazeDB.printMsg('DEBUG', "SpikeGadgets sync signal sent")

        # Wait for the width of the sync signal
        rospy.sleep(self.width_sync_sig)

        # Set the spike gadgets sync signal low
        self.EsmaCom.writeEcatMessage(
            EsmacatCom.MessageType.SYNC_SET_SPIKEGADGETS_PIN, 0, do_print=False)

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
                'ATTN', "Sync Arduino Handshake Confirmed")
            
            # Publish to ROS event
            self.event_pub.publish("sync_ease_connected", rospy.Time.now())

            # Set the handshake flag
            self.is_handshake_confirmed = True

            # Set the Ecat connected flag
            self.EsmaCom.isEcatConnected = True

        # Reset new message flag
        self.EsmaCom.rcvEM.isNew = False


if __name__ == '__main__':
    # Initialize the ROS node with name 'sync_sender'
    rospy.init_node('sync_sender')
    SyncSender()  # Create an instance of the SyncSender class
    rospy.spin()  # Keep the node running until it is explicitly killed
