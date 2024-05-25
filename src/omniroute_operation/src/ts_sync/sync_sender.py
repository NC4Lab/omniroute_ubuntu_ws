#!/usr/bin/env python

# Custom Imports
from shared_utils.maze_debug import MazeDB
from shared_utils.esmacat_com import EsmacatCom
from gantry.gcodeclient import Client as GcodeClient
from omniroute_operation.msg import *
from omniroute_esmacat_ros.msg import *

# Other Imports
import rospy
from std_msgs.msg import *
import random

class SyncSender:
    def __init__(self):

        # Sync signal parameters
        self.interval = 10.0 # Interval between sync signals
        self.width = 1.0 # Width of the sync signal

        # ROS event publisher and subscriber
        self.event_pub = rospy.Publisher('/event', Event, queue_size=1)
        self.event_sub = rospy.Subscriber('/event', Event, self.event_callback)
        
        # ROS timer for for SpikeGadgets sync signal callback
        rospy.Timer(rospy.Duration(self.interval), self.timer_callback_spikeGadgetsSync)

        # Create EsmacatCom object for sync_ease
        self.EsmaComSync = EsmacatCom('sync_ease')

        MazeDB.printMsg('INFO', "[SyncSender]: Initialzed sync_sender_node")

    def event_callback(self, event):
        """ Callback function for modifying Optitrack sync pin through event subscriber """
        if event.event == 'start_optitrack_sync':
            self.EsmaComSync.writeEcatMessage(EsmacatCom.MessageType.SET_OPTITRACK_SYNC_PIN, 1)
            self.event_pub.publish("start_optitrack_recording", rospy.Time.now())
            MazeDB.printMsg('DEBUG', "[SyncSender]: Optitrack sync signal started")
        elif event.event == 'stop_optitrack_sync':
            self.EsmaComSync.writeEcatMessage(EsmacatCom.MessageType.SET_OPTITRACK_SYNC_PIN, 0)
            self.event_pub.publish("stop_optitrack_recording", rospy.Time.now())
            MazeDB.printMsg('DEBUG', "[SyncSender]: Optitrack sync signal stopped")

    def timer_callback_spikeGadgetsSync(self, event):
        """ Callback function for sending sync signal to SpikeGadgets """

        # Add small random delay to the sync signal
        sleep_time = random.random()*(self.interval-self.width)
        rospy.sleep(sleep_time)

        # Publish current time to event topic
        self.event_pub.publish("sync_spikegadgets", rospy.Time.now())

        # Set the spike gadgets sync signal high
        self.EsmaComSync.writeEcatMessage(EsmacatCom.MessageType.SET_SPIKEGADGETS_SYNC_PIN, 1)
        #MazeDB.printMsg('DEBUG', "[SyncSender]: SpikeGadgets sync signal sent")

        # Wait for the width of the sync signal
        rospy.sleep(self.width)

        # Set the spike gadgets sync signal low
        self.EsmaComSync.writeEcatMessage(EsmacatCom.MessageType.SET_SPIKEGADGETS_SYNC_PIN, 0)


if __name__ == '__main__':
    rospy.init_node('sync_sender') # Initialize the ROS node with name 'sync_sender'
    SyncSender() # Create an instance of the SyncSender class
    rospy.spin() # Keep the node running until it is explicitly killed