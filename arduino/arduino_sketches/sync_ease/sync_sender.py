#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from esmacat_ros.msg import *
import random

class SyncSender:
    def __init__(self):
        self.interval = 10.0
        self.width = 1.0
        self.write_sync_ease_pub = rospy.Publisher('/Esmacat_write_sync_ease', ease_registers, queue_size=1)
        self.sync_pub = rospy.Publisher('sync',Time,queue_size=100)
        
        rospy.Timer(rospy.Duration(self.interval), self.timer_callback)

    def timer_callback(self, event):
        sleep_time = random.random()*(self.interval-self.width)
        rospy.sleep(sleep_time)

        reg = [0]*8
        reg[0] = 1

        time = rospy.Time.now()
        self.sync_pub.publish(time)
        self.write_sync_ease_pub.publish(*reg)

        rospy.sleep(self.width)
        
        reg[0] = 0
        self.write_sync_ease_pub.publish(*reg)


if __name__ == '__main__':
    rospy.init_node('sync_sender')
    SyncSender()
    rospy.spin()