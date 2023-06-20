#!/usr/bin/env python

import rospy
from nc4_esmacat_ros.msg import *

class WallController:
    def __init__(self):
        self.foo = 1

        self.maze_ard0_pub = rospy.Publisher('/Esmacat_write_maze_ard0_ease',ease_registers,queue_size=1)
    
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):
        reg = [0]*8
        reg[0] = 2
        self.maze_ard0_pub.publish(*reg)

        pass

if __name__ == '__main__':
    rospy.init_node('wall_controller')
    WallController()
    rospy.spin()