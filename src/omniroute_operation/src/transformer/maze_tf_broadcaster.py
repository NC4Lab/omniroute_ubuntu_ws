#!/usr/bin/env python
# Node that generates transforms between maze, gantry, and harness frames

import time
import rospy
import tf

from omniroute_operation.msg import *

    
# @brief Main code
if __name__ == '__main__':
    rospy.init_node('maze_tf_broadcaster')

    maze_br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0) # 10 Hz
    while not rospy.is_shutdown():
        maze_br.sendTransform((0.0, 0.0, 0.0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    "maze",
                    "world")
        rate.sleep()
