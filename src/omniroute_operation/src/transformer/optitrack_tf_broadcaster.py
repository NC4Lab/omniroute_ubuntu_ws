#!/usr/bin/env python
# Node that generates transform between optitrack and maze frames

import time
import rospy
import tf
import cv2

from omniroute_operation.msg import *
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np

class OptitrackTransformer:
    # @brief Initialize the GantryFeeder class
    def __init__(self):
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker0/pose', PointStamped, self.mazeboundary_marker0_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker1/pose', PointStamped, self.mazeboundary_marker1_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker2/pose', PointStamped, self.mazeboundary_marker2_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker3/pose', PointStamped, self.mazeboundary_marker3_callback, queue_size=1, tcp_nodelay=True)

        self.optitrack_marker0 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker1 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker2 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker3 = np.zeros(3, dtype=np.float32)

        self.maze_marker0 = np.array([0.9, 0.0, 0.0], dtype=np.float32)
        self.maze_marker1 = np.array([0.9, 0.9, 0.0], dtype=np.float32)
        self.maze_marker2 = np.array([0.0, 0.9, 0.0], dtype=np.float32)
        self.maze_marker3 = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        self.optitrack_br = tf.TransformBroadcaster()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()
        
    def loop(self):
        optitrack_tf = cv2.estimateAffine3D(np.concatenate((self.optitrack_marker0, self.optitrack_marker1, self.optitrack_marker2, self.optitrack_marker3), axis=0).reshape(4, 3),
                               np.concatenate((self.maze_marker0, self.maze_marker1, self.maze_marker2, self.maze_marker3), axis=0).reshape(4, 3),
                               ransacThreshold=0.1)
        
        rospy.loginfo(optitrack_tf)
        # self.optitrack_br.sendTransform(optitrack_tf[0][:, 3],
        #                                     tf.transformations.quaternion_from_matrix(optitrack_tf[0]),
        #                                     rospy.Time.now(),
        #                                     "optitrack",
        #                                     "maze")
                        
        
    def mazeboundary_marker0_callback(self, msg):
        self.optitrack_marker0  = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

    def mazeboundary_marker1_callback(self, msg):
        self.optitrack_marker1  = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

    def mazeboundary_marker2_callback(self, msg):
        self.optitrack_marker2  = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
    
    def mazeboundary_marker3_callback(self, msg):
        self.optitrack_marker3  = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
    
# @brief Main code
if __name__ == '__main__':
    rospy.init_node('optitrack_tf_broadcaster')
    OptitrackTransformer()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown