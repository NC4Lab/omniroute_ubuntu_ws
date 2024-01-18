#!/usr/bin/env python
# Node that generates transform between optitrack and maze frames

import time
import rospy
import tf

from omniroute_operation.msg import *
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np

class OptitrackTransformer:
    # @brief Initialize the OptitrackTransformer class
    def __init__(self):
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker0/pose', PointStamped, self.mazeboundary_marker0_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker1/pose', PointStamped, self.mazeboundary_marker1_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker2/pose', PointStamped, self.mazeboundary_marker2_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker3/pose', PointStamped, self.mazeboundary_marker3_callback, queue_size=1, tcp_nodelay=True)

        self.optitrack_marker0 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker1 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker2 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker3 = np.zeros(3, dtype=np.float32)

        self.maze_br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.transformer = tf.TransformerROS()

        self.maze_br.sendTransform([0,0,0], [0,0,0,1], rospy.Time.now(), "maze", "world")

        rospy.Subscriber('/natnet_ros/Harness/pose', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/Gantry/pose', PoseStamped, self.gantry_pose_callback, queue_size=1, tcp_nodelay=True)

        self.harness_pose_in_maze_pub = rospy.Publisher('/harness_pose_in_maze', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.gantry_pose_in_maze_pub = rospy.Publisher('/gantry_pose_in_maze', PoseStamped, queue_size=1, tcp_nodelay=True)

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()
        
    def loop(self):
        xhat = self.optitrack_marker1 - self.optitrack_marker0
        yhat = self.optitrack_marker2 - self.optitrack_marker0
        zhat = np.cross(xhat, yhat)

        xhat = xhat / np.linalg.norm(xhat)
        yhat = yhat / np.linalg.norm(yhat)
        zhat = zhat / np.linalg.norm(zhat)

        R = np.array([xhat, yhat, zhat])
        R = np.concatenate((R, np.zeros((3,1))), axis=1)
        R = np.concatenate((R, np.zeros((1,4))), axis=0)
        R[3,3] = 1.0

        t = self.optitrack_marker0

        self.maze_br.sendTransform(t, tf.transformations.quaternion_from_matrix(R), rospy.Time.now(), "maze", "world")

    
    def harness_pose_callback(self, msg):
        self.tf_listener.waitForTransform(target_frame="maze", source_frame="world", time=rospy.Time.now(), timeout=rospy.Duration(1.0))
        transformed_pose = self.tf_listener.transformPose(target_frame="maze", ps=msg)
        self.harness_pose_in_maze_pub.publish(transformed_pose)
    
    def gantry_pose_callback(self, msg):
        self.tf_listener.waitForTransform(target_frame="maze", source_frame="world", time=rospy.Time.now(), timeout=rospy.Duration(1.0))
        transformed_pose = self.tf_listener.transformPose(target_frame="maze", ps=msg)
        self.gantry_pose_in_maze_pub.publish(transformed_pose)


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
    rospy.init_node('maze_tf_broadcaster')
    OptitrackTransformer()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown