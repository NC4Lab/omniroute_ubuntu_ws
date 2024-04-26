#!/usr/bin/env python
# Node that generates transform between optitrack and maze frames

import time
import rospy
import tf

from omniroute_operation.msg import *
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np

class MazeTransformer:
    # Initialize the OptitrackTransformer class
    def __init__(self):
        rospy.loginfo("[MazeTransformer]: INITAILIZING...")

        # Initialize the subscribers for reading in the optitrack data
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker0/pose', PointStamped, self.mazeboundary_marker0_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker1/pose', PointStamped, self.mazeboundary_marker1_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker2/pose', PointStamped, self.mazeboundary_marker2_callback, queue_size=1, tcp_nodelay=True)

        self.optitrack_marker0 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker1 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker2 = np.zeros(3, dtype=np.float32)

        self.maze_br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.maze_t = np.array([0,0,0], dtype=np.float32)
        self.maze_R = np.eye(4, dtype=np.float32)

        rospy.Subscriber('/natnet_ros/Harness/pose', PoseStamped, self.harness_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/Gantry/pose', PoseStamped, self.gantry_pose_callback, queue_size=1, tcp_nodelay=True)

        self.harness_pose_in_maze_pub = rospy.Publisher('/harness_pose_in_maze', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.gantry_pose_in_maze_pub = rospy.Publisher('/gantry_pose_in_maze', PoseStamped, queue_size=1, tcp_nodelay=True)

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()
        
    def loop(self):
        # Compute vectors from marker0 to marker1 and marker0 to marker2
        xhat = self.optitrack_marker1 - self.optitrack_marker0
        yhat = self.optitrack_marker2 - self.optitrack_marker0

        # Normalize the vectors to unit length to form the first two axes of the coordinate frame
        # Check if normalization is possible (vector length should not be zero)
        if np.linalg.norm(xhat) > 0:
            xhat = xhat / np.linalg.norm(xhat)
        else:
            #rospy.logwarn("Zero-length vector for xhat, skipping loop iteration.")
            return

        if np.linalg.norm(yhat) > 0:
            yhat = yhat / np.linalg.norm(yhat)
        else:
            #rospy.logwarn("Zero-length vector for yhat, skipping loop iteration.")
            return

        # Compute the third axis using the cross product to ensure orthogonality
        zhat = np.cross(xhat, yhat)

        # Construct the rotation matrix using the three orthogonal axes
        R = np.array([xhat, yhat, zhat]).transpose()
        self.maze_R = R  # Store the rotation matrix globally if needed elsewhere
        
        # Pad the rotation matrix R to a 4x4 matrix to include translation
        R = np.concatenate((R, np.zeros((3,1))), axis=1)  # Add zero column for translation
        R = np.concatenate((R, np.zeros((1,4))), axis=0)  # Add zero row for homogeneous coordinates
        R[3,3] = 1.0  # Set the bottom right element to 1 for valid homogeneous transformation matrix

        # Use the position of marker0 as the translation part of the transform
        t = self.optitrack_marker0
        self.maze_t = t  # Store the translation globally if needed elsewhere

        # Broadcast the transformation from 'world' frame to 'maze' frame
        # Quaternion is derived from the 4x4 transformation matrix R
        self.maze_br.sendTransform(t, tf.transformations.quaternion_from_matrix(R), rospy.Time.now(), "maze", "world")

    def harness_pose_callback(self, msg):    
        try:
            # Get the latest time for which the TF listener has the transform
            latest_time = self.tf_listener.getLatestCommonTime("maze", msg.header.frame_id)

            # Set the timestamp of the message to this latest available time
            msg.header.stamp = latest_time

            # Perform the transformation
            transformed_pose = self.tf_listener.transformPose("maze", msg)
            self.harness_pose_in_maze_pub.publish(transformed_pose)

        except (tf.ExtrapolationException, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn("TF exception in harness_pose_callback: {}".format(e))

            
    def gantry_pose_callback(self, msg):
        try:
            # Adjust the timestamp slightly to the past to ensure the transform is available
            msg.header.stamp = rospy.Time.now() - rospy.Duration(0.01)

            # Wait for the transform to be available
            self.tf_listener.waitForTransform(target_frame="maze", source_frame=msg.header.frame_id, 
                                            time=msg.header.stamp, timeout=rospy.Duration(1.0))

            # Perform the transformation
            transformed_pose = self.tf_listener.transformPose(target_frame="maze", ps=msg)
            self.gantry_pose_in_maze_pub.publish(transformed_pose)

        except (tf.ExtrapolationException, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn("TF exception in gantry_pose_callback: {}".format(e))

    def mazeboundary_marker0_callback(self, msg):
        self.optitrack_marker0  = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

    def mazeboundary_marker1_callback(self, msg):
        self.optitrack_marker1  = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

    def mazeboundary_marker2_callback(self, msg):
        self.optitrack_marker2  = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
    
# @brief Main code
if __name__ == '__main__':
    rospy.init_node('maze_tf_broadcaster')
    MazeTransformer()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown