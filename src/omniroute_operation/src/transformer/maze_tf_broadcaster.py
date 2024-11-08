#!/usr/bin/env python
# Node that generates transform between optitrack and maze frames

# Custom Imports
from shared_utils.maze_debug import MazeDB

# ROS Imports
import rospy
from omniroute_operation.msg import *

# Other Imports
import time
import tf
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np

class MazeTransformer:
    # Initialize the OptitrackTransformer class
    def __init__(self):
        MazeDB.printMsg('ATTN', "MazeTransformer Node Started")

        # Publishers to output transformed poses of the harness and gantry within the maze
        self.harness_pose_in_maze_pub = rospy.Publisher(
            '/harness_pose_in_maze', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.gantry_pose_in_maze_pub = rospy.Publisher(
            '/gantry_pose_in_maze', PoseStamped, queue_size=1, tcp_nodelay=True)

        # Subscribers to obtain pose data for three boundary markers of the maze
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker0/pose', PointStamped,
                         self.mazeboundary_marker0_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker1/pose', PointStamped,
                         self.mazeboundary_marker1_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/MazeBoundary/marker2/pose', PointStamped,
                         self.mazeboundary_marker2_callback, queue_size=1, tcp_nodelay=True)

        # Initialize arrays to store the 3D coordinates of the maze boundary markers
        self.optitrack_marker0 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker1 = np.zeros(3, dtype=np.float32)
        self.optitrack_marker2 = np.zeros(3, dtype=np.float32)

        # Track if maze transform has been published
        self.is_maze_tranform_published = False

        # Track the broadcasting status of Optitrack
        self.is_marker0_broadcasting = False
        self.is_marker1_broadcasting = False
        self.is_marker2_broadcasting = False
        self.is_harness_pose_broadcasting = False
        self.is_gantry_pose_broadcasting = False

        # Timeout duration for Optitrack broadcasting (sec)
        self.dt_optitrack_timeout = 5.0
        self.ts_node_init = time.time()

        # Transform broadcaster and listener for handling transformations
        self.maze_br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # Initialize transformation matrices for translation and rotation
        self.maze_t = np.array([0, 0, 0], dtype=np.float32)
        self.maze_R = np.eye(4, dtype=np.float32)

        # Subscribers for additional pose data regarding harness and gantry
        rospy.Subscriber('/natnet_ros/Harness/pose', PoseStamped,
                         self.ros_callback_harness_pose, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/natnet_ros/Gantry/pose', PoseStamped,
                         self.ros_callback_gantry_pose, queue_size=1, tcp_nodelay=True)

        # Control loop running at 100 Hz to handle data processing and transformation broadcasting
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):

        # Check if markers are broadcasting
        if (
            self.is_marker0_broadcasting and
            self.is_marker1_broadcasting and
            self.is_marker2_broadcasting
        ):
    
            # Compute and pulish the maze to world transform
            self.compute_and_publish_maze_transform()

            # Set the broadcasting flag for the maze transform
            self.is_maze_tranform_published = True

        # Check that all assets are broadcasting  
        if not (
            self.is_marker0_broadcasting and
            self.is_marker1_broadcasting and
            self.is_marker2_broadcasting and
            self.is_harness_pose_broadcasting and
            self.is_gantry_pose_broadcasting
        ):

            # Check for timeout on Optitrack broadcasting
            if time.time() - self.ts_node_init > self.dt_optitrack_timeout:
                MazeDB.printMsg(
                    'ERROR', "Optitrack Markers Not Broadcasting: Shutting Down Node")
                rospy.signal_shutdown("Optitrack Markers Not Broadcasting")
        
    def compute_and_publish_maze_transform(self):

        # Compute vectors from marker0 to marker1 and marker0 to marker2 for the maze boundary
        xhat = self.optitrack_marker1 - self.optitrack_marker0
        yhat = self.optitrack_marker2 - self.optitrack_marker0

        # Normalize the vectors to unit length to form the first two axes of the coordinate frame
        xhat = xhat / np.linalg.norm(xhat)
        yhat = yhat / np.linalg.norm(yhat)

        # Compute the third axis using the cross product to ensure orthogonality
        zhat = np.cross(xhat, yhat)

        # Construct the rotation matrix using the three orthogonal axes
        R = np.array([xhat, yhat, zhat]).transpose()
        self.maze_R = R

        # Pad the rotation matrix R to a 4x4 matrix to include translation
        # Add zero column for translation
        R = np.concatenate((R, np.zeros((3, 1))), axis=1)
        # Add zero row for homogeneous coordinates
        R = np.concatenate((R, np.zeros((1, 4))), axis=0)
        # Set the bottom right element to 1 for valid homogeneous transformation matrix
        R[3, 3] = 1.0

        # Use the position of marker0 as the translation part of the transform
        t = self.optitrack_marker0
        self.maze_t = t  # Store the translation globally if needed elsewhere

        # Broadcast the transformation from 'world' frame to 'maze' frame
        # Quaternion is derived from the 4x4 transformation matrix R
        self.maze_br.sendTransform(t, tf.transformations.quaternion_from_matrix(
            R), rospy.Time.now(), "maze", "world")

        # Add a delay to make sure the transform is available
        rospy.sleep(0.1)

    def transform_and_publish_pose(self, msg, publisher):
        
        # Wait for the transform to be available
        if not self.is_maze_tranform_published:
            return

        try:
            # Get the latest time for which the TF listener has the transform
            latest_time = self.tf_listener.getLatestCommonTime(
                "maze", msg.header.frame_id)

            # Set the timestamp of the message to this latest available time
            msg.header.stamp = latest_time

            # Perform the transformation
            transformed_pose = self.tf_listener.transformPose("maze", msg)
            publisher.publish(transformed_pose)

        except (tf.ExtrapolationException, tf.LookupException, tf.ConnectivityException) as e:
            MazeDB.printMsg(
                'WARNING', "TF exception when transforming pose: %s", str(e))

    def ros_callback_harness_pose(self, msg):
        # Set the broadcasting flag for the harness pose and transform and publish it
        self.is_harness_pose_broadcasting = True
        self.transform_and_publish_pose(msg, self.harness_pose_in_maze_pub)

    def ros_callback_gantry_pose(self, msg):
        # Set the broadcasting flag for the gantry pose and transform and publish it
        self.is_gantry_pose_broadcasting = True
        self.transform_and_publish_pose(msg, self.gantry_pose_in_maze_pub)

    def mazeboundary_marker0_callback(self, msg):
        # Update marker position and set the broadcasting flag
        self.optitrack_marker0 = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        self.is_marker0_broadcasting = True

    def mazeboundary_marker1_callback(self, msg):
        # Update marker position and set the broadcasting flag
        self.optitrack_marker1 = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        self.is_marker1_broadcasting = True

    def mazeboundary_marker2_callback(self, msg):
        # Update marker position and set the broadcasting flag
        self.optitrack_marker2 = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
        self.is_marker2_broadcasting = True


# @brief Main code
if __name__ == '__main__':
    # Initialize the ROS node with name 'maze_tf_broadcaster'
    rospy.init_node('maze_tf_broadcaster')
    MazeTransformer()  # Create an instance of the class
    rospy.spin()  # Keep the program running until it is explicitly shutdown
