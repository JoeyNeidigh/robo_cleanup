#!/usr/bin/env python
import rospy
import map_utils
import random
import tf
import actionlib
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker
from sound_play.msg import SoundRequest

class RoboCleanupNode(object):
    def __init__(self):
        rospy.init_node('robo_cleanup')

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        self.map_msg = None

        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(.1)

        self.map = map_utils.Map(self.map_msg)

        while not rospy.is_shutdown():
            x_goal = 100000
            y_goal = 100000

            while self.map.get_cell(x_goal, y_goal) != 0:
                x_goal = random.random() * 20.0 - 10.0
                y_goal = random.random() * 20.0 - 10.0

            rospy.loginfo("Random goal found: ({}, {})".format(x_goal, y_goal))

            goal = self.goal_message(x_goal, y_goal, 0)
            rospy.loginfo("Waiting for server.")
            self.ac.wait_for_server()

            rospy.loginfo("Sending goal.")
            self.ac.send_goal(goal)
            rospy.loginfo("Goal Sent.")

            # Wait until the server reports a result.
            self.ac.wait_for_result()
            rospy.loginfo(self.ac.get_goal_status_text())

    def goal_message(self, x_target, y_target, theta_target):
        """ Create a goal message in the base_link coordinate frame"""

        quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
        # Create a goal message ...
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = x_target
        goal.target_pose.pose.position.y = y_target
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg


if __name__ == "__main__":
    RandomNavNode()
