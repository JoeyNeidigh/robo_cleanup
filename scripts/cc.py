#!/usr/bin/env python
import rospy
import pickle
import os
from socket import *
import sys
import socket
import time
from geometry_msgs.msg import Pose, PoseWithCovariance
from visualization_msgs.msg import Marker

# Command and control node for robo_cleanup.
# Aggregates the location of all robots and their corresponding messes.
# Divides up the messes among robots once it has collected mess lists from each robot
# Publishes to 'teammate_marker' and 'mess' topics for display in rviz
class CommandControl():
    def __init__(self):
        rospy.init_node('command_control')
        mess_pub = rospy.Publisher('mess', PoseWithCovariance, queue_size=10)
        teammate_marker_pub = rospy.Publisher('teammate_marker', Marker, queue_size=10)

        # Set up port for incomming traffic
        host = ""
        port = 13000
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        try:
            s.bind(("", port))
        except:
            rospy.loginfo("FAILED TO BIND")
            sys.exit()

        robot_pose = Pose()
        rospy.loginfo("COMMAND AND CONTROL ONLINE")

        while not rospy.is_shutdown():
            try:
                data, addr = s.recvfrom(1024)
                z = pickle.loads(data)
                robot_id = z[0]
                robot_pose.position.x = z[1]
                robot_pose.position.y = z[2]
            except Exception as e:
                s.close()
                rospy.loginfo(e)
            teammate_marker_pub.publish(self.make_marker(robot_pose, robot_id))

    # Create a marker for a robot whos position is stored in 'pose' and whose
    # ID is stored in 'robot_id'
    def make_marker(self, pose, robot_id):
        """ Create a Marker message with the given x,y coordinates """
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = 'map'
        m.ns = "robots"
        m.id = robot_id
        m.type = m.SPHERE
        m.action = m.ADD
        m.pose.position.x = pose.position.x
        m.pose.position.y = pose.position.y
        m.scale.x = m.scale.y = m.scale.z = .35 
        if robot_id == 0:
            m.color.r = 255
            m.color.g = 0
            m.color.b = 0
        else:
            m.color.r = 0
            m.color.g = 0
            m.color.b = 255
        m.color.a = 1.0
        return m
                
if __name__ == "__main__":
    CommandControl()
