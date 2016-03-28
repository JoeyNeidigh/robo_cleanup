#!/usr/bin/env python
import rospy
import pickle
import os
from socket import *
import sys
import socket
import time
from geometry_msgs.msg import Pose, PoseWithCovariance

# Command and control node for robo_cleanup
class CommandControl():
    def __init__(self):
        rospy.init_node('command_control')
        mess_pub = rospy.Publisher('mess', PoseWithCovariance, queue_size=10)
        teammate_marker_pub = rospy.Publisher('teammate_marker', Pose, queue_size=10)

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

        teammate_pose = Pose()
        rospy.loginfo("SERVER SETUP COMPLETE")

        while not rospy.is_shutdown():
            try:
                data, addr = s.recvfrom(1024)
                z = pickle.loads(data)
                teammate_pose.position.x = z[0]
                teammate_pose.position.y = z[1]
                teammate_marker_pub.publish(teammate_pose)
                rospy.loginfo(teammate_pose)
                s.sendto(data, ("134.126.125.237", 13000))
            except Exception as e:
                s.close()
                rospy.loginfo("ERROR. CLOSING SOCKET")


                
if __name__ == "__main__":
    CommandControl()
