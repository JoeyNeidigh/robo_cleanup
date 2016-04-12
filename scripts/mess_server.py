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

class MessServer():
    def __init__(self):
        rospy.init_node('mess_server')

        # Set up port for incomming traffic
        host = ""
        port = 13004
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
            except Exception as e:
                s.close()
                rospy.loginfo(e)

                
if __name__ == "__main__":
    CommandControl()
