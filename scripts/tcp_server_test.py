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
from nav_msgs.msg import OccupancyGrid


class CommandControl():
    def __init__(self):
        rospy.init_node('command_control')
        teammate_marker_pub = rospy.Publisher('teammate_marker', Marker, queue_size=10)

        # Set up port for incomming traffic
        host = ""
        port = 13001
        BUFFER_SIZE = 1024
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        try:
            s.bind(("", port))
            s.listen(1)
            conn, addr = s.accept()
        except:
            rospy.loginfo("FAILED TO BIND")
            sys.exit()

        rospy.loginfo("COMMAND AND CONTROL ONLINE")
        og = OccupancyGrid()

        while not rospy.is_shutdown():
            try:
                data = conn.recv(BUFFER_SIZE)
                z = pickle.loads(data)
                rospy.loginfo(z.header)
                conn.send(data)
             
            except Exception as e:
                s.close()
                rospy.loginfo(e)

  
                
if __name__ == "__main__":
    CommandControl()
