#!/usr/bin/env python
import rospy
import pickle
import os
from socket import *
import sys
import socket
import time
from std_msgs.msg import Float32MultiArray

class MessServer():
    def __init__(self):
        rospy.init_node('mess_server')

        # Set up port for incomming traffic
        host = ""
        port = 13004

        self.mess_pub = rospy.Publisher('/messes_to_clean', Float32MultiArray, queue_size=10)
        mess_arr = Float32MultiArray()        

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

        rospy.loginfo("COMMAND AND CONTROL ONLINE")

        while not rospy.is_shutdown():
            try:
                data, addr = s.recvfrom(1024)
                z = pickle.loads(data)
                mess_arr.data = z
                self.mess_pub.publish(mess_arr)
                
            except Exception as e:
                s.close()
                rospy.loginfo(e)

                
if __name__ == "__main__":
    MessServer()
