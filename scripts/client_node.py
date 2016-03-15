#!/usr/bin/env python
import rospy
import os
import pickle
from socket import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

class ClientNode():
    def __init__(self):
        rospy.init_node('client_node')
        host = "127.0.0.1" # ip of server
        port = 13000
        self.addr = (host, port)
        self.UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.pickled_str = ""
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('mess', PoseWithCovariance, pose_callback)

        self.rate = rospy.Rate(1)

        #self.UDPSock.close()
        #os._exit(0)

    def odom_callback(self, odom_msg):
        data = "0" + pickle.dumps(odom_msg.pose)
        self.UDPSock.sendto(data, self.addr)
        self.rate.sleep()

    def pose_callback(self, pose_msg):
        data = "1" + pickle.dumps(pose_msg)
        self.UDPSock.sendto(data, self.addr)
        self.rate.sleep()
        
if __name__ == "__main__":
    ClientNode()
