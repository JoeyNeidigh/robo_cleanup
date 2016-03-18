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
        host = "134.126.125.125" # ip of server

        odom_port = 13000
        self.odom_addr = (host, odom_port)
        self.odom_UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.odom_UDPSock.bind(self.odom_addr)

        mess_port = 13001
        self.mess_addr = (host, mess_port)
        self.mess_UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.mess_UDPSock.bind(self.mess_addr)

        self.pickled_str = ""

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/mess', PoseWithCovariance, self.pose_callback)

        self.rate = rospy.Rate(1)

        #self.UDPSock.close()
        #os._exit(0)

    def odom_callback(self, odom_msg):
        a = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
        data = pickle.dumps(a)
        self.odom_UDPSock.sendto(data, self.odom_addr)
        self.rate.sleep()

    def pose_callback(self, pose_msg):
        data = pickle.dumps(pose_msg)
        self.mess_UDPSock.sendto(data, self.mess_addr)
        self.rate.sleep()
        
if __name__ == "__main__":
    ClientNode()
