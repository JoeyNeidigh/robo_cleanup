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

        odom_buf = 1024
        odom_port = 13000
        self.odom_addr = (host, odom_port)
        odom_UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.odom_UDPSock.bind(odom_addr)

        mess_buf = 1024
        mess_port = 13001
        self.mess_addr = (host, mess_port)
        mess_UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.mess_UDPSock.bind(mess_addr)

        self.pickled_str = ""

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/mess', PoseWithCovariance, pose_callback)

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
