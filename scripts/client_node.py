#!/usr/bin/env python
import rospy
import os
import pickle
from socket import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

        # in client_node
            # subscribe to odom topic
            # in subscribe callback:
                # pull out just the info we need (pose probably)
                # pickle it using pickle.dumps(obj_to_pickle)
                # store pickle in global variable
            # in main loop:
                # send pickle string through the UDP socket to the server at some interval
class ClientNode():
    def __init__(self):
        rospy.init_node('client_node')
        host = "127.0.0.1" # ip of server
        port = 13000
        self.addr = (host, port)
        self.UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.pickled_str = ""
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            data = self.pickled_str
            self.UDPSock.sendto(data, self.addr)
            if data == "exit":
                break
            rate.sleep()

        self.UDPSock.close()
        os._exit(0)

    def odom_callback(self, odom_msg):
        self.pickled_str = pickle.dumps(odom_msg.pose)
        
if __name__ == "__main__":
    ClientNode()
