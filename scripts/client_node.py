#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

class ClientNode():
    def __init__(self):
        rospy.init_node('client_node')

        host = "134.126.125.125" # ip of server
        port = 13000
        addr = (host, port)

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.odom = (0,0)
        self.rate = rospy.Rate(1)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        
        while rospy.is_shutdown():
            a = self.odom
            data = pickle.dumps(a)
            try:
                s.sendto(data, addr)
            except:
                s.close()
                rospy.loginfo("ERROR. CLOSING SOCKET")

    def odom_callback(self, odom_msg):
        self.odom = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
            
        
if __name__ == "__main__":
    ClientNode()
