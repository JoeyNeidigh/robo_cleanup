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

        odom_port = 13000
        self.odom_addr = (host, odom_port)
        self.odom_UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.odom_UDPSock.bind(self.odom_addr)

        mess_port = 13001
        self.mess_addr = (host, mess_port)
        self.mess_UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.mess_UDPSock.bind(self.mess_addr)
        host = "134.126.125.125" # ip of server
        port = 13000
        self.addr = (host, port)

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        self.pickled_str = ""

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/mess', PoseWithCovariance, self.pose_callback)
        rospy.Subscriber('/mess', 

        self.rate = rospy.Rate(1)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        
        rospy.spin()

    def odom_callback(self, odom_msg):
        a = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
        data = pickle.dumps(a)
        try:
            self.s.sendto(data, self.addr)
        except:
            s.close()
            rospy.loginfo("ERROR. CLOSING SOCKET")
            
        #self.rate.sleep()
        
if __name__ == "__main__":
    ClientNode()
