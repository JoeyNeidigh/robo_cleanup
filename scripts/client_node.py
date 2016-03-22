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
        self.addr = (host, port)

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

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
