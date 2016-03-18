#!/usr/bin/env python
import rospy
import pickle
import os
from socket import *
from geometry_msgs.msg import Pose, PoseWithCovariance


class ServerNode():
    def __init__(self):
        rospy.init_node('server_node')
        mess_pub = rospy.Publisher('mess', PoseWithCovariance, queue_size=10)
        teammate_marker_pub = rospy.Publisher('teammate_marker', Pose, queue_size=10)

        host = ""

        odom_buf = 1024
        odom_port = 13000
        odom_addr = (host, odom_port)
        odom_UDPSock = socket(AF_INET, SOCK_DGRAM)
        odom_UDPSock.bind(odom_addr)

        mess_buf = 1024
        mess_port = 13001
        mess_addr = (host, mess_port)
        mess_UDPSock = socket(AF_INET, SOCK_DGRAM)
        mess_UDPSock.bind(mess_addr)

        teammate_pose = Pose()
        
        while True:
            (odom_data, odom_addr) = odom_UDPSock.recvfrom(odom_buf)
            depickled_odom = pickle.loads(odom_data)
            teammate_pose.x = depickled_odom[0]
            teammate_pose.y = depickled_odom[1]
            teammate_marker_pub.publish(teammate_odom)
            rospy.loginfo(depickled_odom)
            
            (mess_data, mess_addr) = mess_UDPSock.recvfrom(mess_buf)
            depickled_mess = pickle.loads(mess_data)
            mess_pub.publish(depickled_mess)               

        self.UDPSock.close()
        os._exit(0)
                
if __name__ == "__main__":
    ServerNode()
