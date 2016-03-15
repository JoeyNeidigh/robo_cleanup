#!/usr/bin/env python
import rospy
import pickle
import os
from socket import *
from geometry_msgs.msg import PoseWithCovariance

class ServerNode():
    def __init__(self):
        rospy.init_node('server_node')
        mess_pub = rospy.Publisher('mess', PoseWithCovariance, queue_size=10)
        teammate_marker_pub = rospy.Publisher('teammate_marker', PoseWithCovariance, queue_size=10)
        host = ""
        port = 13000
        self.buf = 1024
        self.addr = (host, port)
        self.UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.UDPSock.bind(self.addr)
        
        while True:
            (data, self.addr) = self.UDPSock.recvfrom(self.buf)
            depickled_str = pickle.loads(data)
            mode = depickled_str[0]
            depickled_str = depickled_str[1:]
            if mode == 0: # if odom message
                mess_pub.publish(depickled_str)
            elif mode == 1: # if mess message
                teammate_marker_pub.publish(depickled_str)

        self.UDPSock.close()
        os._exit(0)
                
if __name__ == "__main__":
    ServerNode()
