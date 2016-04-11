#!/usr/bin/env python
import rospy
import sys
import pickle
import cPickle
import socket
import numpy as np
from nav_msgs.msg import OccupancyGrid
from map_utils import Map

class ClientNode():
    def __init__(self):
        rospy.init_node('seenmap_client_node')

        host = "134.126.125.125" # ip of server
        port = 13001
        self.BUFFER_SIZE = 20
        self.addr = (host, port)
        self.count = 0
        self.map_msg = None
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((host, port))
        except Exception as e:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            rospy.loginfo(e)
            sys.exit()

        rospy.Subscriber('/seenmap', OccupancyGrid, self.map_callback)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        
        rospy.spin()
            
    def map_callback(self, map_msg):
        self.map_msg = Map(map_msg)
        try:
            if self.count == 0:
                #rospy.loginfo(map_msg)
                self.count += 1
                self.s.sendall(map_msg)
            dat = self.s.recv(self.BUFFER_SIZE)
            rospy.loginfo(dat)

        except Exception as e:
            self.s.close()
            rospy.loginfo(e)
            

            
        
if __name__ == "__main__":
    ClientNode()
