#!/usr/bin/env python
import rospy
import sys
import pickle
import cPickle
import socket
import numpy as np
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool
import map_utils

class ClientNode():
    def __init__(self):
        rospy.init_node('goal_client')
        self.og = None
        rospy.Subscriber('/seenmap', OccupancyGrid, self.seenmap_callback)

        host = "134.126.125.237" # ip of server
        port = 13006
        BUFFER_SIZE = 1024
        self.goal_reached = True

        # set up socket
        connected = False
        count = 0
        while not connected:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((host, port))
                connected = True
            except Exception as e:
                if count <= 10:
                    rospy.loginfo("FAILED TO CONNECT. RETRYING...")
                    connected = False
                    count += 1
                    rospy.sleep(1)
                else:
                    rospy.loginfo("I QUIT")
                    sys.exit()

        while self.og is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for seen_map...")
            rospy.sleep(1)

        rospy.loginfo("GOAL CLIENT ONLINE")

        s.send("CLIENT READY")
        
        while not rospy.is_shutdown():
            self.seenmap = map_utils.Map(self.og)
            try:
                goal = pickle.loads(s.recv(BUFFER_SIZE))
                if self.seen(goal):
                    s.send("SEEN")
                else:
                    s.send("UNSEEN")
            except Exception as e:
                s.close()
                rospy.loginfo("TCP_TEST ERROR:")
                rospy.loginfo(e)
                sys.exit()

    def seen(self, goal):
        return self.seenmap.get_cell(goal[0], goal[1]) == 1

    def seenmap_callback(self, msg):
        self.og = msg

if __name__ == "__main__":
    ClientNode()
