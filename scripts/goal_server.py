#!/usr/bin/env python
import rospy
import pickle
import os
import map_utils
import sys
import socket
import time
import random
from thread import start_new_thread
from geometry_msgs.msg import Pose, PoseWithCovariance
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool
import numpy as np


class CommandControl():
    def __init__(self):
        rospy.init_node('goal_server')
        teammate_marker_pub = rospy.Publisher('teammate_marker', Marker, queue_size=10)
        self.map_msg = None
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        goal_pub = rospy.Publisher('/new_goal', MoveBaseGoal, queue_size=10)
        rospy.Subscriber('/goal_reached', Bool, self.goal_reached_callback)
        self.og = None
        self.seenmap = None
        rospy.Subscriber('/seenmap', OccupancyGrid, self.seenmap_callback)

        # set up map for choosing goals
        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)

        

        self.map = map_utils.Map(self.map_msg)
        self.goal_reached = True

        # Set up port for incomming traffic
        host = ""
        port = 13005
        BUFFER_SIZE = 1024
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = "map"

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        try:
            s.bind(("", port))
            s.listen(1)
            conn, addr = s.accept()
            #rospy.loginfo("WAITING FOR CONNECTION")
        except Exception as e:
            rospy.loginfo("FAILED TO BIND")
            rospy.loginfo(e)
            sys.exit()

        rospy.loginfo("GOAL SERVER ONLINE")
        
        # main loop
        while not rospy.is_shutdown():
            self.seenmap = map_utils.Map(self.og)
            try:
                goal = self.random_goal()
                conn.send(pickle.dumps(goal))
                response = conn.recv(BUFFER_SIZE)
                if response == "UNSEEN" and self.goal_reached == True:
                    rospy.loginfo("GOAL CHOSEN (%f, %f)", goal[0], goal[1])
                    goal_msg.target_pose.header.stamp = rospy.get_rostime()
                    goal_msg.target_pose.pose.position.x = goal[0]
                    goal_msg.target_pose.pose.position.y = goal[1]
                    goal_pub.publish(goal_msg)
                else:
                    rospy.loginfo("TRYING ANOTHER GOAL")
            except Exception as e:
                s.close()
                rospy.loginfo("SEENMAP SERVER ERROR:")
                rospy.loginfo(e)
                sys.exit()
            rospy.sleep(5)

    def random_goal(self):
        x_goal = 100000
        y_goal = 100000

        while self.map.get_cell(x_goal, y_goal) != 0 and not self.seen((x_goal, y_goal)):
            x_goal = random.random() * 20.0 - 10.0
            y_goal = random.random() * 20.0 - 10.0

        return (x_goal, y_goal)

    def seen(self, goal):
        return self.seenmap.get_cell(goal[0], goal[1]) == 1

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

    def goal_reached_callback(self, msg):
        self.goal_reached = msg.data

    def seenmap_callback(self, msg):
        self.og = msg

  
                
if __name__ == "__main__":
    CommandControl()
