#!/usr/bin/env python
import rospy
import pickle
import os
import sys
import socket
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

# Command and control node for robo_cleanup.
# Aggregates the location of all robots and their corresponding messes.
# Divides up the messes among robots once it has collected mess lists from each robot
# Publishes to 'teammate_marker' and 'mess' topics for display in rviz
class CommandControl():
    def __init__(self):
        rospy.init_node('command_control')
        teammate_marker_pub = rospy.Publisher('/teammate_marker', Marker, queue_size=10)
        mess_marker_pub = rospy.Publisher('/mess_marker', Marker, queue_size=10)
        mess_arr_pub = rospy.Publisher('/mess_arr', Float32MultiArray, queue_size=10)

        self.messes = []
        self.TIME = 60
        self.start_time = rospy.get_time()

        # Set up port for incomming traffic
        host = ""
        port = 13000
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        try:
            s.bind(("", port))
        except:
            rospy.loginfo("FAILED TO BIND")
            sys.exit()

        rospy.loginfo("COMMAND AND CONTROL ONLINE")
        arr = Float32MultiArray()
        counter = 0
        while not rospy.is_shutdown():
            try:
                data, addr = s.recvfrom(1024)
                z = pickle.loads(data)
                if z[0] == 0:
                    teammate_marker_pub.publish(self.make_marker(z[1], z[2], z[3], 'robots'))
                elif z[0] == 1 and self.is_new_mess(z[2], z[3]):
                    rospy.loginfo("HERE!!!")
                    mess_marker_pub.publish(self.make_marker(z[1], z[2], z[3], 'mess'))

                if(self.is_time_to_clean() and counter == 0):
                    arr.data = self.to_array()
                    mess_arr_pub.publish(arr)
                    counter += 1
            except Exception as e:
                s.close()
                rospy.loginfo(e)
                sys.exit()

    def to_array(self):
        array = []
        for m in self.messes:
            array.append(m.pose.position.x)
            array.append(m.pose.position.y)

        return array
    
    def is_time_to_clean(self):
        if rospy.get_time() - self.start_time > self.TIME: 
            return True
        return False
       
    # Create a marker for a robot whos position is stored in 'pose' and whose
    # ID is stored in 'robot_id'
    def make_marker(self, robot_id, x, y, ns):
        """ Create a Marker message with the given x,y coordinates """
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = 'map'
        m.ns = ns
        m.action = m.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        if robot_id == 0:
            m.color.r = 255
            m.color.g = 0
            m.color.b = 0
        else:
            m.color.r = 0
            m.color.g = 0
            m.color.b = 255
        m.color.a = 1.0

        if ns == 'mess':
            m.type = m.CUBE
            m.id = len(self.messes)
            m.scale.x = m.scale.y = m.scale.z = .15
            self.messes.append(m)
        else:
            m.type = m.SPHERE
            m.id = robot_id
            m.scale.x = m.scale.y = m.scale.z = .35
        return m

    def is_new_mess(self, x, y):
        result = True
        if len(self.messes) >= 0:
            for m in self.messes:
                if (np.sqrt((x - m.pose.position.x)**2 + (y - m.pose.position.y)**2) < .2):
                    result = False
                    break
        return result
                
if __name__ == "__main__":
    CommandControl()
    
