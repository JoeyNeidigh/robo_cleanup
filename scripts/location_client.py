#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# This node will periodically send its position to the CC.
# It will also send a list of the messes that it has discovered to the CC.
class ClientNode():
    def __init__(self):
        rospy.init_node('client_node')
        rospy.Subscriber('/mess', Marker, self.mess_callback)

        host = "134.126.125.125" # ip of server
        port = 13000
        addr = (host, port)
        self.messes = []

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        ROBOT_ID = 1
        self.pos = [0,0]
        self.mess = [0,0]
        rate = rospy.Rate(5)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        mess_x = 0
        mess_y = 0
        while not rospy.is_shutdown():
            mess_x = self.mess[0]
            mess_y = self.mess[1]
            try:
                s.sendto(pickle.dumps([0, ROBOT_ID, self.pos[0], self.pos[1]]), addr)
                if mess_x != 0 and mess_y != 0 and self.is_new_mess(mess_x, mess_y):
                    rospy.loginfo("HERE!!!!!!")
                    self.messes.append((mess_x, mess_y))
                    s.sendto(pickle.dumps([1, ROBOT_ID, mess_x, mess_y]), addr)
            except Exception as e:
                s.close()
                rospy.loginfo("ERROR. CLOSING SOCKET")
                rospy.loginfo(e)
                sys.exit()

            rate.sleep()

    def is_new_mess(self, x, y):
        result = True
        if len(self.messes) >= 0:
            for m in self.messes:
                if (np.sqrt((x - m[0])**2 + (y - m[1])**2) < .1):
                    result = False
                    break
        return result

    # Callback for 'amcl_pose' topic
    def amcl_callback(self, amcl_msg):
        self.pos[0] = amcl_msg.pose.pose.position.x
        self.pos[1] = amcl_msg.pose.pose.position.y

    def mess_callback(self, msg):
        self.mess[0] = msg.pose.position.x
        self.mess[1] = msg.pose.position.y
            
        
if __name__ == "__main__":
    ClientNode()
