#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msg.msgs import Float32MultiArray

class MessClient():
    def __init__(self):
        rospy.init_node('mess_client')

        host = "134.126.125.236" # ip of server
        port = 13004 
        self.addr = (host, port)

        rospy.Subscriber('mess_arr', Float32MultiArray, self.mess_arr_callback) 

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.loginfo("MESS CLIENT SETUP COMPLETE")
        
        rospy.spin()

    def mess_arr_callback(self, mess_msg):
        data = pickle.dumps(mess_msg.data)
        try:
            self.s.sendto(data, self.addr)

        except:
            self.s.close()
            rospy.loginfo("ERROR. CLOSING SOCKET")
        
        
if __name__ == "__main__":
    MessClient()
