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

        host1 = "134.126.125.236" # ip of server
        host2 = "134.126.125.125
        port = 13004 
        self.addr1 = (host1, port)
        self.addr2 = (host2, port)

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
        #split data into two seperate arrays
        try:
            self.s.sendto(data, self.addr1)
            self.s.sendto(data, self.addr2)

        except:
            self.s.close()
            rospy.loginfo("ERROR. CLOSING SOCKET")
        
        
if __name__ == "__main__":
    MessClient()
