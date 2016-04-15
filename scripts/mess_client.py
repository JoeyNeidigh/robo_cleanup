#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray

class MessClient():
    def __init__(self):
        rospy.init_node('mess_client')

        host1 = "134.126.125.236" # ip of server
        host2 = "134.126.125.237"
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

    def split_list(self, a_list):
        a, b = 
        if len(mess_msg.data)/2 % 2 is 0:
            half = len(a_list)/2
            a,b = a_list[:half], a_list[half:]
        elif len(mess_msg.data)/2 is 1:
            a = a_list
            b = []
        else:
            half = len(a_list)/2
            a,b = a_list[:half-1], a_list[half-1:]
        return a,b

    def mess_arr_callback(self, mess_msg):
        rospy.loginfo("callback")
        
        try:
            if len(mess_msg.data) != 0:
                list_a, list_b = self.split_list(mess_msg.data)
                data1 = pickle.dumps(list_a)
                data2 = pickle.dumps(list_b)
                self.s.sendto(data1, self.addr1)
                self.s.sendto(data2, self.addr2)
                rospy.loginfo("SENT MESS LISTS")

        except Exception as e:
            self.s.close()
            rospy.loginfo(e)
            rospy.loginfo("ERROR. CLOSING SOCKET")
        
        
if __name__ == "__main__":
    MessClient()
