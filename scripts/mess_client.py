#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class MessClient():
    def __init__(self):
        rospy.init_node('mess_client')

        host = "134.126.125.125" # ip of server
        port = 13004 
        addr = (host, port)

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rate = rospy.Rate(5)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        
        while not rospy.is_shutdown():
            a = self.position
            data = pickle.dumps(a)
            try:
                if old_mess != new_mess:
                s.sendto(data, addr)

            except:
                s.close()
                rospy.loginfo("ERROR. CLOSING SOCKET")

            rate.sleep()

        
            
        
if __name__ == "__main__":
    ClientNode()
