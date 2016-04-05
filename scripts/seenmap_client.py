#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
from nav_msgs.msg import OccupancyGrid

# This node will periodically send its position to the CC.
# It will also send a list of the messes that it has discovered to the CC.
class SeenmapClientNode():
    def __init__(self):
        rospy.init_node('seenmap_client_node')

        host = "134.126.125.125" # ip of server
        port = 13001
        addr = (host, port)

        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.Subscriber('/seenmap', OccupancyGrid, self.map_callback)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        
        rospy.spin()
            
    # Callback for 'amcl_pose' topic
    def map_callback(self, map_msg):
        data = pickle.dumps(map_msg)
        try:
            self.s.sendto(data, addr)
        except:
            self.s.close()
            rospy.loginfo("ERROR. CLOSING SOCKET")


            
        
if __name__ == "__main__":
    SeenmapClientNode()
