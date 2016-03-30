#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# This node will periodically send its position to the CC.
# It will also send a list of the messes that it has discovered to the CC.
class ClientNode():
    def __init__(self):
        rospy.init_node('client_node')

        host = "134.126.125.125" # ip of server
        port = 13000
        addr = (host, port)

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        robot_id = 1
        self.position = [robot_id,0,0]
        rate = rospy.Rate(5)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        
        while not rospy.is_shutdown():
            a = self.position
            data = pickle.dumps(a)
            try:
                s.sendto(data, addr)
            except:
                s.close()
                rospy.loginfo("ERROR. CLOSING SOCKET")

            rate.sleep()

    # Callback for 'amcl_pose' topic
    def amcl_callback(self, amcl_msg):
        self.position[1] = amcl_msg.pose.pose.position.x
        self.position[2] = amcl_msg.pose.pose.position.y
            
        
if __name__ == "__main__":
    ClientNode()
