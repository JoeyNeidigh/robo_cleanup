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
        self.messes = []

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        ROBOT_ID = 1
        self.pos = (0,0)
        self.mess = (0,0)
        rate = rospy.Rate(5)

        rospy.loginfo("CLIENT SETUP COMPLETE")
        
        while not rospy.is_shutdown():
            try:
                s.sendto(pickle.dumps([0, ROBOT_ID, self.pos[0], self.pos[1]]), addr)
                if self.is_new_mess(self.mess[0], self.mess[1]):
                    self.messes.append((self.mess[0], self.mess[1]))
                    s.sendto(pickle.dumps([1, ROBOT_ID, self.mess[0], self.mess[1]]), addr)
            except:
                s.close()
                rospy.loginfo("ERROR. CLOSING SOCKET")

            rate.sleep()

    def is_new_mess(self, x, y):
        result = True
        if len(self.messes >= 0):
            for m in self.messes:
                if (np.sqrt((x - m[0])**2 + (y - m[1])**2) < .1):
                    result = False
                    break
        return result

    # Callback for 'amcl_pose' topic
    def amcl_callback(self, amcl_msg):
        self.pos[0] = amcl_msg.pose.pose.position.x
        self.pos[1] = amcl_msg.pose.pose.position.y
            
        
if __name__ == "__main__":
    ClientNode()
