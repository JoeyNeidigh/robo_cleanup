#!/usr/bin/env python
import rospy
import sys
import pickle
import socket
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PointStamped

# This node will periodically send its position to the CC.
# It will also send a list of the messes that it has discovered to the CC.
class ClientNode():
    def __init__(self):
        rospy.init_node('client_node')
        rospy.Subscriber('/visualization_marker', Marker, self.marker_callback)

        host = "134.126.125.125" # ip of server
        port = 13000
        addr = (host, port)
        self.messes = []
        self.tf_listener = tf.TransformListener()

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            rospy.loginfo("FAILED TO CREATE SOCKET")
            sys.exit()

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        ROBOT_ID = 1
        self.pos = [0,0]
        self.mess = [None,None]
        rate = rospy.Rate(5)

        rospy.loginfo("LOCATION_CLIENT SETUP COMPLETE")
        self.old_marker = Point()
        self.old_marker.x = 0
        self.old_marker.y = 0
        while not rospy.is_shutdown():
            mess_x = self.mess[0]
            mess_y = self.mess[1]

            try:
                s.sendto(pickle.dumps([0, ROBOT_ID, self.pos[0], self.pos[1]]), addr)
##############################################################################################
#                      NEED TO SEND THEM ONLY IF THEY ARE DIFFERENT MESSES BECAUSE THIS CLIENT
#                      SENDS MESS IN ROBOT FRAME AND THE MESS_X AND MESS_Y WILL CHANGE WHEN
#                      THE ROBOT MOVES 
##############################################################################################
                if mess_x != None and mess_y != None:
                    rospy.loginfo("HERE!!!!!!")
                    s.sendto(pickle.dumps([1, ROBOT_ID, mess_x, mess_y]), addr)
                    old_mess_x = mess_x
                    old_mess_y = mess_y
            except Exception as e:
                s.close()
                rospy.loginfo("LOCATION_CLIENT ERROR. CLOSING SOCKET")
                rospy.loginfo(e)
                sys.exit()

            rate.sleep()

    def is_new_mess(self, xone, yone, xtwo, ytwo):
        return np.sqrt((xone - xtwo)**2 + (yone - ytwo)**2) < .2

    # Callback for 'amcl_pose' topic
    def amcl_callback(self, amcl_msg):
        self.pos[0] = amcl_msg.pose.pose.position.x
        self.pos[1] = amcl_msg.pose.pose.position.y

    def marker_callback(self, marker):
        """ The marker callback to see the ar_markers"""
        if not self.close_enough(self.old_marker.x, self.old_marker.y, marker.pose.position.x, marker.pose.position.y, .15):
            marker_point = PointStamped()
            marker_point.header.frame_id = 'camera_rgb_optical_frame'
            marker_point.header.stamp = rospy.get_rostime()
            marker_point.point.x = marker.pose.position.x
            marker_point.point.y = marker.pose.position.y
            marker_point.point.z = marker.pose.position.z

            try: # transform marker position into the map frame
                marker_point.header.stamp = rospy.get_rostime()
                self.tf_listener.waitForTransform('camera_rgb_optical_frame',
                                                  'map',
                                                  marker_point.header.stamp,
                                                  rospy.Duration(1.0))
                # get the point transform
                marker_point = self.tf_listener.transformPoint('map', marker_point) 
            except tf.Exception as e:
                print(e)
                print("ERROR in marker_callback")

            if self.close_enough(self.position.position.x, self.position.position.y, marker_point.point.x, marker_point.point.y, 1):
                self.mess_pub.publish(marker_point.point)
                self.old_marker.x = marker_point.point.x
                self.old_marker.y = marker_point.point.y

    def close_enough(self, x_one, y_one, x_two, y_two, threshold):
        """ Checks to see if its close enough to a goal"""
        return (np.sqrt((x_one - x_two)**2 + (y_one - y_two)**2) < threshold)
            
        
if __name__ == "__main__":
    ClientNode()
