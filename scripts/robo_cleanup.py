#!/usr/bin/env python
import rospy
import map_utils
import random
import tf
import actionlib
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker
from sound_play.msg import SoundRequest
from geometry_msgs.msg import Twist

"""
Might Use later on for a state machine

def enum(**enums):
    Enumerated type for the states
    return type('Enum', (), enums)
"""

class RoboCleanupNode(object):
    def __init__(self):
        rospy.init_node('robo_cleanup')

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.map_msg = None
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/visualization_marker', Marker, self.marker_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                            self.position_callback)

        self.marker_pub = rospy.Publisher('mess_marker', Marker,
                                            queue_size=10)
 
        #Add a publisher to publish Twists message to the base to back up robo
        self.mv_base = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        #Add a subscriber that listens for the list of mess objects
        
        #Add a publisher to publish different mess objects it sees in its own view
        self.mess = rospy.Publisher('/mess', Pose, queue_size=10)

        self.mess_id = 1
        self.position = None
        self.searching = False
        self.cur_mess = None
        self.tf_listener = tf.TransformListener()

        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)

        self.map = map_utils.Map(self.map_msg)
        
        while (self.position is None and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for position...")
            rospy.sleep(.1)
        
	    #Initially Sets the safezone to just the start location 
        self.safezone = self.position
   

        while not rospy.is_shutdown():
            #Search the space randomly eventually will ask CC for locations to go to 
            if not self.searching or self.ac.get_state() is 3 or self.ac.get_state() is 4:
                self.random_search()

            # Just does the current mess it sees 
            # Need to have it search through a shared list of the mess objects and
            # have it choose a object with an algorithm 
            if (self.cur_mess is not None):
                self.drive_to_mess(self.cur_mess)
                #Take mess to safezone
                self.take_to_safezone()         
                #return searching
                self.cur_mess = None  
                self.searching = False      
    
            
    def take_to_safezone(self):
	mv_back = Twist()
	mv_back.linear.x = -.5

        goal = self.goal_message(self.safezone.position.x, self.safezone.position.y, 0)
        self.go_to_point(goal)
        self.ac.wait_for_result() 
        #Send twist messsage to back up a foot to drop off the mess
	self.mv_base.publish(mv_back)      

    def drive_to_mess(self, mess):
        # stop moving
        self.ac.cancel_goal()
        marker_point = PointStamped()
        marker_point.header.frame_id = 'ar_marker'
        marker_point.header.stamp = rospy.get_rostime()
        marker_point.point.x = 0
        marker_point.point.y = 0
        marker_point.point.z = .3
        # transformation to rotate the robot to face the mess
        quat = (self.mark_angles.orientation.x,
                self.mark_angles.orientation.y,
                self.mark_angles.orientation.z,
                self.mark_angles.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        marker_point.header.stamp = rospy.get_rostime()
        try: # transform the marker to the map frame
            self.tf_listener.waitForTransform('ar_marker', # from here
                                              'map',     # to here
                                              marker_point.header.stamp,
                                              rospy.Duration(1.0))

            marker_point = self.tf_listener.transformPoint('map',
                                                            marker_point)

        except tf.Exception as e:
            print(e)

        # send the goal to the robot
        theta = (euler[2] + np.pi/2) % (2 * (np.pi))
        goal = self.goal_message(marker_point.point.x,
                                 marker_point.point.y, theta)
        self.go_to_point(goal)
        self.ac.wait_for_result() 

        mess_marker = self.make_marker(marker_point.point.x, marker_point.point.y, .25, 255, 0, 0,
                                                self.mess_id, 'mess')
        self.mess_id += 1
        self.marker_pub.publish(mess_marker)
            
    def random_search(self):
        x_goal = 100000
        y_goal = 100000

        while self.map.get_cell(x_goal, y_goal) != 0:
            x_goal = random.random() * 20.0 - 10.0
            y_goal = random.random() * 20.0 - 10.0

        rospy.loginfo("Random goal found: ({}, {})".format(x_goal, y_goal))

        goal = self.goal_message(x_goal, y_goal, 0)
        
        self.go_to_point(goal)
        self.searching = True
        # Wait until the server reports a result.
        #self.ac.wait_for_result()
        #rospy.loginfo(self.ac.get_goal_status_text())

    def go_to_point(self, goal):
        """Sends the robot to a given goal point"""
        rospy.loginfo("Waiting for server.")
        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        rospy.loginfo("Goal Sent.")

    def goal_message(self, x_target, y_target, theta_target):
        """ Create a goal message in the base_link coordinate frame"""

        quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
        # Create a goal message ...
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = x_target
        goal.target_pose.pose.position.y = y_target
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal
 
    def marker_callback(self, marker):
        """ The marker callback to see the ar_markers"""
        marker_point = PointStamped()
        marker_point.header.frame_id = 'camera_rgb_optical_frame'
        marker_point.header.stamp = rospy.get_rostime()
        marker_point.point.x = marker.pose.position.x
        marker_point.point.y = marker.pose.position.y
        marker_point.point.z = marker.pose.position.z

        marker_angles = PoseStamped()
        marker_angles.header.frame_id = 'camera_rgb_optical_frame'
        marker_angles.header.stamp = rospy.get_rostime()
        marker_angles.pose = marker.pose

        try: # transform marker position into the map frame
            marker_point.header.stamp = rospy.get_rostime()
            marker_angles.header.stamp = rospy.get_rostime()
            self.tf_listener.waitForTransform('camera_rgb_optical_frame',
                                              'map',
                                              marker_point.header.stamp,
                                              rospy.Duration(1.0))
            # get the point transform
            marker_point = self.tf_listener.transformPoint('map', marker_point) 
            # get the pose  transform to get correct rotation
            marker_angles = self.tf_listener.transformPose('map',marker_angles) 
            self.mark_angles = marker_angles.pose
            
        except tf.Exception as e:
            print(e)
            print("ERROR in marker_callback")

        self.cur_mess = marker_point.point

    def make_marker(self, x, y, size, r, g, b, ID, ns):
        """ Create a Marker message with the given x,y coordinates """
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = 'map'
        m.ns = ns
        m.id = ID
        m.type = m.SPHERE
        m.action = m.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.scale.x = m.scale.y = m.scale.z = size    
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        return m

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg


    def position_callback(self, pos):
        """ Saves the current position of the robot"""
        self.position = pos.pose.pose

if __name__ == "__main__":
    RoboCleanupNode()
