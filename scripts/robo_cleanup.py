#!/usr/bin/env python
import rospy
import map_utils
import tf
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray

class RoboCleanupNode(object):
    def __init__(self):
        rospy.init_node('robo_cleanup')
        # Subscribers/Publishers
        self.map_msg = None
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                            self.position_callback)
        rospy.Subscriber('new_goal', MoveBaseGoal, self.new_goal_callback)
        rospy.Subscriber('messes_to_clean', Float32MultiArray, self.mess_arr_callback)
        self.mv_base = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.goal_reached = rospy.Publisher('/goal_reached', Bool, queue_size=10)
        # Other globals
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.position = None
        self.goal = None
        self.searching = True
        self.cleaning = False
        self.cur_mess = None
        self.mess_id = 1
        self.tf_listener = tf.TransformListener()
        self.old_marker = Point()
        self.old_marker.x = 0
        self.old_marker.y = 0

        # Wait for the map and the robot's position to be initialized
        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)

        self.map = map_utils.Map(self.map_msg)
        
        while (self.position is None and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for position...")
            rospy.sleep(.1)
        
        # Initially set the safezone to the start location 
        self.safezone = self.position
        counter = 0

        # Main execution loop
        while not rospy.is_shutdown():   
            if (self.searching and self.goal is not None): # if in searching state AND has a valid goal
                # if first goal
                if counter == 0:
                    goal = self.goal_message(self.goal.x, self.goal.y, 0)
                    self.ac.send_goal(goal)
                    counter += 1

                # if close enough to the goal OR the goal has been reached OR the goal has been abandoned
                if (self.close_enough(self.position.position.x,self.position.position.y, self.goal.x, self.goal.y, .7)
                    or self.ac.get_state() is 4 or self.ac.get_state() is 3):
                    goal = self.goal_message(self.goal.x, self.goal.y, 0)
                    self.ac.send_goal(goal)
                self.goal = None                
                
            if self.cleaning: # if in cleaning state
                length = len(self.mess_arr)/2
                count = 0 
                # retreive each mess in self.mess_arr
                for i in range(0, length):
                    self.drive_to_mess(self.mess_arr[i+count], self.mess_arr[i+1+count])
                    self.take_to_safezone()         
                    count += 1
                self.cleaning = False
    	
    def take_to_safezone(self):
        """ Return to safezone with the mess and drop it off """
        mv_back = Twist()
        mv_back.linear.x = -.5
        goal = self.goal_message(self.safezone.position.x, self.safezone.position.y, 0)
        self.go_to_point(goal)
        self.ac.wait_for_result() 
        self.mv_base.publish(mv_back)      

    def drive_to_mess(self, mess_x, mess_y):
        """ Drive to the mess located at (mess_x, mess_y) and gather it in the plow """
        goal = self.goal_message(mess_x, mess_y, 0)
        self.go_to_point(goal)
        self.ac.wait_for_result()

    def go_to_point(self, goal):
        """ Sends the robot to a given goal point """
        rospy.loginfo("Waiting for server.")
        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        rospy.loginfo("Goal Sent.")

    def goal_message(self, x_target, y_target, theta_target):
        """ Create a goal message in the base_link coordinate frame """
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

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

    def position_callback(self, pos):
        """ Saves the current position of the robot """
        self.position = pos.pose.pose

    def close_enough(self, x_one, y_one, x_two, y_two, distance):
        """ Checks to see if (x_one, y_one) is within the given distance of (x_two, y_two) """
        return (np.sqrt((x_one - x_two)**2 + (y_one - y_two)**2) < distance)

    def new_goal_callback(self, new_goal):
        """ Continuously set self.goal to be a valid goal """
        self.goal = new_goal.target_pose.pose.position

    def mess_arr_callback(self, messes_msg):
        """ Sets self.mess_arr to the array of goals that this robot is responsible for """
        """ and switches the state from searching to cleaning                           """
        self.mess_arr = messes_msg.data
        self.searching = False
        self.cleaning = True
        self.ac.cancel_goal()

if __name__ == "__main__":
    RoboCleanupNode()
