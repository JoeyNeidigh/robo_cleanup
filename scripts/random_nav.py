#!/usr/bin/env python

"""This node repeatedly selects random map positions and searches for victims.
   It logs the victims once found and then will report them back to the user.
   After a given time has elapsed, the robot will return to original spot.

  Authors: "Bamano"
  (Original Code for Random Nav provided by Nathan Sprague)

  Honor Code: We did not give or receive any unauthorized help for this 
  assignment.

  Version: 3.1

"""
import rospy
import map_utils
import random
import tf
import actionlib
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sound_play.msg import SoundRequest
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import *

def enum(**enums):
    """ Enumerated type for the states """
    return type('Enum', (), enums)

class RandomNavNode(object):
    def __init__(self):
        """ 
        The main control loop for searching and recording victim locations
        """
        # States for the state machine
        self.state = enum(SEARCHING=1, INVESTIGATING=2, GO_HOME=3)
    
        rospy.init_node('random_nav')
        # setup publishers and subscribers
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.Subscriber('/visualization_marker', Marker, self.marker_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                            self.position_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.marker_pub = rospy.Publisher('victim_marker', Marker,
                                            queue_size=10)
        self._talk_pub = rospy.Publisher('/robotsound', SoundRequest,
                                            queue_size=10)
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        self.tf_listener = tf.TransformListener()

        self.current_goal = (1000,1000)         # current nav goal
        self.TIME = 450                         # search time for robot
        self.old_goals = []                     # list of previous nav goals
        self.cv_bridge = CvBridge()             # for image conversion
        self.image = None                       # raw rgb image
        self.map_msg = None                     # sets up map
        self.victims = []                       # list of found victims
        self.start_time = rospy.get_time()      # start time of the robot
        self.home = False                       # signals when to go home
        self.position = None                    # robot's current position
        self.cur_state = self.state.SEARCHING   # current state of the robot
        self.cur_marker = Point()               # ar marker currently seen
        self.first_goal = True                  # signals if it is the 1st goal
        self.goals = 0                          # diagnostic count of old goals
        old_state = None                        # previous state of the robot

        # Set up a SoundRequest message:
        sound_request = SoundRequest()
        sound_request.sound = sound_request.SAY
        sound_request.command = sound_request.PLAY_ONCE
        sound_request.arg = "Report is Ready!"

        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(.1)

        self.map = map_utils.Map(self.map_msg)

        while (self.position is None and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for position...")
            rospy.sleep(.1)

        # sets the home postion
        home_pos = self.position

        # generate the first random goal
        x_goal = 100000
        y_goal = 100000

        while self.map.get_cell(x_goal, y_goal) != 0:
                    x_goal = random.random() * 20.0 - 10.0
                    y_goal = random.random() * 20.0 - 10.0
                    rospy.loginfo("Random goal found: ({}, {})".format(x_goal,
                                    y_goal))

        # main execution loop
        while not rospy.is_shutdown():
            local_marker = self.cur_marker
            local_state = self.cur_state

            # Go back to start after certain time limit
            local_state = self.go_home(local_state)

            # Set the goal to a random point
            if local_state is self.state.SEARCHING:
                goals = self.search(x_goal, y_goal, old_state)
                x_goal = goals[0]
                y_goal = goals[1]
            # Set the goal to the starting position
            elif local_state is self.state.GO_HOME:
                x_goal = home_pos.position.x
                y_goal = home_pos.position.y
            # Set the goal to the victim's position
            elif local_state is self.state.INVESTIGATING:
                local_state = self.investigate(local_state, local_marker)
                
            # Only set a new goal if the state has changed
            if (old_state is not local_state):
                goal = self.goal_message(x_goal, y_goal, 0)
                self.go_to_point(goal)
                self.old_goals.append((x_goal, y_goal))
                self.current_goal = (x_goal,y_goal)
        
            old_state = local_state
            self.cur_state = local_state

            # if going home, wait to reach goal then break out of loop
            if self.home:
                self.ac.wait_for_result()
                self._talk_pub.publish(sound_request)
                rospy.sleep(3)
                break

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

    def is_near_old_goal(self, x_goal, y_goal):
        """Function that checks if it is close to goal"""
        if len(self.old_goals) is 0:
            return False
        else:
            for g in self.old_goals:
                if self.close_enough(x_goal, y_goal, g[0], g[1], 1.25):
                    return True
        return False

    def go_home(self, loc_state):
        """ Sends the  robot home after a certain time"""
        state = loc_state
        if rospy.get_time() - self.start_time > self.TIME:
            state = self.state.GO_HOME
            self.home = True
        return state


    def search(self, x_goal, y_goal, old_state):
        """ Randomly Searches through the  map"""
        # if close enough to goal or it is the first goal
        if ((self.close_enough(self.position.position.x, 
             self.position.position.y, self.current_goal[0], 
             self.current_goal[1], .7) or self.ac.get_state() is 4) 
             or self.first_goal):

            if not self.first_goal:
                self.old_goals.append((x_goal, y_goal))

            # get new random goal
            x_goal = 10000
            y_goal = 10000
            while self.map.get_cell(x_goal, y_goal) != 0:
                x_goal = random.random() * 20.0 - 10.0
                y_goal = random.random() * 20.0 - 10.0
            self.first_goal = False

            # if the new goal is not near the old goal
            if not self.is_near_old_goal(x_goal, y_goal):
                goal = self.goal_message(x_goal, y_goal, 0)
                goal_marker = self.make_marker(x_goal, y_goal, .25, 255, 0, 0,
                                                self.goals, 'goals')
                self.marker_pub.publish(goal_marker)
                self.goals += 1
                self.ac.send_goal(goal)
                self.current_goal = (x_goal,y_goal)
          
        return (x_goal, y_goal)
    
    def investigate(self, local_state, local_marker):
        """ If the robot sees a marker he investigates the marker"""
        # if this is not the first victim
        if len(self.victims) is not 0:
            for victim in self.victims:
                # if it is too close to a previous victim, return to searching
                if (self.close_enough(victim.x, victim.y, local_marker.x,
                                     local_marker.y, 1)):
                    return self.state.SEARCHING

        # if robot is close enough to victim, record it
        if (self.close_enough(self.position.position.x,self.position.position.y,
                             local_marker.x, local_marker.y, 1)):
            # add victim to victims list and mark location on the map
            self.victims.append(local_marker)
            victim_marker = self.make_marker(local_marker.x, local_marker.y,
                                .35, 255, 165, 0, len(self.victims), 'victims')
            self.marker_pub.publish(victim_marker)
            # write to file
            f = open('/home/student/catkin_ws/src/zeta_rescue' +
                     '/results/victim_locations.csv', 'a')
            f.write(str(victim_marker.pose.position.x) + "," +
                    str(victim_marker.pose.position.y) + ",victim" +
                    str(victim_marker.id) + ".jpg" +  "\n")
            f.close()
            # return to searching state
            local_state = self.state.SEARCHING
            #take picture
            cv_img = self.cv_bridge.imgmsg_to_cv2(self.image, "bgr8")
            cv2.imwrite('/home/student/catkin_ws/src/zeta_rescue/results/' +
                        'victim' + str(victim_marker.id) + '.jpg', cv_img) 
        # if not close enough to victim, move up to it         
        else:
            # stop moving
            self.ac.cancel_goal()
            marker_point = PointStamped()
            marker_point.header.frame_id = 'ar_marker'
            marker_point.header.stamp = rospy.get_rostime()
            marker_point.point.x = 0
            marker_point.point.y = 0
            marker_point.point.z = .7
            # transformation to rotate the robot to face the victim
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
                print("ERROR in investigate")

            # send the goal to the robot
            theta = (euler[2] + np.pi/2) % (2 * (np.pi))
            goal = self.goal_message(marker_point.point.x,
                                     marker_point.point.y, theta)
            self.go_to_point(goal)
            self.ac.wait_for_result()
            
        return local_state

    def go_to_point(self, goal):
        """Sends the robot to a give goal point"""
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

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

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

        self.cur_marker = marker_point.point
        self.cur_state = self.state.INVESTIGATING
    
    def position_callback(self, pos):
        """ Saves the current position of the robot"""
        self.position = pos.pose.pose

    def image_callback(self, img):
        """ Saves the current image the kinect sees"""
        self.image = img

    def close_enough(self, x_one, y_one, x_two, y_two, threshold):
        """ Checks to see if its close enough to a goal"""
        return (np.sqrt((x_one - x_two)**2 + (y_one - y_two)**2) < threshold)


if __name__ == "__main__":
    RandomNavNode()
