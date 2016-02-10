#!/usr/bin/env python
""" Wander safely using the Kinect.

Author: Nathan Sprague
Version: 9/10/2015

"""
import rospy
import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WanderNode(object):
    """ Wander node uses a hand-coded finite state machine to manage
    the obstacle avoidance logic. """





    BUMP_DISTANCE = 1.0
    BACKUP_DURATION = .5
    TURN_DURATION = 1.0

    # State identifiers
    DRIVE = 0
    BACKUP = 1
    TURN = 2


    def __init__(self):

        # record initial location and go home after 10 minutes


        # subscribe to our "finder" node so that once it finds 
        # something it drives up to it instead of just wandering


        # subscribe to scanner so that it doesn't 
        """ Initialize the bumber node. """
        rospy.init_node('bumper')

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.drive_pub = rospy.Publisher('/cmd_vel_mux/input/navi',
                                         Twist, queue_size=1)

        self.cur_distance = float('inf')

        self.state = self.DRIVE
        self.state_start_time = time.time()
        self.twist = Twist

    def scan_callback(self, scan_msg):
        """ bumper_msg will be of type LaserScan """

        # Grab the middle range value (straight ahead)...
        self.cur_distance = scan_msg.ranges[len(scan_msg.ranges) // 2]
        if math.isnan(self.cur_distance):
            self.cur_distance = 0  # Consider nan to be a collision


    def run(self):
        """ The main loop for this node. """
        twist = Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.state == self.DRIVE:
                twist.linear.x = .2
                twist.angular.z = 0

                if self.cur_distance < self.BUMP_DISTANCE:
                    self.state = self.BACKUP
                    self.state_start_time = time.time()
                    rospy.loginfo("SWITCHING TO BACKUP STATE")
                    continue

            elif self.state == self.BACKUP:
                twist.linear.x = -.2
                twist.angular.z = 0

                if time.time() > self.state_start_time + self.BACKUP_DURATION:
                    self.state = self.TURN
                    self.state_start_time = time.time()
                    rospy.loginfo("SWITCHING TO TURN STATE")
                    continue

            elif self.state == self.TURN:
                twist.linear.x = 0
                twist.angular.z = 1.0

                if time.time() > self.state_start_time + self.TURN_DURATION:
                    self.state = self.DRIVE
                    self.state_start_time = time.time()
                    rospy.loginfo("SWITCHING TO DRIVE STATE")
                    continue
            else:
                rospy.logerr("BAD STATE!")

            self.drive_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = WanderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

