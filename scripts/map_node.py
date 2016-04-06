#!/usr/bin/env python
import rospy
import map_utils
import numpy as np
import cv2
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped

class MapNode():
    def __init__(self):
        rospy.init_node('map_node')
        self.map_msg = None
        self.pos = None
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                            self.position_callback)

        self.tf_listener = tf.TransformListener()

        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)
        
        occmap = map_utils.Map(self.map_msg) 

        seenmap = map_utils.Map(width = self.map_msg.info.width,
                                height = self.map_msg.info.height,
                                resolution = self.map_msg.info.resolution,
                                origin_x = self.map_msg.info.origin.position.x,
                                origin_y = self.map_msg.info.origin.position.y) 

        while (self.pos is None and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for position...")
            rospy.sleep(.1)
        
        # set initial values of seenmap. -1 = unknown, 0 = unseen, 1 = seen
        it = np.nditer(occmap.grid, flags=['multi_index'])
        while not it.finished:
            if it[0] < 0:
                seenmap.grid[it.multi_index] = -1
            elif it[0] == 0:
                seenmap.grid[it.multi_index] = 0
            else:
                seenmap.grid[it.multi_index] = 1
            it.iternext()

        while not rospy.is_shutdown():
            img = cv2.cvtColor(seenmap.grid.astype(np.float32), cv2.COLOR_GRAY2BGR)
            seenmap.grid[occmap._cell_index(self.pos.position.x, self.pos.position.y)] = 1
            # probably gonna need to use tf to find the final two points of the triangle
            # just transform the points (x=1, y= +- .5) from the robot frame to map
            # robot's position is the 3rd point of the triangle
            # use cv2.fillConvexPoly to draw onto img
            pts = self.calc_triangle()
            one_x, one_y = occmap._cell_index(pts[0], pts[1])
            two_x, two_y = occmap._cell_index(pts[2], pts[3])
            three_x, three_y = occmap._cell_index(self.pos.position.x, self.pos.position.y)
            triangle = np.array([[one_x, one_y], [two_x, two_y], [three_x, three_y]], np.int32)
            cv2.fillConvexPoly(img, triangle, 1)

            # display seenmap image to screen
            cv2.imshow("MAP", img)
            cv2.waitKey(10)
 
     
    def calc_triangle(self):
        # transform for the first corner of triangle
        fst_pt = PointStamped()
        fst_pt.header.frame_id = 'base_link'
        fst_pt.point.x = 1
        fst_pt.point.y = .5
        fst_pt.header.stamp = rospy.get_rostime()
        try: 
            self.tf_listener.waitForTransform('base_link', # from here
                                              'map',     # to here
                                              fst_pt.header.stamp,
                                              rospy.Duration(1.0))

            fst_pt = self.tf_listener.transformPoint('map', fst_pt)
        except tf.Exception as e:
            rospy.loginfo(e)
        
        # transform for the second corner of triangle
        snd_pt = PointStamped()
        snd_pt.header.frame_id = 'base_link'
        snd_pt.point.x = 1
        snd_pt.point.y = -.5
        snd_pt.header.stamp = rospy.get_rostime()
        try: 
            self.tf_listener.waitForTransform('base_link', # from here
                                              'map',     # to here
                                              snd_pt.header.stamp,
                                              rospy.Duration(1.0))

            snd_pt = self.tf_listener.transformPoint('map', snd_pt)
        except tf.Exception as e:
            rospy.loginfo(e)

        return [fst_pt.point.x, fst_pt.point.y, snd_pt.point.x, snd_pt.point.y]

    def position_callback(self, pos_msg):
        self.pos = pos_msg.pose.pose
        
    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

            
        
if __name__ == "__main__":
    MapNode()
