#!/usr/bin/env python
import rospy
import map_utils
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class MapNode():
    def __init__(self):
        rospy.init_node('map_node')
        self.map_msg = None
        self.pos = None
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                            self.position_callback)

        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)
        
        occmap = map_utils.Map(self.map_msg) 

        seenmap = map_utils.Map(width = self.map_msg.info.width, height = self.map_msg.info.height, resolution = self.map_msg.info.resolution, origin_x = self.map_msg.info.origin.position.x, origin_y = self.map_msg.info.origin.position.y) 

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
            seenmap.grid[occmap._cell_index(self.pos.position.x, self.pos.position.y)] = 1
            img = cv2.cvtColor(seenmap.grid.astype(np.float32), cv2.COLOR_GRAY2BGR)
            cv2.imshow("MAP", img)
            cv2.waitKey(100)
     
  

    def position_callback(self, pos_msg):
        self.pos = pos_msg.pose.pose
        
    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

            
        
if __name__ == "__main__":
    MapNode()
