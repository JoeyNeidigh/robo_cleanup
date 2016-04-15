#!/usr/bin/env python
import socket
import cv2
import pickle
import sys
import struct
import rospy
import map_utils
import numpy as np
from nav_msgs.msg import OccupancyGrid

class SeenmapServer():
    def __init__(self):
        rospy.init_node('seenmap_server_a')
        seenmap_pub = rospy.Publisher('/seenmap_a', OccupancyGrid, queue_size=5)
        self.map_msg = None
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.bind(("", 13008))
            sock.listen(1)
            sock, addr = sock.accept()
        except Exception as e:
            sock.close()
            rospy.loginfo("SEENMAP_SERVER ERROR:")
            rospy.loginfo(e)
            sys.exit()

        seenmap = map_utils.Map(width = self.map_msg.info.width,
                                height = self.map_msg.info.height,
                                resolution = self.map_msg.info.resolution,
                                origin_x = self.map_msg.info.origin.position.x,
                                origin_y = self.map_msg.info.origin.position.y) 

        while True:
            message = self.recv_msg(sock)
            if not message is None:
                rospy.loginfo("here")
                seenmap.grid = pickle.loads(message)
                og = seenmap.to_message()
                seenmap_pub.publish(og)
                rospy.loginfo("here2")

    def recv_msg(self, sock):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(sock, 4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return self.recvall(sock, msglen)

    def recvall(self, sock, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = ''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

if __name__ == "__main__":
    SeenmapServer()
