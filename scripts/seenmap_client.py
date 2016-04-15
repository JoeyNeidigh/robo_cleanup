#!/usr/bin/env python
import socket
import cv2
import pickle
import sys
import struct
import rospy
import map_utils
from nav_msgs.msg import OccupancyGrid
import zlib

class SeenmapClient():
    def __init__(self):
        rospy.init_node('seenmap_client')
        rospy.Subscriber('/seenmap', OccupancyGrid, self.callback, queue_size=1)

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(("134.126.125.125", 13008))
        except Exception as e:
            sock.close()
            rospy.loginfo("SEENMAP_CLIENT ERROR")
            rospy.loginfo(e)
            sys.exit()
        
        rospy.spin()

    def callback(self, og):
        seenmap = map_utils.Map(og)
        message = pickle.dumps(seenmap.grid)
        compressed_message = zlib.compress(message)
        self.send_msg(compressed_message)

    def send_msg(self, msg):
        # Prefix each message with a 4-byte length (network byte order)
        msg = struct.pack('>I', len(msg)) + msg
        try:
            self.sock.sendall(msg)
        except Exception as e:
            sock.close()
            rospy.loginfo("SEENMAP_CLIENT ERROR")
            rospy.loginfo(e)
            sys.exit()

if __name__ == "__main__":
    SeenmapClient()
