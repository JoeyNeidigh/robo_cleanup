#!/usr/bin/env python
import socket
import cv2
import pickle
import sys
import struct
import rospy
import map_utils
from nav_msgs.msg import OccupancyGrid

class SeenmapClient():
    def __init__(self):
        rospy.init_node('seenmap_server_a')
        seenmap_pub = rospy.Piblisher('/seenmap_a', OccupancyGrid, queue_size=5)

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

        seenmap = map_utils.Map(OccupancyGrid())

        while True:
            message = recv_msg(sock)
            seenmap.grid = pickle.loads(message)
            og = seenmap.to_message()
            seenmap_pub.publish(og)

    def recv_msg(self, sock):
        # Read message length and unpack it into an integer
        raw_msglen = recvall(sock, 4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return recvall(sock, msglen)
