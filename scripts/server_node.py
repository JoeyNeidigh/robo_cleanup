#!/usr/bin/env python
import rospy
import pickle
import os
from socket import *

class ServerNode():
    def __init__(self):
        rospy.init_node('server_node')
        host = ""
        port = 13000
        self.buf = 1024
        self.addr = (host, port)
        self.UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.UDPSock.bind(self.addr)
        # in server_node
            # have loop that listens to client_node
                # when message received, de-pickle using pickle.loads(string_to_depickle)
                # store in global variable
            # probably create rviz marker from pose of other robot and publish to topic 
        
        while True:
            (data, self.addr) = self.UDPSock.recvfrom(self.buf)
            depickled_str = pickle.loads(data)
            rospy.loginfo(depickled_str)
            if data == "exit":
                break
        self.UDPSock.close()
        os._exit(0)
                
if __name__ == "__main__":
    ServerNode()
