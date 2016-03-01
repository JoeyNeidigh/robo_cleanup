#!/usr/bin/env python
import rospy
import pickle
import os
from socket import *

class ServerNode():
    def __init__(self):
        host = ""
        port = 13000
        self.buf = 1024
        self.addr = (host, port)
        self.UDPSock = socket(AF_INET, SOCK_DGRAM)
        self.UDPSock.bind(self.addr)
        # in client_node
            # subscribe to odom topic
            # in subscribe callback:
                # pull out just the info we need (pose probably)
                # pickle it using pickle.dumps(obj_to_pickle)
                # store pickle in global variable
            # in main loop:
                # send pickle string through the UDP socket to the server at some interval
        # in server_node
            # have loop that listens to client_node
                # when message received, de-pickle using pickle.loads(string_to_depickle)
                # store in global variable
            # probably create rviz marker from pose of other robot and publish to topic 
        
    def run(self):
        print "Waiting to receive messages..."
        while True:
            (data, self.addr) = self.UDPSock.recvfrom(self.buf)
            print "Received message: " + data
            if data == "exit":
                break
        self.UDPSock.close()
        os._exit(0)
                
if __name__ == "__main__":
    node = ServerNode()
    node.run()
