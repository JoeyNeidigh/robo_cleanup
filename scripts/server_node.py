#!/usr/bin/env python
import rospy
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
        # run
        
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
