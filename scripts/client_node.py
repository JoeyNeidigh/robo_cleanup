#!/usr/bin/env python
import rospy
import os
from socket import *

class ClientNode():
    def __init__(self):
        host = "127.0.0.1" # ip of server
        port = 13000
        self.addr = (host, port)
        self.UDPSock = socket(AF_INET, SOCK_DGRAM)
        
    def run(self):
        while True:
            data = raw_input("Enter message to send or type 'exit': ")
            self.UDPSock.sendto(data, self.addr)
            if data == "exit":
                break
        self.UDPSock.close()
        os._exit(0)
        
if __name__ == "__main__":
    node = ClientNode()
    node.run()
