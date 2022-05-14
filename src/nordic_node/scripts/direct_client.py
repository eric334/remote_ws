#!/usr/bin/env python3

import socket
import sys
import traceback

buffer_size = 8192
#ip = "10.0.1.128"
ip = "10.0.1.128"
port = 6000

class Client():
    def __init__(self):
        self.server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.server_socket.settimeout(3)

        # instantiate stocket
        try:
            self.server_socket.connect((ip,port))
        except Exception:
            print ("Could not connect to server.")
            print(traceback.format_exc())
            self.close_socket()

    def send_data(self, data):
        # send command to server, get back output
        try:
            self.server_socket.send(data)
        except Exception:
            print(traceback.format_exc())
            print ("Failed to send data.")
        

    def recv_data(self):
        try:
            return self.server_socket.recv(buffer_size)
        except Exception:
            print(traceback.format_exc())
            print ("Failed to recieve data.")

    def close_socket(self):
        self.server_socket.close()
        
