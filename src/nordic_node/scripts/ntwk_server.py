#!/usr/bin/env python3

import socket
import subprocess
import traceback
import os
import sys
from threading import Thread
import threading


# create the socket
# AF_INET == ipv4
# SOCK_STREAM == TCP
local_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
hostname = socket.gethostname()
print(hostname)

port = 6000

# encode string utf-8
def ENC(string):
    return string.encode("utf-8")

# decode string utf-8
def DEC(string):
    return string.decode("utf-8")

# threading connection, so other connections can occur
class connection(Thread):
    def __init__(self, socket, address):
        threading.Thread.__init__(self)
        self.socket = socket
        self.address = address
        self.socket.settimeout(.5)
    
    def run(self):
        try:
            # recieve command
            command = self.socket.recv(2048)
            print(DEC(command))

            # send command
            self.socket.send(bytes(ENC("testing")))

        except Exception:
            print ("File transmission failed.")
            print(traceback.format_exc())

        finally:
            self.socket.close()

        self.handled = True

try:

    # bind to current hostname, ie localhost
    local_socket.bind((hostname, port))
    local_socket.settimeout(None)
    local_socket.listen(5)

    client_socket = None
    remote_address = None
    while client_socket is None:
        print("accepting")
        client_socket, remote_address = local_socket.accept()

    thread = connection(client_socket, remote_address)
    thread.start()
finally:
    local_socket.close()
