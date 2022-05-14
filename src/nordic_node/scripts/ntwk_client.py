#!/usr/bin/env python3

import socket
import sys
import traceback


ip = socket.gethostname()
print(ip)
port = 6000

server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_socket.settimeout(3)

# encode string utf-8
def ENC(string):
    return string.encode("utf-8")

# decode string utf-8
def DEC(string):
    return string.decode("utf-8")

# instantiate stocket
try:
    server_socket.connect((ip,port))
except Exception:
    print ("Could not connect to server.")
    print(traceback.format_exc())
    sys.exit(1)

cmd = "hello world"

# send command to server, get back output
try:
    server_socket.send(bytes(ENC(cmd)))
except Exception:
    print ("Failed to send command. Terminating.")
    sys.exit(1)

try:
    print(DEC(server_socket.recv(2048)))

except Exception:
    print("Connection failed.")
    print(traceback.format_exc())
    
finally:
    server_socket.close()
    sys.exit(1)
