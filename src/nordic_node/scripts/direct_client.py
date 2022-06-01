#!/usr/bin/env python3

import socket
import sys
import traceback
import binascii
import struct

chunk_size = 512

class Client():
    def __init__(self, port):
        ip = "10.0.1.128"
        self.server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.server_socket.settimeout(10)

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
            size = len(data)
            chunks = [data[i:i+chunk_size] for i in range(0, size, chunk_size)]
            size_bytes = struct.pack(">i",size)
            self.server_socket.send(size_bytes)
            for chunk in chunks:
                self.server_socket.send(chunk)
                #print(chunk)
        except Exception:
            print(traceback.format_exc())
            print ("Failed to send data.")
        

    def recv_data(self):
        try:
            size = struct.unpack(">i",self.server_socket.recv(4))[0]
        
            message = b''
            remaining_size = size
            while len(message) < size:
                recv_size = chunk_size
                if remaining_size < chunk_size:
                    recv_size = remaining_size
                data = self.server_socket.recv(chunk_size)
                message += data

            message = message[:size]
            return message
        except Exception:
            print(traceback.format_exc())
            print ("Failed to recieve data.")

    def close_socket(self):
        self.server_socket.close()


# if __name__ == '__main__':
#     client = Client(6000)
#     data = b'What the fuck did you just fucking say about me, you little bitch? Ill have you know I graduated top of my class in the Navy Seals, and Ive been involved in numerous secret raids on Al-Quaeda, and I have over 300 confirmed kills. I am trained in gorilla warfare and Im the top sniper in the entire US armed forces. You are nothing to me but just another target. I will wipe you the fuck out with precision the likes of which has never been seen before on this Earth, mark my fucking words. You think you can get away with saying that shit to me over the Internet? Think again, fucker. As we speak I am contacting my secret network of spies across the USA and your IP is being traced right now so you better prepare for the storm, maggot. The storm that wipes out the pathetic little thing you call your life. Youre fucking dead, kid. I can be anywhere, anytime, and I can kill you in over seven hundred ways, and thats just with my bare hands. Not only am I extensively trained in unarmed combat, but I have access to the entire arsenal of the United States Marine Corps and I will use it to its full extent to wipe your miserable ass off the face of the continent, you little shit. If only you could have known what unholy retribution your little clever comment was about to bring down upon you, maybe you would have held your fucking tongue. But you couldnt, you didnt, and now youre paying the price, you goddamn idiot. I will shit fury all over you and you will drown in it. Youre fucking dead, kiddo.'
    
#     client.send_data(data)
#     print(binascii.hexlify(data))
#     print(str(len(data)))
#     client.close_socket()
        
