# https://github.com/NordicSemiconductor/nRF-Sniffer-for-802.15.4/blob/master/nrf802154_sniffer/nrf802154_sniffer.py

#!/usr/bin/env python
import sys
import os
import time
from serial import Serial, serialutil

dev = '/dev/ttyACM0'
baud = 115200

serial = Serial(dev, timeout=1, baudrate=baud)
serial.close()
serial.open()

serial.write("t".encode())

while True:
    bytesToRead = serial.inWaiting()
    data = serial.read(bytesToRead)
    if (bytesToRead > 0):
        print(bytesToRead)
        print(data)