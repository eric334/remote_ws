# https://github.com/NordicSemiconductor/nRF-Sniffer-for-802.15.4/blob/master/nrf802154_sniffer/nrf802154_sniffer.py

#!/usr/bin/env python
import sys
import os
import time
from serial import Serial, serialutil

dev = '/dev/ttyUSB0'
baud = 115200

serial = Serial(dev, timeout=1, baudrate=baud, bytesize=8, parity='N', stopbits=1)
serial.open()

serial.write("test\r".encode())

time.sleep(0.1)
serial.close()

