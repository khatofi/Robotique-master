import time
import pycom
import gc
import struct
from pytrack import Pytrack
from network import LoRa
from L76GNSS import L76GNSS
import sys
import os
# Disable default blue LED blink
pycom.heartbeat(False)
# Pytrack GPS
time.sleep(2)
# enable garbage collector
gc.enable()
# Enable pytrack shield
py = Pytrack()
fw_version = py.read_fw_version()
print("Pytrack firmware version: " + str(fw_version))
counter = 0
while(True):
        #print(input())
        lora_data=input()
        if lora_data=="1":
            print('envoi 1')
            # make the socket blocking
            s.setblocking(True)
            # send some data
            s.send(bytes([1]))
            # make the socket non-blocking
            # (because if there's no data received it will block forever...)
            s.setblocking(False)
            # get any data received (if any...)
            data = s.recv(64)
            print(data)
