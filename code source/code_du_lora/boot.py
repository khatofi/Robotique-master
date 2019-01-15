import socket
import binascii
import struct
import pycom
import time
import os
import machine
from machine import UART
from network import WLAN
from network import LoRa

color_yellow = 0xffff00
color_red = 0xff0000
color_green = 0x00ff00
color_blue = 0x0000ff
color_brown = 0x996633

uart = UART(0, baudrate=115200)
os.dupterm(uart)

# Initialize LoRa in LORAWAN mode.
lora = LoRa(mode=LoRa.LORAWAN)

# create an ABP authentication params
dev_addr = struct.unpack(">l", binascii.unhexlify('F7838A65'))[0] #device address
nwk_swkey = binascii.unhexlify('80F3B0BB0B96F583669080537FFB7B2E') # network session key
app_swkey = binascii.unhexlify('72B95F61FD3A3E5C3435A18695A04692') #application session key
# join a network using ABP (Activation By Personalization)
lora.join(activation=LoRa.ABP, auth=(dev_addr, nwk_swkey, app_swkey))

# create a LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 3)

machine.main('main.py')
