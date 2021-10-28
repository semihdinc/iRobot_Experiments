# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 14:01:14 2021

@author: sdinc
"""

import serial
import time

# Open a serial connection to Roomba
ser = serial.Serial(port='COM3', baudrate=115200)

# Assuming the robot is awake, start safe mode so we can hack.
ser.write(('\x83').encode())
time.sleep(.1)

# Program a five-note start song into Roomba.
song = ('\x8c\x01\x05C\x10H\x18J\x08L\x10O\x20').encode()
ser.write(song)

# Play the song we just programmed.
ser.write(('\x8d\x01').encode())
time.sleep(1.6) # wait for the song to complete

# Leave the Roomba in passive mode; this allows it to keep
#  running Roomba behaviors while we wait for more commands.
ser.write(('\x80').encode())

# Close the serial port; we're done for now.
ser.close()