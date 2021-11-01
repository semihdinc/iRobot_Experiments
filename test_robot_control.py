# -*- coding: utf-8 -*-
"""
Created on Mon Nov  1 12:55:30 2021

@author: sdinc
"""

import numpy as np

#%% Here we define the desired path function ------------------

def desiredPathCircle(t):
    x0 = 100 #100 cm radius circular motion
    y0 = 0
    
    thetar = t*(2*np.pi/36)

    xr = np.cos(thetar)*x0 - np.sin(thetar)*y0
    yr = np.sin(thetar)*x0 + np.cos(thetar)*y0
    
    qr = [xr,yr,thetar]
    return qr


#%% Here we create the desired path for simulation -----------

#we create a simulation of 36 seconds.
#simulation runs in every 0.1 seconds.
simulation_time = np.arange(0,36,0.1)

#desired pose (trajectory) of the vehicle in every time instant
qr = np.zeros([3,np.size(simulation_time)])
for idx, t in enumerate(simulation_time):
    qr[:,idx] = desiredPathCircle(t);

#%% -----------------------------------------------------------
from  pycreate2 import Create2
import time

bot = Create2("COM3")

bot.start() # Start the Create 2
bot.safe()

l = 23.5    #Wheel base of vehicle = 23.5 cm
R = 50      #Distance to the rotational center in cm

#we calculate velocity instructions of left and right wheel
# Vr/Vl cm must be travelled in every 0.1 seconds
for idx, val in enumerate(simulation_time,1):
    Vr = (R + l/2)*(qr[2,idx-1]-qr[2,idx-2]) 
    Vl = (R - l/2)*(qr[2,idx-1]-qr[2,idx-2])
    bot.drive_direct(10*Vr, 10*Vl)
    time.sleep(0.1) #robot will go 10*V mm for 0.1 sec

# Stop the bot
bot.drive_stop()

bot.power() #go back to passive mode
bot.stop() #Puts the Create 2 into OFF mode.
bot.close() # Close the connection
