# -*- coding: utf-8 -*-
"""
Created on Mon Nov  1 12:55:30 2021

@author: sdinc
"""

import numpy as np

#%% Here we define the Circular desired path function ------------------

def desiredPathCircle(t,totalSim):
    x0 = 300 #mm radius circular motion
    y0 = 0
    
    thetar = t*(2*np.pi/totalSim)
    xr = np.cos(thetar)*x0 - np.sin(thetar)*y0
    yr = np.sin(thetar)*x0 + np.cos(thetar)*y0
    
    thetar_dot = 2*np.pi/totalSim
    xr_dot = thetar_dot*-np.sin(thetar) - thetar_dot*np.cos(thetar)
    yr_dot = thetar_dot*np.cos(thetar) - thetar_dot*np.sin(thetar)
    
    omega  = thetar_dot
    vr     = np.sqrt(xr_dot*xr_dot+yr_dot*yr_dot)
    
    qr = [xr,yr,thetar,vr,omega]
    return qr

#%% Here we define the Sin Wave desired path function ------------------

def desiredPathSineWave(t):    
    thetar = t*(2*np.pi/36)

    xr = t*5 #robot will move 0.5 cm in x direction for every 0.1 seconds
    yr = np.sin(thetar) * 20 #robot will move between -20 cm and 20 cm in y
    
    qr = [xr,yr,thetar]
    return qr

#%% Here we create the desired path for simulation -----------

#we create a simulation of 36 seconds.
#simulation runs in every 0.1 seconds.
totalTime = 18 #secs
simulation_time = np.arange(0,totalTime,1)

#desired pose (trajectory) of the vehicle in every time instant
qr = np.zeros([5,np.size(simulation_time)])
for idx, t in enumerate(simulation_time):
    qr[:,idx] = desiredPathCircle(t,totalTime);

#%% -----------------------------------------------------------
from  pycreate2 import Create2
import time

bot = Create2("COM3")

bot.start() # Start the Create 2
bot.safe()

l = 235     #Wheel base of vehicle in mm
R = 300     #Distance to the rotational center in mm

#we calculate velocity instructions of left and right wheel
# Vr/Vl cm must be travelled in every 0.1 seconds
for idx, val in enumerate(simulation_time[1:],1):
    #Vr = 
    
    Vr = (R + l/2)*(qr[2,idx]-qr[2,idx-1]) 
    Vl = (R - l/2)*(qr[2,idx]-qr[2,idx-1])
    print([idx, val, Vl, Vr])
    
    bot.drive_direct(int(Vr), int(Vl))
    time.sleep(1) #robot will go V mm for 0.9 sec

# Stop the bot
bot.drive_stop()

bot.power() #go back to passive mode
bot.stop() #Puts the Create 2 into OFF mode.
bot.close() # Close the connection
