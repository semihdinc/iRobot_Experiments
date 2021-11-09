# -*- coding: utf-8 -*-
"""
Created on Mon Nov  1 12:55:30 2021

@author: sdinc
"""

import numpy as np
#import matplotlib.pyplot as plt

#%% This is the kinematic model of the diff drive for simulations
def kinematicModel(u,q):
    # Kinematic Model of the diff drive robot
    l = 0.125; 
    theta = q[2];
    
    xdot = l*np.cos(theta)*u[0]
    ydot = l*np.sin(theta)*u[0]
    thetadot = l*u[1]
    
    qdot = [xdot, ydot, thetadot]
    return qdot


#%% Here we define the Sin Wave desired path function ------------------
def desiredPathSineWave(t,totalSim):    
    yWidth = 300
    xWidth = 50

    xr = t*xWidth #robot will move xWidth mm in x direction for every t seconds
    yr = np.sin(2*np.pi*t/totalSim) * yWidth #robot will move between -/+ yWidth mm in y
    
    xr_dot = xWidth
    yr_dot = (2*np.pi/totalSim)*np.cos(2*np.pi*t/totalSim)*yWidth
    
    thetar = np.arctan2(xr_dot,yr_dot);
    
    #desired second derivatives
    xr_ddot = 0;
    yr_ddot = -yWidth*(2*np.pi/totalSim)**2 * np.sin(2*np.pi*t/totalSim);

    #desired theta derivative
    thetar_dot = (yr_ddot - xr_ddot*np.tan(thetar))/(xr_dot*(1+np.tan(thetar)**2));

    vr     = np.sqrt(xr_dot**2+yr_dot**2);
    omegar = thetar_dot;
    
    qr = [xr,yr,thetar,vr,omegar]
    return qr

#%% Here we define the Circular desired path function ------------------
def desiredPathCircle(t,totalSim):
    x0 = 500 #mm radius circular motion
    y0 = 0
    
    thetar = t*(2*np.pi/totalSim)
    xr = np.cos(thetar)*x0 - np.sin(thetar)*y0
    yr = np.sin(thetar)*x0 + np.cos(thetar)*y0
    
    thetar_dot = 2*np.pi/totalSim
    xr_dot = thetar_dot*-np.sin(thetar)*x0 - thetar_dot*np.cos(thetar)*y0
    yr_dot = thetar_dot*np.cos(thetar)*x0 - thetar_dot*np.sin(thetar)*y0
    
    omega  = thetar_dot
    vr     = np.sqrt(xr_dot*xr_dot+yr_dot*yr_dot)
    
    qr = [xr,yr,thetar,vr,omega]
    return qr

#%% Here we create the desired path for simulation -----------

#we create a simulation of 36 seconds.
#simulation runs in every 0.1 seconds.
totalTime = 20 #secs
simulation_time = np.arange(0,totalTime,1)

#desired pose (trajectory) of the vehicle in every time instant
qr = np.zeros([5,np.size(simulation_time)])
for idx, t in enumerate(simulation_time):
    #qr[:,idx] = desiredPathCircle(t,totalTime);
    qr[:,idx] = desiredPathSineWave(t,totalTime);



#%% --- Simulations of the Robot -------------------------------

# qSave = np.zeros([3,np.size(simulation_time)])
# q = [0,0,0.56]
# for idx, t in enumerate(simulation_time):
#     ur = [qr[3,idx], qr[4,idx]]
#     qdot = kinematicModel(ur,q)
    
#     q = q + qdot
#     qSave [:,idx]= q

#%% --- Actual Robot Experiments -------------------------------

# from  pycreate2 import Create2
# import time

# bot = Create2("COM3")

# bot.start() # Start the Create 2
# bot.safe()

# l = 235     #Wheel base of vehicle in mm

# #we calculate velocity instructions of left and right wheel
# # Vr/Vl cm must be travelled in every 0.1 seconds
# for idx, val in enumerate(simulation_time):
#     v = qr[3,idx]
#     omega = qr[4,idx]
    
#     Vl = v - omega*l/2 
#     Vr = v + omega*l/2 

#     print([idx, val, Vl, Vr])
    
#     bot.drive_direct(int(Vr), int(Vl))
#     time.sleep(1) #robot will go V mm for 0.9 sec

# # Stop the bot
# bot.drive_stop()

# bot.power() #go back to passive mode
# bot.stop() #Puts the Create 2 into OFF mode.
# bot.close() # Close the connection
