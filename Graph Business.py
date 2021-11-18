# -*- coding: utf-8 -*-
"""
Created on Wed Nov 17 20:51:47 2021

@author: riley
"""

import matplotlib.pyplot as mpl
import math
import numpy as np

#%% Robot x,y coordinates and the orientation angle theta

qr = np.array([[0.000,50.000,100.000,150.000,200.000,250.000,300.000,350.000,400.000,450.000,500.000,550.000,600.000,650.000,700.000],
              [0.000,122.021,222.943,285.317,298.357,259.808,176.336,62.374,-62.374,-176.336,-259.808,-298.357,-285.317,-222.943,-122.021],
              [0.379,0.411,0.536,0.910,1.828,2.469,2.685,2.755,2.755,2.685,2.469,1.828,0.910,0.536,0.411]])

#%%
#ok so the robot will have an X,Y and THETA.
RobotPose = np.array([[0,2,4],[1,0,1]]) #THIS IS THE X AND Y OF THE ROBOT

Xrobot = [2,4,6,8,6,4,2] #This should always be positive.
Yrobot = [1,2,1,0,-1,0,1] #this can fluctuate based on position of axis

Time = [0,1,2,3,4,5,6] #Time starts at 0 and moves to 11 units<?>

CounterforPoints=['first','second','third','fourth','fifth','sixth','last']

#Path of robot on a positive grid!
#MUST COMMENT THIS OUT TO GET THE OTHERS TO WORK!!!!!!!
fig, ax = mpl.subplots()
ax.plot(Xrobot,Yrobot,linestyle=':',marker='h') #draws the path without arrows.
ax.set(xlabel='Length', ylabel='Height', #both positive values
        title="Path of Robot!")

ax.grid() #draws a grid
ax.set_aspect('equal', 'box') 
ax.arrow(Xrobot[0],Yrobot[0],0.5,0.25,head_width=.1)
#okay this draws an arrow for the first path. x[0],y[0] at a 45DEG angle.

#I get to know the starting point and arguably decide it when charting path.
for increm in Time: #forloop that is Time long. idk? maybe i'm doing it wrong.
    ax.annotate(CounterforPoints[increm],(Xrobot[increm],Yrobot[increm]))
mpl.show() #shows the entire defined plot

#This bit is for X over time! Nothing too fancy. no arrows.
fig, bx = mpl.subplots()
bx.plot(Time,Xrobot,linestyle=':',marker='h')
bx.set(xlabel='Time',ylabel='X Axis',title='X over time')
bx.grid()
mpl.show()

#This is the plot for the Y axis over time!
fig, cx = mpl.subplots()
cx.plot(Time,Yrobot,linestyle=':',marker='h')
cx.set(xlabel='Time',ylabel='Y Axis',title='Y over time')
cx.grid()
mpl.show()

#unsure on how to get Theta.. Like i know the math. since we 
#have a right triangle each time. we will always have the n-1 angle and 90DEG.
#Although it looks like in my example, i ended up with 45 degree variations


