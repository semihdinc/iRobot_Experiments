# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 10:47:32 2022

@author: sdinc
"""
import numpy as np
import matplotlib.pyplot as plt

from CameraOperations import extract_2d_coordinates
from CameraOperations import extract_pixel_coordinates
from CameraOperations import jacobian_matrix_pc
from CameraOperations import plotScene
from CameraOperations import plotPoseScene
from CameraOperations import kinematicModel

#%% Intrinsic Camera Parameters

f,ox,oy,d = [700,320,240,0]
K = np.array([[f,d,ox,0],[0,f,oy,0],[0,0,1,0],[0,0,0,1]])

#%%
#initial (q0) and desired (qr) pose of the vehicle
#units are considered in cm
q0 = np.array([100, 30, 0, 0, np.pi/6, 0])
qr = np.array([ 0, 30, 100, 0, 0, 0])


pointCloud = np.array([[-10,20,200,1],
                       [ 10,20,200,1],
                       [ 10,40,200,1],
                       [-10,40,200,1]]).T

#initial pixel coordinates of pointcloud for actual and desired pose
act_pixel_coord = extract_pixel_coordinates(pointCloud, K, q0)
des_pixel_coord = extract_pixel_coordinates(pointCloud, K, qr)

plotScene(act_pixel_coord, des_pixel_coord)


#%%
mse = 10 #mean squared pixel error

q = q0
qSave = np.array([q0[2],q0[0],q0[4]]).reshape((3,1)) 
#plotPoseScene(pp)

#we run this loop until the 2d projection error is small enough
#while mse > 1e-05:
for i in range(0, 30):
    
    #We are projecting 3d points into 2d in image space, but not in pixel space
    act_2d_coord,z = extract_2d_coordinates(pointCloud,q)
    des_2d_coord,z = extract_2d_coordinates(pointCloud,qr)

    #Jacobian matrix is constructed for 2d image space.
    J = jacobian_matrix_pc(act_2d_coord,z)
    inv_j = np.linalg.pinv(J)

    point_2d_err = (act_2d_coord - des_2d_coord).T
    qdot = -0.125 * np.matmul(inv_j, point_2d_err.flatten())

    vr     = np.sqrt(qdot[2]**2+qdot[0]**2)
    omegar = qdot[4]
    u = np.array([vr,omegar])

    #kinematic model. u is 
    #xdot, zdot, thetadot = kinematicModel(u, q[4])
    l = 1
    theta = q[4]
    xdot = l*np.cos(theta)*u[0]
    zdot = l*np.sin(theta)*u[0]
    thetadot = l*u[1]

    #update the pose of the camera    
    #q = q - np.array([xdot,0,zdot,0,thetadot,0])
    q = q - qdot


    #calculate pixel coordinates from new pose and plot
    act_pixel_coord = extract_pixel_coordinates(pointCloud, K, q)
    #plotScene(act_pixel_coord, des_pixel_coord)
    
    mse = np.square(point_2d_err).mean()
    #print(mse)
    
    q_2d = np.array([q[2],q[0],q[4]]).reshape((3,1))
    qSave = np.append(qSave,q_2d,axis=1)

#%% 

plotPoseScene(qSave)
plt.scatter(pointCloud[2,:],pointCloud[0,:])
plt.gca().invert_yaxis()
plt.axis('equal')
plt.xlabel("Z")
plt.ylabel("X")