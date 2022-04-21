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

#%% Intrinsic Camera Parameters

f,ox,oy,d = [700,320,240,0]
K = np.array([[f,d,ox,0],[0,f,oy,0],[0,0,1,0],[0,0,0,1]])

#%%
#initial (q0) and desired (qr) pose of the vehicle
#units are considered in cm
q0 = np.array([40, 30, 0, 0, 0, 0])
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
q = q0
qSave = q0
mse = 10

pp = np.zeros([3,1])
pp[0] = q0[2]
pp[1] = -q0[0]
pp[2] = q0[4]
#plotPoseScene(pp)

#we run this loop until the 2d projection error is small enough
while mse > 1e-05:
    
    #We are projecting 3d points into 2d in image space, but not in pixel space
    act_2d_coord,z = extract_2d_coordinates(pointCloud,q)
    des_2d_coord,z = extract_2d_coordinates(pointCloud,qr)

    #Jacobian matrix is constructed for 2d image space.
    J = jacobian_matrix_pc(act_2d_coord,z)
    inv_j = np.linalg.pinv(J)

    point_2d_err = (act_2d_coord - des_2d_coord).T
    qdot = -0.125 * np.matmul(inv_j, point_2d_err.flatten())

    #update the pose of the camera    
    q = q - qdot

    #calculate pixel coordinates from new pose and plot
    act_pixel_coord = extract_pixel_coordinates(pointCloud, K, q)
    #plotScene(act_pixel_coord, des_pixel_coord)
    
    mse = np.square(point_2d_err).mean()
    #print(mse)
    
    pr = np.zeros([3,1])
    pr[0] = q[2]
    pr[1] = -q[0]
    pr[2] = q[4]
    pp = np.append(pp,pr,axis=1)

#%% 

plotPoseScene(pp)
plt.scatter(pointCloud[2,:],pointCloud[0,:])
plt.axis('equal')
plt.xlabel("Z")
plt.ylabel("X")