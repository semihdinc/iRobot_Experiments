# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 10:47:32 2022

@author: sdinc
"""
import numpy as np
import matplotlib.pyplot as plt

#%% Intrinsic Camera Parameters

f = 700
ox = 320
oy = 240
d = 0
intrinsic_matrix = np.array([[f,d,ox,0],[0,f,oy,0],[0,0,1,0],[0,0,0,1]])

#%% Euler Transformation from Pose to Matrix
def euler_trans(pose):
    Dx,Dy,Dz,phi,theta,psi = pose

    Rz=np.array([[np.cos(psi),-np.sin(psi),0,0],[np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
    Ry=np.array([[np.cos(theta),0,np.sin(theta),0],[0,1,0,0],[-np.sin(theta),0,np.cos(theta),0],[0,0,0,1]])
    Rx=np.array([[1,0,0,0],[0,np.cos(phi),-np.sin(phi),0],[0,np.sin(phi),np.cos(phi),0],[0,0,0,1]])
    Tr=np.array([[1,0,0,Dx],[0,1,0,Dy],[0,0,1,Dz],[0,0,0,1]])

    trans_matrix = np.linalg.multi_dot([Tr, Rx, Ry, Rz])
    return trans_matrix

#%% Point cloud projection to image plane
def project_to_image_plane(pointCloud,camPose):
    world_to_cam = euler_trans(camPose)
    
    image_space_coord = np.linalg.multi_dot([intrinsic_matrix, world_to_cam, pointCloud])
    u = image_space_coord[0,:]/image_space_coord[2,:]
    v = image_space_coord[1,:]/image_space_coord[2,:]
    
    pixel_coord = np.vstack([u,v])
    return pixel_coord

#%% Point cloud projection to image plane
def extract_2d_features(pointCloud,camPose):
    world_to_cam = euler_trans(camPose)
    
    image_space_coord = np.linalg.multi_dot([world_to_cam, pointCloud])
    x = image_space_coord[0,:]/image_space_coord[2,:]
    y = image_space_coord[1,:]/image_space_coord[2,:]
    
    image_features = np.vstack([x,y])
    return image_features

#%% Constructs the jacobian matrix from image pixels and depth of each point
def jacobian_matrix(pointCloud):
    
    u = pointCloud[0,:]
    v = pointCloud[1,:]
    z = pointCloud[2,:]
    
    J = np.zeros([len(z)*2, 6])
    
    for i in range(0, len(z)):
        J[2*i,:] = np.array([-1/z[i],   0,       u[i]/z[i],   u[i]*v[i],   -(1+u[i]**2),  v[i]])
        J[2*i+1,:] = np.array([ 0,     -1/z[i],  v[i]/z[i],   1+v[i]**2,   -u[i]*v[i],   -u[i]])

    return J

#%% This is the kinematic model of the diff drive for simulations
def kinematicModel(u,q):
    # Kinematic Model of the diff drive robot
    l = 1; 
    theta = q[2];
    
    xdot = l*np.cos(theta)*u[0]
    ydot = l*np.sin(theta)*u[0]
    thetadot = l*u[1]
    
    qdot = np.array([xdot, ydot, thetadot])
    return qdot

#%%
#initial (q0) and desired (qr) pose of the vehicle
q0 = np.array([5, 10, 50, 0, 0, 0])
qr = np.array([0, 0, 10, 0, 0, 0])

pointCloud = np.array([[-1,-1,1,1],
                       [-1, 1,1,1],
                       [ 1, 1,1,1],
                       [ 1,-1,1,1]]).T

actual_pixel_coord = project_to_image_plane(pointCloud, q0)
des_pixel_coord = project_to_image_plane(pointCloud, qr)

plt.figure(0)
plt.scatter(des_pixel_coord[0,:], des_pixel_coord[1,:],c ="blue")
plt.plot(des_pixel_coord[0,[0,1,2,3,0]], des_pixel_coord[1,[0,1,2,3,0]])
plt.scatter(actual_pixel_coord[0,:], actual_pixel_coord[1,:],c ="red")
plt.plot(actual_pixel_coord[0,[0,1,2,3,0]], actual_pixel_coord[1,[0,1,2,3,0]])
plt.xlim(0, 2*ox)
plt.ylim(0, 2*oy) 

#%%
J = jacobian_matrix(pointCloud)
inv_j = np.linalg.pinv(J)

actual_features = extract_2d_features(pointCloud, q0)
desired_features = extract_2d_features(pointCloud, qr)

pixel_err = (actual_features - desired_features).T
qdot = np.matmul(inv_j, pixel_err.flatten())
    
q1 = q0 + qdot

#%%

actual_pixel_coord = project_to_image_plane(pointCloud, q1)

plt.figure(1)
plt.scatter(des_pixel_coord[0,:], des_pixel_coord[1,:],c ="blue")
plt.plot(des_pixel_coord[0,[0,1,2,3,0]], des_pixel_coord[1,[0,1,2,3,0]])
plt.scatter(actual_pixel_coord[0,:], actual_pixel_coord[1,:],c ="red")
plt.plot(actual_pixel_coord[0,[0,1,2,3,0]], actual_pixel_coord[1,[0,1,2,3,0]])
plt.xlim(0, 2*ox)
plt.ylim(0, 2*oy) 



