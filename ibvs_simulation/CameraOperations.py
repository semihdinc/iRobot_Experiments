#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 21 08:58:12 2022

@author: sdinc
"""
import numpy as np
import matplotlib.pyplot as plt


#%%
def plotScene(act_pixel_coord,des_pixel_coord):
    plt.figure()
    plt.scatter(des_pixel_coord[0,:], des_pixel_coord[1,:],c ="blue")
    plt.plot(des_pixel_coord[0,[0,1,2,3,0]], des_pixel_coord[1,[0,1,2,3,0]])
    plt.scatter(act_pixel_coord[0,:], act_pixel_coord[1,:],c ="red")
    plt.plot(act_pixel_coord[0,[0,1,2,3,0]], act_pixel_coord[1,[0,1,2,3,0]])
    plt.xlim(0, 640)
    plt.ylim(0, 480) 

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
def extract_pixel_coordinates(pointCloud,intrinsic_matrix,camPose):
    world_to_cam = euler_trans(camPose)
    
    image_space_coord = np.linalg.multi_dot([intrinsic_matrix, world_to_cam, pointCloud])
    u = image_space_coord[0,:]/image_space_coord[2,:]
    v = image_space_coord[1,:]/image_space_coord[2,:]
    
    pixel_coord = np.vstack([u,v])
    return pixel_coord

#%% Point cloud projection to image plane
def extract_2d_coordinates(pointCloud,camPose):
    world_to_cam = euler_trans(camPose)
    
    image_space_coord = np.linalg.multi_dot([world_to_cam, pointCloud])
    x = image_space_coord[0,:]/image_space_coord[2,:]
    y = image_space_coord[1,:]/image_space_coord[2,:]
    
    image_features = np.vstack([x,y])
    return image_features, image_space_coord[2,:]

#%% Constructs the jacobian matrix from image pixels and depth of each point
def jacobian_matrix_pc(point_2d_coord, z):
    
    x = point_2d_coord[0,:]
    y = point_2d_coord[1,:]
    
    J = np.zeros([len(z)*2, 6])
    
    for i in range(0, len(z)):
        J[2*i,:] = np.array([-1/z[i],   0,       x[i]/z[i],   x[i]*y[i],   -(1+x[i]**2),  y[i]])
        J[2*i+1,:] = np.array([ 0,     -1/z[i],  y[i]/z[i],   1+y[i]**2,   -x[i]*y[i],   -x[i]])

    return J

#%% Constructs the jacobian matrix from image pixels and depth of each point
def jacobian_matrix_pixel(pixel_coord,z):
    
    u = pixel_coord[0,:]
    v = pixel_coord[1,:]
    
    J = np.zeros([len(z)*2, 6])
    
    for i in range(0, len(z)):
        J[2*i,:] = np.array([-1/z[i],   0,       u[i]/z[i],   u[i]*v[i],   -(1+u[i]**2),  v[i]])
        J[2*i+1,:] = np.array([ 0,     -1/z[i],  v[i]/z[i],   1+v[i]**2,   -u[i]*v[i],   -u[i]])

    return J