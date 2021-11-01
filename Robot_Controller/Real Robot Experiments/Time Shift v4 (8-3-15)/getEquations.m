function [equations, constants] = getEquations(Cam_PDes,S_PDes,Err,M)
% This function takes the 3 desired target points in selected camera 
% and robot space, image errors for 3 points and M transformation matrix.
% Then computes the equation (6x12) for selected camera.
%
% Cam_PDes  :(4x4) Desired 3 target points in Selected Camera Space 
% S_PDes    :(4x4) Desired 3 target points in robot space
% Err       :(2x4) Image Errors of 3 target points of camera 
% M         :(4x4) Transformation matrix (Intrinsic & Extrinsic Parameters)
%
% equations :(6x12) 6 equations (3 Points x 2 errors)
% constants :(6x1) 6 constants for equations

equations = zeros(8,12);
constants = zeros(8,1);

n = zeros(3,4);

j=1;
for i=1:4 %number of points

    n(1,:)=Cam_PDes(3,i)*M(1,:)-Cam_PDes(1,i)*M(3,:);
    n(2,:)=Cam_PDes(3,i)*M(2,:)-Cam_PDes(2,i)*M(3,:);
    n(3,:)=Cam_PDes(3,i)*M(3,:);

    O = n/n(3,4);
    O(3,4) = 0;

    Vx = O(1,1:3)-Err(1,i)*O(3,1:3);
    Vy = O(2,1:3)-Err(2,i)*O(3,1:3);
    
    equations(j,:) = [ Vx.*S_PDes(1,i) Vx.*S_PDes(2,i) Vx.*S_PDes(3,i) Vx ];
    equations(j+1,:) = [ Vy.*S_PDes(1,i) Vy.*S_PDes(2,i) Vy.*S_PDes(3,i) Vy ];
    
    constants(j) = Err(1,i)-O(1,4);
    constants(j+1) = Err(2,i)-O(2,4);
    
    j = j + 2;
end