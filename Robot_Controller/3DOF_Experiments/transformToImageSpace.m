function [p,P,M] = transformToImageSpace(S_P,Cam_E_S,K)
% This function takes the 3D target points in robot space and transforms 
% them into 2D image space. 
%
% S_P     :(4x4) Desired position of the 3 target points in Robot Space 
% Cam_E_S :(4x4) Transformation Matrix (from robot space to camera space)
% K       :(4x4) Intrinsic Parameters Matrix for specified camera
%
% P     :(4x4) 3 Target points in 3D camera space
% p     :(2x4) 3 Target points in 2D image space

%yani burada koordinat sistemini right hand system haline getiriyoruz.
Cam_E_S = Cam_E_S([2 3 1 4],:);
Cam_E_S(1,:) = Cam_E_S(1,:) * -1;

%transformation matrix
M = K * Cam_E_S;      

%transform into camera space (3D)
P = M * S_P;  

% Project 3D point into 2D image space
p = zeros(2,4);
p(1,:) = P(1,:)./ P(3,:);
p(2,:) = P(2,:)./ P(3,:);
