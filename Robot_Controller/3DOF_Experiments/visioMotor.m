function [parameters] = visioMotor(S_PDes,L_p,R_p)
% This function gets the desired positions of 4 3D target points in 
% robot space and 8 2D target points on the image for left and right 
% cameras. Then it computes the t parameters that contains direction 
% information to close desired position
%
% S_PDes :(3x3) Desired position of the 3 target points in Robot Space 
% L_p    :(2x3) Pixel values of the 3 target points on the left image
% R_p    :(2x3) Pixel values of the 3 target points on the right image
%
% parameters :(12x1) 9 Rotation and 3 Translation Parameters. Robot will use
% these t parameters and change its position into desired position

%% Transformation Matrices (Extrinsic Parameters)

% Pose of the left camera with respect to robot frame
LCamPos.x = 1;
LCamPos.y = +1.7;
LCamPos.z = 1;
LCamPos.phi = 0;%-0.062; %yukari asagi
LCamPos.theta = 0;%-0.031; 
LCamPos.psi = 0;

% Pose of the right camera with respect to robot frame
RCamPos.x = 1;
RCamPos.y = -1.7;
RCamPos.z = 1;
RCamPos.phi = 0;%-0.062;
RCamPos.theta = 0;%-0.03;
RCamPos.psi = 0;

L_E_S = inv(EulerTrans(LCamPos));
R_E_S = inv(EulerTrans(RCamPos));

%% Transformation into Camera/Image Space
 
% intrinsic parameters of Left Camera
f1 = 800;   % focal length of the lens (pixels)
f2 = 800;
ox = 320;   % coordinates of the optical axis (pixels)
oy = 240;   % coordinates of the optical axis (pixels)
d  = 0.0;       % radial distortion

L_K = [[f1,d,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

% intrinsic parameters of Right Camera
f1 = 800;   % focal length of the lens (pixels)
f2 = 800;
ox = 320;   % coordinates of the optical axis (pixels)
oy = 240;   % coordinates of the optical axis (pixels)
d  = 0.0;       % radial distortion

R_K = [[f1,d,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

[L_pDes L_PDes L_M] = transformToImageSpace(S_PDes,L_E_S,L_K);
[R_pDes R_PDes R_M] = transformToImageSpace(S_PDes,R_E_S,R_K);

%% Image Error

% figure(3);plot(L_p(1,:),L_p(2,:),'*');
% hold all;plot(L_pDes(1,:),L_pDes(2,:),'*');
% 
% figure(4);plot(R_p(1,:),R_p(2,:),'*');
% hold all;plot(R_pDes(1,:),R_pDes(2,:),'*');

Err_Left = L_p - L_pDes;
Err_Right = R_p - R_pDes;

%% Leftf and Right Camera Equations (2 equations for 1 point)
[Left_Equations Left_Constants] = getEquations(L_PDes,S_PDes,Err_Left,L_M);
[Right_Equations Right_Constants] = getEquations(R_PDes,S_PDes,Err_Right,R_M);

AllEquations = [Left_Equations;Right_Equations];
AllConstants = [Left_Constants;Right_Constants];

parameters = solveEquations(AllEquations,AllConstants);
