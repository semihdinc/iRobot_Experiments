%This script finds the relative camera and intrinsic parametersc
clc;clear;

global C_pDes;

vid = videoinput('winvideo', 1,'MJPG_640x480');
set(vid, 'ReturnedColorSpace', 'RGB');


vid2 = videoinput('winvideo', 2, 'MJPG_640x480');
set(vid2, 'ReturnedColorSpace', 'RGB');


% % Target points in the world frame
% W_P = [   0        0      22.5     9;
%          12.75   -12.75    0       0;
%           2        2       2       22.5;
%           1        1       1       1];
% 
% W_P(1,:) = W_P(1,:) + 33*2.54;      
% W_P(2,:) = W_P(2,:) + 15*2.54; 
% 
% % Desired Robot Pose
% robotPoseDes.x = 0;
% robotPoseDes.y = 15*2.54;
% robotPoseDes.z = 6.5*2.54;
% robotPoseDes.phi = 0;
% robotPoseDes.theta = 0;
% robotPoseDes.psi = 0;

W_P = [-2.5	-1	-2.5	-1	0	-1.75	0	-1.75;
        6.5	4.5	3.5	2	-1.5	-3	-4.5	-6.5;
        3	-3	3	-3	3	-3	3	-3;
        1	1	1	1	1	1	1	1];

    
%Convert to cm    
W_P(1:3,:) = W_P(1:3,:) * 2.54;

%Location in the table
W_P(1,:) = W_P(1,:) + 42*2.54;      
W_P(2,:) = W_P(2,:) + 15*2.54; 
W_P(3,:) = W_P(3,:) + 4.5*2.54;

% Desired Robot Pose
robotPoseDes.x = -2;
robotPoseDes.y = 15*2.54;
robotPoseDes.z = 6.5*2.54;
robotPoseDes.phi = 0;
robotPoseDes.theta = 0;
robotPoseDes.psi = 0;

%Transform from world to desired robot frame
world2robotDes = inv(EulerTrans(robotPoseDes));

%Target points in the desired robot frame
S_PDes = world2robotDes * W_P;

numberOfFrames = 20;
for i=1:numberOfFrames

    leftImage = getsnapshot(vid);
    rightImage = getsnapshot(vid2);
    
%     [L_p, R_p] = getCenter(leftImage,rightImage);

    L_p = getCenter8PointModel(leftImage);
    R_p = getCenter8PointModel(rightImage);
    
    %Optimize x parameters. C_p and S_PDes are constant parameters
    x0L = [3.5 5.5 1 0.05 0.1 0.03 700 700 320 240];
    x0R = [3.5 -6 1 0.05 0.1 0.03 700 700 320 240];
    
    [xsL, errL] = fminsearch(@(x) projectionErrorFunc(x,L_p,S_PDes),x0L);
    subplot(1,2,1);plotTargetPoints(leftImage,L_p,C_pDes);
    [xsR, errR] = fminsearch(@(x) projectionErrorFunc(x,R_p,S_PDes),x0R);
    subplot(1,2,2);plotTargetPoints(rightImage,R_p,C_pDes);
    
    xFinalL(i,:) = [errL xsL];
    xFinalR(i,:) = [errR xsR];
    
    disp([i errL errR]);
end
[~, indL] = sort(xFinalL(:,1));
[~, indR] = sort(xFinalR(:,1));

leftParams = xFinalL(indL(1),2:end);
rightParams = xFinalR(indR(1),2:end);

% save('C:\Users\sd0016\Dropbox\UAH Doktora\Journal Pose Estimation\Matlab Code\Reviews Matlab Code\intMatParams.mat','leftParams','rightParams');

% clear('leftCam','rightCam');


