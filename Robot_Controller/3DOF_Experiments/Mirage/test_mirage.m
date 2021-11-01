%Test apem() function
clear; clc;
%% Define the target object (in object space)
PO(:,1)=[8/sqrt(3),  0,   0,         1];
PO(:,2)=[0,         -4,  -4/sqrt(3), 1];
PO(:,3)=[0,          4,  -4/sqrt(3), 1];
PO(:,4)=[4/sqrt(3),          0,   8/sqrt(3), 1];

objectPose = newPose(0,0,0,0,0,0);
object2world = transformationMatrix(objectPose);
P = object2world * PO;

% objectPose = newPose(3,4,5,0,pi/6,0);
% object2world = transformationMatrix(objectPose);
% P2 = object2world * PO;
% 
% P = [P P2];

%% Define Robot with its cameras

%define camera Poses with respect to the robot
leftCamPose = newPose(2,1,1,0,0,-pi/18);
rightCamPose = newPose(2,-1,1,0,0,pi/18);

%define cameras 
leftCamera = newCamera(leftCamPose,800,800,320,240,0,0);
rightCamera = newCamera(rightCamPose,800,800,320,240,0,0);

%Define Robot
robotPose = newPose(-32,-2,5,0,0,0);
robot = newRobot(robotPose, leftCamera, rightCamera);

robotPoseDes = newPose(-30,5,0,0,0,pi/3);
robotDes = newRobot(robotPoseDes, leftCamera, rightCamera);

world2robotDes = inv(transformationMatrix(robotPoseDes));
S_PDes = world2robotDes * P;

[L_p, R_p] = simulateRealRobot(P,robot);
[L_pDes, R_pDes] = simulateRealRobot(P,robotDes);

%% Add Noise
% L_p = L_p + randi([-1, 1],2,4);
% R_p = R_p + randi([-1, 1],2,4);

%% Analytically calculate the pose error
errParams = mirage(robot, S_PDes, L_p, R_p)';

%Calculate 6DOF pose errors
poseError = eye(4,4);
poseError(1:3,:) = reshape(errParams,3,4);

robot2world = poseError*world2robotDes;
robotPose2 = calculatePose(robot2world);

%% Visualize the Scene

figure;
subplot(1,2,1);
drawImage(L_p,[1 0 0]);
hold all;
drawImage(L_pDes,[0 1 0]);
%xlim([0 640]); ylim([0 480]);

subplot(1,2,2);
drawImage(R_p,[1 0 0]);
hold all;
drawImage(R_pDes,[0 1 0]);
%xlim([0 640]); ylim([0 480]);

figure;
drawRobot(robot); hold on;
drawRobot(robotDes,1.4);hold on;
drawObject(P);axis equal;



