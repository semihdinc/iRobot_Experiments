clc;clear;

%%

% Target points in the world frame
W_P = [  -9       -9      13.5     0;
         12.75   -12.75    0       0;
          2        1.75    2      22.5;
          1        1       1       1];

% Desired Robot Pose
robotPoseDes.x = -395;
robotPoseDes.y = 2;
robotPoseDes.z = 45;
robotPoseDes.phi = 0;
robotPoseDes.theta = 0;
robotPoseDes.psi = 0;

%Transform from world to desired robot frame
world2robotDes = inv(EulerTrans(robotPoseDes));

%Target points in the desired robot frame
S_PDes = world2robotDes * W_P;

%##########################################################################
%% Capture Left and Right Images
numberOfFrames = 40;

leftCam = webcam(1);
rightCam = webcam(2);

leftCam.WhiteBalance = 3000;
rightCam.WhiteBalance = 3000;

leftCam.Resolution = '1280x960';
rightCam.Resolution = '1280x960';

%Set a loop that stop after 1 frames of aquisition
for i=1:numberOfFrames

    leftImage = snapshot(leftCam);
    rightImage = snapshot(rightCam);
    
    [L_p, R_p] = sm_getCenter(leftImage,rightImage);
    L_p = orderPoints(L_p); R_p = orderPoints(R_p);
    
    L_p(2,:) = size(leftImage,1) - L_p(2,:);
    R_p(2,:) = size(leftImage,1) - R_p(2,:);
    
    %MIRAGE POSE ESTIMATION
    tic
    errParams = mirage(L_p,R_p,S_PDes);
    toc
    
    poseErrorMat = eye(4,4);
    poseErrorMat(1:3,:) = reshape(errParams,3,4);
    
    world2robot = poseErrorMat*world2robotDes;
    
    robotPoseEst = calculatePose(world2robot);

    poseError(i,:) = [   -robotPoseEst.x  - robotPoseDes.x;
                    -robotPoseEst.y  - robotPoseDes.y;
                    -robotPoseEst.z  - robotPoseDes.z;
                    -robotPoseEst.phi  - robotPoseDes.phi;
                    -robotPoseEst.theta  - robotPoseDes.theta;
                    -robotPoseEst.psi  - robotPoseDes.psi;]';
end

clear('leftCam','rightCam');

%% Detect the target points in images
% [leftCenters, rightCenters] = sm_getCenter(leftImage,rightImage);
% 
% faces = combnk(1:4,2);
% subplot(1,2,1);imshow(leftImage);hold on;
% patch('Faces',faces,'Vertices',leftCenters,'EdgeColor','r');
% plot(leftCenters(:,1),leftCenters(:,2),'ro');
% 
% subplot(1,2,2);imshow(rightImage);hold on;
% patch('Faces',faces,'Vertices',rightCenters,'EdgeColor','r');
% plot(rightCenters(:,1),rightCenters(:,2),'ro');