clc;clear;

% Target points in the world frame
W_P = [   0        0      22.5     9;
         12.75   -12.75    0       0;
          2        2       2      22.5;
          1        1       1       1];

% W_P(1,:) = W_P(1,:) - 213;      
% W_P(2,:) = W_P(2,:) + 30.48; 

%##########################################################################
%% Capture Left and Right Images
numberOfFrames = 20;

leftCam = webcam(1);
rightCam = webcam(2);

leftCam.WhiteBalance = 3000;
rightCam.WhiteBalance = 3000;

leftCam.Resolution = '640x480';%'1280x960';
rightCam.Resolution = '640x480';%'1280x960';

leftImage = snapshot(leftCam);
rightImage = snapshot(rightCam);
[L_p, R_p] = getCenter(leftImage,rightImage);

ti = 6/numberOfFrames;
t = 0:ti:6;

for i=1:numberOfFrames

    [qr, ur] = desired(t(1));
    qr = qr*100;
    
    % Desired Robot Pose
    robotPoseDes.x = qr(1);
    robotPoseDes.y = qr(2);
    robotPoseDes.z = qr(3);
    robotPoseDes.phi = qr(4);
    robotPoseDes.theta = qr(5);
    robotPoseDes.psi = qr(6);

    %Transform from world to desired robot frame
    world2robotDes = inv(EulerTrans(robotPoseDes));

    %Target points in the desired robot frame
    S_PDes = world2robotDes * W_P;
    
    %Capture Images
    leftImage = snapshot(leftCam);
    rightImage = snapshot(rightCam);
    
%     [L_p, R_p] = getCenterV3(leftImage,rightImage, L_p, R_p);
    [L_p, R_p] = getCenter(leftImage,rightImage);
    
    %MIRAGE POSE ESTIMATION
    errParams = mirage(L_p,R_p,S_PDes);
    
    poseErrorMat = eye(4,4);
    poseErrorMat(1:3,:) = reshape(errParams,3,4);
        
    world2robot = poseErrorMat*world2robotDes;
    
    robotPoseEst = calculatePose((world2robot));
   
    poseError(i,:) = [  -robotPoseEst.x  - robotPoseDes.x;
                        -robotPoseEst.y  - robotPoseDes.y;
                        -robotPoseEst.z  - robotPoseDes.z;
                        -robotPoseEst.phi  - robotPoseDes.phi;
                        -robotPoseEst.theta  - robotPoseDes.theta;
                        -robotPoseEst.psi  - robotPoseDes.psi]';
    
    pose(i,:) = [  -robotPoseEst.x;
                    -robotPoseEst.y;
                    -robotPoseEst.z;
                    -robotPoseEst.phi;
                    -robotPoseEst.theta;
                    -robotPoseEst.psi]';
                    
%     disp([poseError(i,1:3) rad2deg(poseError(i,4:6))]);
%     plot(poseError(:,1:3));
%     drawnow;
    toc
end
% mean(poseError)
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