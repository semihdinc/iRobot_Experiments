function retVal = transformImageSpace(S_P, camera)

%Takes the 3D points in camera space and camera object and transform the 3D
%points to the given camera image space. 

% S_P : (3xN) N number of 3D points in Robot space
% camera : Camera object that includes, focal length, principle points,
% skewness coefficient and distortion coefficient. These parameters can be
% calculated by camera calibration procedure.
% PLEASE NOTE: In this version, I did not consider distortion and skewness
%
% p: (2xN) N number of pixel coordinates in the image space. Please note
% that some of the 3D points may not be (regularly) defined in image space
% because some pointd may be projected out of the borders of image plane.

%Tranform object from robot space to camera space
robot2cam = (transformationMatrix(camera.pose))^-1;

%Bu islemi dogrulamadik!!!!
robot2cam = robot2cam([2 3 1 4],:);
robot2cam(1,:) = robot2cam(1,:) * -1;

% C_P = robot2cam * S_P;
% 
% %Project object 3D points to the camera image space
% p(1,:) = camera.fx*(C_P(1,:)./C_P(3,:)) + camera.cx;
% p(2,:) = camera.fy*(C_P(2,:)./C_P(3,:)) + camera.cy;

K = intrinsicMatrix(camera);
M = K * robot2cam;

C_P = M * S_P;

p = zeros(2,size(S_P,2));
p(1,:) = C_P(1,:)./ C_P(3,:);
p(2,:) = C_P(2,:)./ C_P(3,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
retVal.p = p;
retVal.C_P = C_P;
retVal.M = M;
