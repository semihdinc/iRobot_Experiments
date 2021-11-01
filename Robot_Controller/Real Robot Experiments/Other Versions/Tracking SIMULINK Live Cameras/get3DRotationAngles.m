function [roll pitch yaw] = get3DRotationAngles(rotMat)
%#eml

% This function get the rotation matrix and computes the euler angles.
% rotMat :(3x3) rotation matrix 
%
% roll  : angle phi
% pitch : angle theta
% yaw   : angle psi

pitch = atan2(-rotMat(3,1),sqrt(rotMat(1,1)^2+rotMat(2,1)^2));
roll = atan2(rotMat(3,2)/cos(pitch),rotMat(3,3)/cos(pitch));
yaw = atan2(rotMat(2,1)/cos(pitch),rotMat(1,1)/cos(pitch));


