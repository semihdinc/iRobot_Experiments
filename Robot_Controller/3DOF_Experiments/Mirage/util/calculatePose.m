function pose = calculatePose(transMat)

[phi, theta, psi] = get3DRotationAngles(transMat(1:3,1:3));

% pose.tx = transMat(1,4);
% pose.ty = transMat(2,4);
% pose.tz = transMat(3,4);
% pose.phi = phi;
% pose.theta = theta;
% pose.psi = psi;

pose = [transMat(1,4), transMat(2,4), transMat(3,4), phi, theta, psi]';