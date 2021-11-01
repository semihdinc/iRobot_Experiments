function pose = calculatePose(transMat)

    [phi, theta, psi] = get3DRotationAngles(transMat(1:3,1:3));

    pose.x = transMat(1,4);
    pose.y = transMat(2,4);
    pose.z = transMat(3,4);
    pose.phi = phi;
    pose.theta = theta;
    pose.psi = psi;

end
