function [L_p, R_p] = simulateRealRobot(P,robot)
    %Simulates real cameras and generates the actual pixel coordinates of
    %the object in Left and Right image spaces.
    
    %P: 3D object points in world space
    
    %Transform object from world to robot space
    world2robot = inv(transformationMatrix(robot.pose));
    S_P = world2robot * P;
             
    %Project desired points into image space
    L_retVal = transformImageSpace(S_P,robot.leftCamera);
    R_retVal = transformImageSpace(S_P,robot.rightCamera);
    
    L_p = L_retVal.p;
    R_p = R_retVal.p;
end