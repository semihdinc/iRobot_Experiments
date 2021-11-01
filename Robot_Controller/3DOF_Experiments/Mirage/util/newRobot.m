function robot = newRobot(pose,leftCamera,rightCamera)

    %pose : robot pose in world space
    %leftCamera : left camera object
    %rightCamera : right camera object
    
    robot.pose = pose;
    robot.leftCamera = leftCamera;
    robot.rightCamera = rightCamera;
end