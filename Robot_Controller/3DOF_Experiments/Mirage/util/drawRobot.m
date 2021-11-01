function drawRobot(varargin)
    
    if(nargin == 1)
        robot = varargin{1};
        c = 1;
    else
        robot = varargin{1};
        c = varargin{2};
    end
    
    robot2world = transformationMatrix(robot.pose);
    
    %% Robot Center
    
    p1x = 0;
    p1y = 0;
    p1z = 0;

    %% Box shape
    deltaX = 2;
    deltaY = 2;
    deltaZ = 2;

    PS(:,1) = [p1x-deltaX, p1y-deltaY, p1z-deltaZ,1];
    PS(:,2) = [p1x+deltaX, p1y-deltaY, p1z-deltaZ,1];
    PS(:,3) = [p1x+deltaX, p1y-deltaY, p1z+deltaZ,1];
    PS(:,4) = [p1x-deltaX, p1y-deltaY, p1z+deltaZ,1];

    PS(:,5) = [p1x-deltaX, p1y+deltaY, p1z-deltaZ,1];
    PS(:,6) = [p1x+deltaX, p1y+deltaY, p1z-deltaZ,1];
    PS(:,7) = [p1x+deltaX, p1y+deltaY, p1z+deltaZ,1];
    PS(:,8) = [p1x-deltaX, p1y+deltaY, p1z+deltaZ,1];
    
    %Camera Frustum
    P2(:,1) = [p1x, p1y, p1z,1];
    P2(:,2) = [p1x+deltaX/2, p1y-deltaY/3, p1z-deltaZ/3,1];
    P2(:,3) = [p1x+deltaX/2, p1y-deltaY/3, p1z+deltaZ/3,1];
    P2(:,4) = [p1x+deltaX/2, p1y+deltaY/3, p1z+deltaZ/3,1];
    P2(:,5) = [p1x+deltaX/2, p1y+deltaY/3, p1z-deltaZ/3,1];
    
    left2Robot = transformationMatrix(robot.leftCamera.pose);
    right2Robot = transformationMatrix(robot.rightCamera.pose);
    
    PL = left2Robot * P2;
    PR = right2Robot * P2;
    %%
    
    PS = [PS PL PR];
    PW = robot2world * PS; %All points in world coordinate system.
    
    %%   
    boxFaces = [1 2 3 4; 2 6 7 3; 5 6 7 8; 1 5 8 4; 1 2 6 5; 4 3 7 8];
    patch('Vertices',PW(1:3,:)','Faces',boxFaces,'FaceColor',c*[0.7 0.7 0.7]);
    
    camFrustFaces = [1 2 5; 1 2 3; 1 3 4; 1 4 5];
    patch('Vertices',PW(1:3,9:13)','Faces',camFrustFaces,'FaceColor',c*[0.7 0.4 0.4]);
    patch('Vertices',PW(1:3,14:18)','Faces',camFrustFaces,'FaceColor',c*[0.4 0.4 0.7]);
    
end