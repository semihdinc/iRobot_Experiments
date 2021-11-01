function drawCamera(varargin)
    %Draws a simple camera to the scene using given pose parameters
   
if(nargin == 1)
    pose = varargin(1);    
    tx = pose.tx;
    ty = pose.ty;
    tz = pose.tz;
    phi = pose.phi;
    theta = pose.theta;
    psi = pose.psi;
elseif(nargin == 6)
    tx = varargin{1};
    ty = varargin{2};
    tz = varargin{3};
    phi = varargin{4};
    theta = varargin{5};
    psi = varargin{6};
end
    
    
    %% Origin Of the camera
    
    p1x = 0;
    p1y = 0;
    p1z = 0;

    %% Box shape
    deltaX = 0.5;
    deltaY = 0.5;
    deltaZ = 0.5;

    P(:,1) = [p1x-deltaX, p1y-deltaY, p1z-deltaZ,1];
    P(:,2) = [p1x+deltaX, p1y-deltaY, p1z-deltaZ,1];
    P(:,3) = [p1x+deltaX, p1y-deltaY, p1z+deltaZ,1];
    P(:,4) = [p1x-deltaX, p1y-deltaY, p1z+deltaZ,1];

    P(:,5) = [p1x-deltaX, p1y+deltaY, p1z-deltaZ,1];
    P(:,6) = [p1x+deltaX, p1y+deltaY, p1z-deltaZ,1];
    P(:,7) = [p1x+deltaX, p1y+deltaY, p1z+deltaZ,1];
    P(:,8) = [p1x-deltaX, p1y+deltaY, p1z+deltaZ,1];

    %% Camera Frustum
    
    P(:,9) = [p1x, p1y, p1z+deltaZ,1];
    P(:,10) = [p1x-deltaX, p1y-deltaY, p1z+2*deltaZ,1];
    P(:,11) = [p1x+deltaX, p1y-deltaY, p1z+2*deltaZ,1];
    P(:,12) = [p1x+deltaX, p1y+deltaY, p1z+2*deltaZ,1];
    P(:,13) = [p1x-deltaX, p1y+deltaY, p1z+2*deltaZ,1];
    
    %% Transform Camera Pose
  
    M = transformationMatrix(tx,ty,tz,phi,theta,psi);
    PT = M*P;
    
    %% Draw Faces
    
    boxFaces = [1 2 3 4; 2 6 7 3; 5 6 7 8; 1 5 8 4; 1 2 6 5; 4 3 7 8];
    patch('Vertices',PT(1:3,:)','Faces',boxFaces,'FaceColor',[0.7 0.7 0.7]);
    
    camFrustFaces = [9 10 13;9 10 11;9 11 12;9 12 13];
    patch('Vertices',PT(1:3,:)','Faces',camFrustFaces,'FaceColor',[0.7 0.4 0.4]);
    
end