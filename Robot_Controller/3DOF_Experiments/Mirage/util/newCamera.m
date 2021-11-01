function camera = newCamera(pose,fx,fy,cx,cy,s,d)
%This function creates a new camera object

camera.pose = pose;

%Focal Length
camera.fx = fx;
camera.fy = fy;

%Principle Points
camera.cx = cx;
camera.cy = cy;

%Skewness Coeff
camera.s = s;

%Distortion Coefficients
camera.d = d;