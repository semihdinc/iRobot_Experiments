function K = intrinsicMatrix(cam)
    %Takes the camera object and returns 4x4 intrinsic camera matrix
    
    K = [[cam.fx,   0,      cam.cx, 0];
         [0,        cam.fy, cam.cy, 0];
         [0,        0,      1,      0];
         [0,        0,      0,      1]];
end