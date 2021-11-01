function [errParams] = mirage(L_p,R_p,S_PDes)
% This function gets the desired positions of 4 3D target points in 
% robot space and 8 2D target points on the image for left and right 
% cameras. Then it computes the t parameters that contains direction 
% information to close desired position
%
% S_PDes :(3x3) Desired position of the 3 target points in Robot Space 
% L_p    :(2x3) Pixel values of the 3 target points on the left image
% R_p    :(2x3) Pixel values of the 3 target points on the right image
%
% errParams :(12x1) 9 Rotation and 3 Translation Parameters. Robot will use
% these t parameters and change its position into desired position

    % Transformation Matrices (Extrinsic Parameters)
    LCamPos.x = 3.2096;
    LCamPos.y = 5.4775;
    LCamPos.z = 3.5866e-06;
    LCamPos.phi = 0.049542;
    LCamPos.theta = 0.10444;
    LCamPos.psi = 0.030472;

    RCamPos.x = 3.6611;
    RCamPos.y = -6.1185;
    RCamPos.z = -0.00032663;
    RCamPos.phi = 0.051959;
    RCamPos.theta = 0.098941;
    RCamPos.psi = 0.030096;

    %Transform from robot space to camera space
    L_E_S = inv(EulerTrans(LCamPos));
    R_E_S = inv(EulerTrans(RCamPos));

    % Intrinsic Camera Parameters
    f1 = 714.6172; % focal length of the lens (pixels)
    f2 = 725.6549; % focal length of the lens (pixels)
    ox = 322.847; % coordinates of the optical axis (pixels)
    oy = 263.4883; % coordinates of the optical axis (pixels) 
    L_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

    f1 = 709.8338; % focal length of the lens (pixels)
    f2 = 719.9022; % focal length of the lens (pixels)
    ox = 322.7411; % coordinates of the optical axis (pixels)
    oy = 231.1758; % coordinates of the optical axis (pixels)
    R_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

    % Project Points into the camera and image space
    [L_pDes1, L_PDes, L_M] = transformToImageSpace(S_PDes,L_E_S,L_K);
    [R_pDes1, R_PDes, R_M] = transformToImageSpace(S_PDes,R_E_S,R_K);

    L_p(2,:) = 480 - L_p(2,:);
    R_p(2,:) = 480 - R_p(2,:);
    
    % Pixel Error
    Err_Left = L_p - L_pDes1;
    Err_Right = R_p - R_pDes1;

    % Left and Right Camera Equations (2 equations for 1 point)
    [Left_Equations, Left_Constants] = getEquations(L_PDes,S_PDes,Err_Left,L_M);
    [Right_Equations, Right_Constants] = getEquations(R_PDes,S_PDes,Err_Right,R_M);

    AllEquations = [Left_Equations;Right_Equations];
    AllConstants = [Left_Constants;Right_Constants];

    errParams = solveEquations(double(AllEquations),double(AllConstants)); 
end
