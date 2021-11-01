function [errParams] = visioMotor(L_p,R_p,S_PDes)
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
	LCamPos.x = 3.535;
	LCamPos.y = 7.9686;
	LCamPos.z = -2.4502e-05;
	LCamPos.phi = 0.059753;
	LCamPos.theta = 0.097862;
	LCamPos.psi = 0.013663;
    
	RCamPos.x = 3.4411;
	RCamPos.y = -4.2378;
	RCamPos.z = 0.00049345;
	RCamPos.phi = 0.085761;
	RCamPos.theta = 0.083684;
	RCamPos.psi = 0.02723;
    
    %Transform from robot space to camera space
    L_E_S = inv(EulerTrans(LCamPos));
    R_E_S = inv(EulerTrans(RCamPos));

	% Intrinsic Camera Parameters
	f1 = 1434.3201; % focal length of the lens (pixels)
	f2 = 1440.8647; % focal length of the lens (pixels)
	ox = 660.0638; % coordinates of the optical axis (pixels)
	oy = 508.0206; % coordinates of the optical axis (pixels)
	L_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

	f1 = 1430.5441; % focal length of the lens (pixels)
	f2 = 1449.8868; % focal length of the lens (pixels)
	ox = 641.7108; % coordinates of the optical axis (pixels)
	oy = 459.6969; % coordinates of the optical axis (pixels)
	R_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

    % Project Points into the camera and image space
    [L_pDes1, L_PDes, L_M] = transformToImageSpace(S_PDes,L_E_S,L_K);
    [R_pDes1, R_PDes, R_M] = transformToImageSpace(S_PDes,R_E_S,R_K);

    L_p(2,:) = 960 - L_p(2,:);
    R_p(2,:) = 960 - R_p(2,:);
    
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
