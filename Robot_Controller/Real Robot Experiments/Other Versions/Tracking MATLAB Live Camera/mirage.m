function errParams = mirage(L_p,R_p,S_PDes)
    
    load params1.mat;
    
    LCamPos.x = leftParams(1);
    LCamPos.y = leftParams(2);    
    LCamPos.z = leftParams(3);  
    LCamPos.phi = leftParams(4);   
    LCamPos.theta = leftParams(5);  
    LCamPos.psi = leftParams(6); 

    RCamPos.x = rightParams(1);  
    RCamPos.y = rightParams(2); 
    RCamPos.z = rightParams(3);
    RCamPos.phi = rightParams(4);  
    RCamPos.theta = rightParams(5);
    RCamPos.psi = rightParams(6);

    %Transform from robot space to camera space
    L_E_S = inv(EulerTrans(LCamPos));
    R_E_S = inv(EulerTrans(RCamPos));

    % Intrinsic Camera Parameters
    f1 = leftParams(7);   % focal length of the lens (pixels)
    f2 = leftParams(8);
    ox = leftParams(9);   % coordinates of the optical axis (pixels)
    oy = leftParams(10);   % coordinates of the optical axis (pixels)
    L_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

    f1 = rightParams(7);   % focal length of the lens (pixels)
    f2 = rightParams(8);
    ox = rightParams(9);   % coordinates of the optical axis (pixels)
    oy = rightParams(10);   % coordinates of the optical axis (pixels)
    R_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

    % Project Points into the camera and image space
    [L_pDes1, L_PDes, L_M] = transformToImageSpace(S_PDes,L_E_S,L_K);
    [R_pDes1, R_PDes, R_M] = transformToImageSpace(S_PDes,R_E_S,R_K);

    L_p(2,:) = 480 - L_p(2,:);
    R_p(2,:) = 480 - R_p(2,:);
    
%     plot(L_p(1,:),L_p(2,:));
%     hold all;
%     plot(L_pDes1(1,:),L_pDes1(2,:));
    
    % Pixel Error
    Err_Left = L_p - L_pDes1;
    Err_Right = R_p - R_pDes1;

%     retVal = sum(sum((L_p - L_pDes1).^2));
    
    % Left and Right Camera Equations (2 equations for 1 point)
    [Left_Equations, Left_Constants] = getEquations(L_PDes,S_PDes,Err_Left,L_M);
    [Right_Equations, Right_Constants] = getEquations(R_PDes,S_PDes,Err_Right,R_M);

    AllEquations = [Left_Equations;Right_Equations];
    AllConstants = [Left_Constants;Right_Constants];

    errParams = solveEquations(double(AllEquations),double(AllConstants));                          
end