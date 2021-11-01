function errParams = mirage(L_p,R_p,S_PDes)
        
    LCamPos.x = 1.0684;
    LCamPos.y = +6.6773;    
    LCamPos.z = 0;  
    LCamPos.phi = -0.0077;   
    LCamPos.theta = -0.0056;  
    LCamPos.psi = 0.0114; 

    RCamPos.x = 0.8901;  
    RCamPos.y = -4.95; 
    RCamPos.z = 0.0026;
    RCamPos.phi = 0.0112;  
    RCamPos.theta = -0.0134;
    RCamPos.psi = 0;

    %Transform from robot space to camera space
    L_E_S = inv(EulerTrans(LCamPos));
    R_E_S = inv(EulerTrans(RCamPos));

    % Intrinsic Camera Parameters
    f1 = 1412.45843318963;   % focal length of the lens (pixels)
    f2 = 1444.55255979655;
    ox = 606.056420829776;   % coordinates of the optical axis (pixels)
    oy = 448.719680077129;   % coordinates of the optical axis (pixels)
    L_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

    f1 = 1418.69905797363;   % focal length of the lens (pixels)
    f2 = 1444.76712590545;
    ox = 631.297389348331;   % coordinates of the optical axis (pixels)
    oy = 386.501819479235;   % coordinates of the optical axis (pixels)
    R_K = [[f1,0,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

    % Project Points into the camera and image space
    [L_pDes1, L_PDes, L_M] = transformToImageSpace(S_PDes,L_E_S,L_K);
    [R_pDes1, R_PDes, R_M] = transformToImageSpace(S_PDes,R_E_S,R_K);

%     L_pDes1(2,:) = 960 - L_pDes1(2,:);
%     R_pDes1(2,:) = 960 - R_pDes1(2,:);
    
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