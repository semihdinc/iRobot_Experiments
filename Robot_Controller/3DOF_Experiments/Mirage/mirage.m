%APEM : (A)nalitic (P)ose (E)stimation (M)ethod
function poseError = mirage(robot,S_PDes, L_p, R_p)
    
    %PDes : Desired Target coordinates in Robot Space
    %L_p : Actual pixel coordinates in Left image space
    %R_p : Actual pixel coordinates in Right image space
    
    %Project desired points into image space
    L_retValDes = transformImageSpace(S_PDes,robot.leftCamera);
    R_retValDes = transformImageSpace(S_PDes,robot.rightCamera);
     
    %Desire image pixel coordinates
    L_pDes = L_retValDes.p;
    R_pDes = R_retValDes.p;
    
    L_PDes = L_retValDes.C_P;
    R_PDes = R_retValDes.C_P;
    
    L_M = L_retValDes.M;
    R_M = R_retValDes.M;
    
    %Calculate the image error
    Err_Left = L_p - L_pDes;
    Err_Right = R_p - R_pDes;
        
    % Left and Right Camera Equations (2 equations for 1 point)
    [Left_Equations, Left_Constants] = getEquations(L_PDes,S_PDes,Err_Left,L_M);
    [Right_Equations, Right_Constants] = getEquations(R_PDes,S_PDes,Err_Right,R_M);

    AllEquations = [Left_Equations;Right_Equations];
    AllConstants = [Left_Constants;Right_Constants];

%     poseError = solveEquations(AllEquations,AllConstants);
%     poseError = linsolve(AllEquations,AllConstants);
    poseError = AllEquations\AllConstants;
end