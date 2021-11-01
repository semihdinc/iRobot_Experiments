function retVal = projectionErrorFunc(x, C_p, S_PDes)

    global C_pDes; 
    
    LCamPos.x =     x(1);
    LCamPos.y =     x(2);    
    LCamPos.z =     x(3);  
    LCamPos.phi =   x(4);   
    LCamPos.theta = x(5);  
    LCamPos.psi =   x(6); 

    %Transform from robot space to camera space
    C_E_S = inv(EulerTrans(LCamPos));

    % Intrinsic Camera Parameters
    C_K = [ x(7),       0,          x(9),       0;
            0,          x(8),       x(10),      0;
            0,          0,          1,          0;
            0,          0,          0,          1];
        
    % Project Points into the camera and image space
    [C_pDes, ~, ~] = transformToImageSpace(S_PDes,C_E_S,C_K);
    C_pDes(2,:) = 480-C_pDes(2,:);

    retVal = sum(sum(abs(C_p - C_pDes)));
end