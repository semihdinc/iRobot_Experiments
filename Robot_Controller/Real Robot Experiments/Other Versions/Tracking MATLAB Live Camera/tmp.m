    load params1.mat;
    
    disp(['LCamPos.x = ',num2str(leftParams(1)),';']);
    disp(['LCamPos.y = ',num2str(leftParams(2)),';']);
    disp(['LCamPos.z = ',num2str(leftParams(3)),';']);
    disp(['LCamPos.phi = ',num2str(leftParams(4)),';']);
    disp(['LCamPos.theta = ',num2str(leftParams(5)),';']);
    disp(['LCamPos.psi = ',num2str(leftParams(6)),';']);
    disp(' ');
    disp(['RCamPos.x = ',num2str(rightParams(1)),';']);
    disp(['RCamPos.y = ',num2str(rightParams(2)),';']);
    disp(['RCamPos.z = ',num2str(rightParams(3)),';']);
    disp(['RCamPos.phi = ',num2str(rightParams(4)),';']);
    disp(['RCamPos.theta = ',num2str(rightParams(5)),';']);
    disp(['RCamPos.psi = ',num2str(rightParams(6)),';']);
    disp(' ');
    disp('% Intrinsic Camera Parameters');
    disp(['f1 = ',num2str(leftParams(7)),'; % focal length of the lens (pixels)']);
    disp(['f2 = ',num2str(leftParams(8)),'; % focal length of the lens (pixels)']);
    disp(['ox = ',num2str(leftParams(9)),'; % coordinates of the optical axis (pixels)']);
    disp(['oy = ',num2str(leftParams(10)),'; % coordinates of the optical axis (pixels)']);
    disp(' ');
    disp(['f1 = ',num2str(rightParams(7)),'; % focal length of the lens (pixels)']);
    disp(['f2 = ',num2str(rightParams(8)),'; % focal length of the lens (pixels)']);
    disp(['ox = ',num2str(rightParams(9)),'; % coordinates of the optical axis (pixels)']);
    disp(['oy = ',num2str(rightParams(10)),'; % coordinates of the optical axis (pixels)']);
    
% LCamPos.x = 3.7319;
% LCamPos.y = 5.2138;
% LCamPos.z = 6.1999e-05;
% LCamPos.phi = 0.054212;
% LCamPos.theta = 0.098725;
% LCamPos.psi = 0.031126;
%  
% RCamPos.x = 3.6111;
% RCamPos.y = -6.7935;
% RCamPos.z = -0.000295;
% RCamPos.phi = 0.05329;
% RCamPos.theta = 0.095534;
% RCamPos.psi = 0.030141;
%  
% % Intrinsic Camera Parameters
% f1 = 1432.806; % focal length of the lens (pixels)
% f2 = 1429.6897; % focal length of the lens (pixels)
% ox = 634.8913; % coordinates of the optical axis (pixels)
% oy = 522.2617; % coordinates of the optical axis (pixels)
%  
% f1 = 1460.5466; % focal length of the lens (pixels)
% f2 = 1437.8156; % focal length of the lens (pixels)
% ox = 636.1514; % coordinates of the optical axis (pixels)
% oy = 452.2788; % coordinates of the optical axis (pixels)