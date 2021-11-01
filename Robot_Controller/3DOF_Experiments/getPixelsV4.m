function [L_p, R_p] = getPixelsV4(q)
%#codegen

P = zeros(4,4);     %4 hedef noktasinin 2D duzlemdeki konumlari 
%S_P = zeros(4,4);   %4 hedef noktasinin robotun 2D duzlemindeki konumlari

p1x = 0;
p1y = 0;
p1z = 0;

P(:,1) = [p1x, p1y, p1z, 1];
P(:,2) = [p1x, p1y+8, p1z, 1];
P(:,3) = [p1x+8, p1y+4, p1z+3, 1];
P(:,4) = [p1x, p1y+4, p1z+8, 1];

RobotPose.x = q(1);
RobotPose.y = q(2);
RobotPose.z = q(3);
RobotPose.phi = q(4);
RobotPose.theta = q(5);
RobotPose.psi = q(6);

S_E_I = inv(EulerTrans(RobotPose));

S_P = S_E_I*P;

% R = [q(1);0;q(2);0];
% S_P1 = P - repmat(R,1,4);

%%
% Pose of the left camera with respect to robot frame
LCamPos.x = 1;
LCamPos.y = +1.7;
LCamPos.z = 1;
LCamPos.phi = 0;%-0.062; %yukari asagi
LCamPos.theta = 0;%-0.031; 
LCamPos.psi = 0;

% Pose of the right camera with respect to robot frame
RCamPos.x = 1;
RCamPos.y = -1.7;
RCamPos.z = 1;
RCamPos.phi = 0;%-0.062;
RCamPos.theta = 0;%-0.03;
RCamPos.psi = 0;

L_E_S = inv(EulerTrans(LCamPos));
R_E_S = inv(EulerTrans(RCamPos));

% L_P = L_E_S*S_P;
% R_P = R_E_S*S_P;

 %% Transformation into Camera/Image Space
 
% intrinsic parameters of Left Camera
f1 = 800;   % focal length of the lens (pixels)
f2 = 800;
ox = 320;   % coordinates of the optical axis (pixels)
oy = 240;   % coordinates of the optical axis (pixels)
d  = 0.0;       % radial distortion

L_K = [[f1,d,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

% intrinsic parameters of Right Camera
f1 = 800;   % focal length of the lens (pixels)
f2 = 800;
ox = 320;   % coordinates of the optical axis (pixels)
oy = 240;   % coordinates of the optical axis (pixels)
d  = 0.0;       % radial distortion

R_K = [[f1,d,ox,0];[0,f2,oy,0];[0,0,1,0];[0,0,0,1]];

[L_p,~,~] = transformToImageSpace(S_P,L_E_S,L_K);
[R_p,~,~] = transformToImageSpace(S_P,R_E_S,R_K);
%%

%bulunan pozisyonlar robotun 2D uzayina aktarilacak. 
%Bu durumda hedef noktalari S_P olarak transform to image space
%fonksiyonuna input olarak verilecek

%[L_pDes L_PDes L_M] = transformToImageSpace(S_PDes,L_E_S,L_K);

%Diger tum parametreler onceki ile normal kullanim ile ayni

%output olarak da mevcut konumdaki robot ve hedeflerin noktalari L_p
%ve R_p olarak  getpixels fonskyonundan cikarilacak.