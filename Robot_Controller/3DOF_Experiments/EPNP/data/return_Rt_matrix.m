function M = return_Rt_matrix(varargin)

if(nargin == 1)
    pose = varargin{1};    
    tx = pose.tx;
    ty = pose.ty;
    tz = pose.tz;
    phi = pose.phi;
    theta = pose.theta;
    psi = pose.psi;

elseif(nargin == 6)
    tx = varargin{4};
    ty = varargin{5};
    tz = varargin{6};
    phi = varargin{1};
    theta = varargin{2};
    psi = varargin{3};
end

%% 

DesRobotPose.x = tx;
DesRobotPose.y = ty;
DesRobotPose.z = tz;
DesRobotPose.phi = phi;
DesRobotPose.theta = theta;
DesRobotPose.psi = psi;

%Transform target points from I space into Desired Robot Space
M = (EulerTrans(DesRobotPose));

% Rx = makehgtform('xrotate',phi);
% Ry = makehgtform('yrotate',theta);
% Rz = makehgtform('zrotate',psi);
% 
% T = makehgtform('translate',[tx ty tz]);
% 
% M = Rx*Ry*Rz*T;

end
% function R=return_Rt_matrix(alpha,beta,gamma,tx,ty,tz)
% 
% % Copyright (C) <2007>  <Francesc Moreno-Noguer, Vincent Lepetit, Pascal Fua>
% % 
% % This program is free software: you can redistribute it and/or modify
% % it under the terms of the version 3 of the GNU General Public License
% % as published by the Free Software Foundation.
% % 
% % This program is distributed in the hope that it will be useful, but
% % WITHOUT ANY WARRANTY; without even the implied warranty of
% % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% % General Public License for more details.       
% % You should have received a copy of the GNU General Public License
% % along with this program. If not, see <http://www.gnu.org/licenses/>.
% %
% % Francesc Moreno-Noguer, CVLab-EPFL, September 2007.
% % fmorenoguer@gmail.com, http://cvlab.epfl.ch/~fmoreno/ 
% 
% R(1,1)=cos(alpha)*cos(gamma)-cos(beta)*sin(alpha)*sin(gamma);
% R(2,1)=cos(gamma)*sin(alpha)+cos(alpha)*cos(beta)*sin(gamma);
% R(3,1)=sin(beta)*sin(gamma);
% R(4,1)=0;
% 
% R(1,2)=-cos(beta)*cos(gamma)*sin(alpha)-cos(alpha)*sin(gamma);
% R(2,2)=cos(alpha)*cos(beta)*cos(gamma)-sin(alpha)*sin(gamma);
% R(3,2)=cos(gamma)*sin(beta);
% R(4,2)=0;
% 
% R(1,3)=sin(alpha)*sin(beta);
% R(2,3)=-cos(alpha)*sin(beta);
% R(3,3)=cos(beta);
% R(4,3)=0;
% 
% R(:,4)=[tx,ty,tz,1]';

