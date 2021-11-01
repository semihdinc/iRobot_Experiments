function M = transformationMatrix(varargin)

if(nargin == 1)
    pose = varargin{1};    
    tx = pose.tx;
    ty = pose.ty;
    tz = pose.tz;
    phi = pose.phi;
    theta = pose.theta;
    psi = pose.psi;

elseif(nargin == 6)
    tx = varargin{1};
    ty = varargin{2};
    tz = varargin{3};
    phi = varargin{4};
    theta = varargin{5};
    psi = varargin{6};
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