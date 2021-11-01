function y_E_x = EulerTrans(Pose)
%#eml

Dx    = Pose.x;
Dy    = Pose.y;
Dz    = Pose.z;
phi   = Pose.phi;
theta = Pose.theta;
psi   = Pose.psi;

Rz=[[cos(psi),-sin(psi),0,0];[sin(psi),cos(psi),0,0];[0,0,1,0];[0,0,0,1]];
Ry=[[cos(theta),0,sin(theta),0];[0,1,0,0];[-sin(theta),0,cos(theta),0];[0,0,0,1]];
Rx=[[1,0,0,0];[0,cos(phi),-sin(phi),0];[0,sin(phi),cos(phi),0];[0,0,0,1]];
Tr=[[1,0,0,Dx];[0,1,0,Dy];[0,0,1,Dz];[0,0,0,1]];

y_E_x = Rx*Ry*Rz*Tr;
