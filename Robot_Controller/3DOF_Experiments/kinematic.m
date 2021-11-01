% Kinematic Model
function qdot = kinematic(q,u)
% q = [ x y theta]^T
% qdot = [ xdot ydot thetadot]^T
% u = [v omega]^T

theta = q(3,1);

qdot = [cos(theta),0;0,sin(theta);0,1]*u;
