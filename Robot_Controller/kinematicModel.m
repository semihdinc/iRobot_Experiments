function qdot = kinematicModel(u,q)
% Kinematic Model
lambda = 0.125;
theta = q(3);
qdot = lambda*[cos(theta),0;sin(theta),0;0,1]*u; % Eq.(2)
