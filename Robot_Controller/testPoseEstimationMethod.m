q = [-32;-2;0]; %initial pose

t = 0:0.15:20; %The simulation runs 20 secs

for iter=1:size(t,2)
    [qr, ur] = desiredPath(t(iter));

    qtilde = q - qr;

    %Controller and Robot Part
    u = controller(qtilde,qr,ur);

    %Robot moves qdot amount according to the kinematic model
    qdot = kinematicModel(u,q);

    %new position of the robot
    q = q + qdot;
end
