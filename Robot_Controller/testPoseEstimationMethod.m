function [qs, qrs] = testPoseEstimationMethod(visionSystem,nl,N)

%     visionSystem = str2func(poseEstFunc);
    q = [-32;-2;0];

    t = 0:0.15:20; %The simulation runs 20 secs
    for iter=1:size(t,2)
        [qr, ur] = desiredPath(t(iter));

        %This part will be different for all methods
    %     qtilde = visionSystemMirage(q,qr);
        qtilde = visionSystem(q,qr,nl,N);

        %Controller and Robot Part
        u = controller(qtilde,qr,ur);
        qdot = kinematicModel(u,q);
        q = q + qdot;

        %Save pose parameters
        qs(iter,:) = q;
        qrs(iter,:) = qr;
    end
end