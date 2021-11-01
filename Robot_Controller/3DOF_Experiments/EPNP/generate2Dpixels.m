function p = generate2Dpixels(cameraPose,A,P)
    
    PC = zeros(size(P));
    n = size(P,1);

    x_rotation = cameraPose.phi;
    y_rotation = cameraPose.theta;
    z_rotation = cameraPose.psi;
    tx = cameraPose.tx; 
    ty = cameraPose.ty; 
    tz = cameraPose.tz;
    
    Rt=return_Rt_matrix(x_rotation,y_rotation,z_rotation,tx,ty,tz);
    
    for i=1:n
       PC(i,:) = transform_3d(Rt,P(i,:)'); 
    end
    
    %we transformed P point from world space to camera space. Now we will
    %project every point to the image plane.
    
    for i=1:n
        %project points into the image plane
        p(i,:) = project_3d_2d([A [0 0 0]'],PC(i,:)');
    end
    
    p = [p ones(n,1)];
end