function [] = plotPointCloud(pointCloud,pointCloud2,arg)

    if(nargin >= 2)
        plot3(pointCloud(1,:),pointCloud(2,:),pointCloud(3,:),arg);
        hold all;
        plot3(pointCloud2(1,:),pointCloud2(2,:),pointCloud2(3,:),arg);
    else    
        plot3(pointCloud(1,:),pointCloud(2,:),pointCloud(3,:),arg);
    end
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    axis equal;
end