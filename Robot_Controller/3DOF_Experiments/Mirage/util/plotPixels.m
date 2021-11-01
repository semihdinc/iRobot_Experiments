function plotPixels(pixels1,pixels2,camera)

    if(nargin >= 2)
        plot(pixels1(1,:),pixels1(2,:),'-');
        hold all;
        plot(pixels2(1,:),pixels2(2,:),'-');
    else    
        plot(pixels1(1,:),pixels1(2,:),'-');
    end
    xlabel('X');
    ylabel('Y');
    
    for i=1:size(pixels1,2)-1
        text(pixels1(1,i),pixels1(2,i),strcat('p_',num2str(i)));
        text(pixels2(1,i),pixels2(2,i),strcat('p_',num2str(i)));
    end
    
    if exist('camera','var')
        axis([0 camera.cx*2 0 camera.cy*2]);
    end
    
    axis equal;
end