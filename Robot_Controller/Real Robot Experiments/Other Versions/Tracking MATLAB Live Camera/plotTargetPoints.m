function plotTargetPoints(image,p,pDes)

%     [m, ~] = size(image);
    imagesc(image);hold on;
    plot(p(1,:),p(2,:),'r*');
    plot(pDes(1,:),pDes(2,:),'go');
    for i=1:size(p,2);
        text(p(1,i),p(2,i),num2str(i));
        text(pDes(1,i),pDes(2,i),num2str(i));
    end
end