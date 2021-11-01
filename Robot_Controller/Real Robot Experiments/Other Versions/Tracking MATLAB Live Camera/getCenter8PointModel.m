function p = getCenter8PointModel(image)

image = rgb2ycbcr(image);

image(:,:,1) = medfilt2(image(:,:,1), [3 3]);
i = image(:,:,1);
T2=-1;
R1=[];
R2=[];
smax=max(i(:));
smin=min(i(:));
T1=mean(i(:));
while(abs(T1-T2)~=0)
    if (T2~=-1)
        T1=T2;
    end
    R1=(i(i(:)<=T1));
    R2=(i(i(:)>T1));
    m1=mean(R1);
    m2=mean(R2);
    T2=(m1+m2)/2;
end
imR1=(i<=T2);
imR2=(i>T2);
imR1 = medfilt2(imR1, [3 3]);
stats = regionprops(imR1,'Centroid','MajorAxisLength','MinorAxisLength','BoundingBox');
% figure(1);
% imshow(imR1); hold on;

numOfRegions = size(stats,1);

c=0;
centers = [];
for i = 1:numOfRegions 
    if (c<=7)   
        l_centers = stats(i).Centroid;
        l_bb = stats(i).BoundingBox;
        l_diameters =stats(i).MajorAxisLength;
        l_radii = l_diameters/2;

        if ((l_radii >10)&&(l_radii <70)) && (round(abs(l_bb(3)-l_bb(4)))<=15)
            centers = [centers;l_centers];
            c=c+1;
        end
    end
end 

if (c==8)
   centers=matchpoint(centers);
end    

% for i=1:c %length(centers)
%     plot(centers(i,1),centers(i,2),'Marker','*');
%     text(centers(i,1),centers(i,2),num2str(i));
% end

p = centers';

end


function ocenters=matchpoint(icenters)
    [~,id]=sort(icenters(:,1));
    ocenters = icenters(id,:);
end
