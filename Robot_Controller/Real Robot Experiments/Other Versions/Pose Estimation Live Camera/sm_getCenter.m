function [y_l, y_r] = sm_getCenter(l_rgb,r_rgb)

    %#codegen
    y_l=zeros(4,2);
    y_r=zeros(4,2);
    
    %input left image(l_rgb), right image(r_rgb), Pose target position 
    [m,n,c]=size(l_rgb);
    
    %define image variables for processing
    l_gray=zeros(m,n);      r_gray=zeros(m,n);
    l_rgb1=zeros(m,n,c);    r_rgb1=zeros(m,n,c);
    l_rgb2=zeros(m,n);      r_rgb2=zeros(m,n);

    l_bw=false(m,n);        r_bw=false(m,n);

    l_rgb1 = imcomplement(l_rgb);        % converts RGB to CMY color space 
    r_rgb1 = imcomplement(r_rgb);        % converts RGB to CMY color space 
    
    l_gray = 0.2989 * l_rgb1(:,:,1) + 0.5870 * l_rgb1(:,:,2) + 0.1140 * l_rgb1(:,:,3); % rgb2gray(rgb) ;convert rgb to grayscale 
    r_gray = 0.2989 * r_rgb1(:,:,1) + 0.5870 * r_rgb1(:,:,2) + 0.1140 * r_rgb1(:,:,3); % rgb2gray(rgb) ;convert rgb to grayscale
    
    l_rgb2 = l_rgb1(:,:,3)-l_gray;       % imsubtract(rgb1(:,:,3),gray)
    r_rgb2 = r_rgb1(:,:,3)-r_gray;       % imsubtract(rgb1(:,:,3),gray) 
    
    l_rgb2 = medfilt2(l_rgb2);     % medfilt2(rgb2,[3 3]);
    r_rgb2 = medfilt2(r_rgb2);     % medfilt2(rgb2,[3 3]);
    
    l_rgb2 = mat2gray(l_rgb2);
    r_rgb2 = mat2gray(r_rgb2);
    
    l_bw = l_rgb2 > 0.5;
    r_bw = r_rgb2 > 0.5;

    %finds yellow regions in image using BlobAnalysis.
    hblob = vision.BlobAnalysis;

    [~,l_center,~] = step(hblob, l_bw); % get [x y] is a centroid of region
    [~,r_center,~] = step(hblob, r_bw); % get [x y] is a centroid of region
    
    %Send Left Camera Points
    ml = size(l_center,1);
    if ml > 4
        ml = 4;
    end
    for i=1:ml
        y_l(i,:)=l_center(i,:);
    end
    
    %Send Right Camera Points
    mr = size(r_center,1);
    if mr > 4
        mr = 4;
    end
    for i=1:mr
        y_r(i,:)=r_center(i,:);
    end

    y_l = y_l';
    y_r = y_r';
%     %Temporary Code for visualization
%     coder.extrinsic('imshow', 'plot','subplot');
%     subplot(1,2,1);imshow(l_rgb);hold on;
%     plot(y_l(:,1),y_l(:,2),'r*');
% 
%     subplot(1,2,2);imshow(r_rgb);hold on;
%     plot(y_r(:,1),y_r(:,2),'r*');
end

