function [L_p, R_p] = getCenter(l_rgb,r_rgb)

    %#codegen
    L_p=zeros(2,4);
    R_p=zeros(2,4);
    
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
    
    l_rgb2 = medfilt2(l_rgb2,[3 3]);     % medfilt2(rgb2,[3 3]);
    r_rgb2 = medfilt2(r_rgb2,[3 3]);     % medfilt2(rgb2,[3 3]);
    
    l_rgb2 = mat2gray(l_rgb2);
    r_rgb2 = mat2gray(r_rgb2);
    
    l_bw = l_rgb2 > 0.4;
    r_bw = r_rgb2 > 0.4;
    
    %finds yellow regions in image using BlobAnalysis.
    hblob = vision.BlobAnalysis;

    [l_area,l_center,~] = step(hblob, l_bw); % get [x y] is a centroid of region
    [r_area,r_center,~] = step(hblob, r_bw); % get [x y] is a centroid of region
    
    [~, indL] = sort(l_area,'descend');
    [~, indR] = sort(r_area,'descend');
    
    L_p = l_center(indL(1:4),:)';
    R_p = r_center(indR(1:4),:)';
        
    L_p = orderPoints(L_p,m); 
    R_p = orderPoints(R_p,m);
end

% function imgOut = myMat2gray(imgIn)
%     %#codegen
%     imgIn = double(imgIn);
%     
%     minL = min(imgIn(:));
%     
%     imgTmp = (imgIn - minL);
%     
%     maxL = max(imgTmp(:));
%     
%     imgOut = imgTmp/maxL;
% end