function [L_p, R_p, sWindowOut] = getCenterV2(leftImage, rightImage, sWindowIn)

    %#codegen
    L_p=zeros(2,4);
    R_p=zeros(2,4);
    
    m = size(leftImage,1);
    sWindowOut = zeros(size(sWindowIn));
    for i=1:4
        lw = sWindowIn(i,:);
        lwx = lw(1):lw(1)+lw(3);
        lwy = lw(2):lw(2)+lw(4);
        
        rw = sWindowIn(i+4,:);
        rwx = rw(1):rw(1)+rw(3);
        rwy = rw(2):rw(2)+rw(4);
        
        pWinL = leftImage(lwy,lwx,:);
        pWinR = rightImage(rwy,rwx,:);
                
        pWinL = imcomplement(pWinL);        
        pWinR = imcomplement(pWinR);        
    
        pGrayL = 0.2989 * pWinL(:,:,1) + 0.5870 * pWinL(:,:,2) + 0.1140 * pWinL(:,:,3);
        pGrayR = 0.2989 * pWinR(:,:,1) + 0.5870 * pWinR(:,:,2) + 0.1140 * pWinR(:,:,3);
    
        pWinLF = pWinL(:,:,3)-pGrayL;       % imsubtract(rgb1(:,:,3),gray)
        pWinRF = pWinR(:,:,3)-pGrayR;       % imsubtract(rgb1(:,:,3),gray) 

        pWinLF = medfilt2(pWinLF);     % medfilt2(rgb2,[3 3]);
        pWinRF = medfilt2(pWinRF);     % medfilt2(rgb2,[3 3]);

        pWinLF = mat2gray(pWinLF);
        pWinRF = mat2gray(pWinRF);

        pBWL = pWinLF > 0.2;
        pBWR = pWinRF > 0.2;

        %finds yellow regions in image using BlobAnalysis.
        hblob = vision.BlobAnalysis;

        [~,cL,bbL] = step(hblob, pBWL); % get [x y] is a centroid of region
        [~,cR,bbR] = step(hblob, pBWR); % get [x y] is a centroid of region
        
        L_p(:,i) = cL(1,:) + lw(1:2);
        R_p(:,i) = cR(1,:) + rw(1:2);
        
        coeffL = double([-0.3 0;0 -0.3;0.3 0;0 0.3]); szL = double(bbL(3:4)');
        coeffR = double([-0.3 0;0 -0.3;0.3 0;0 0.3]); szR = double(bbR(3:4)');
        
        sWindowOut(i,:)   = round([lw(1); lw(2); 0; 0] + double(bbL') + coeffL*szL)';
        sWindowOut(i+4,:) = round([rw(1); rw(2); 0; 0] + double(bbR') + coeffR*szR)';
    end
    
%     L_p(2,:) = m - L_p(2,:);
%     R_p(2,:) = m - R_p(2,:);
end