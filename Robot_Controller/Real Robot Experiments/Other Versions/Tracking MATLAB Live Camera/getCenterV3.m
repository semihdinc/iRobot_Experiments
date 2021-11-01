function [L_p, R_p] = getCenterV3(leftImage, rightImage, L_pIn, R_pIn)
    %#codegen 
    ps = 0; %Pad Size
%     leftImage = padarray(leftImage,[ps ps],'both');
%     rightImage = padarray(rightImage,[ps ps],'both');
    
    lwc = round(L_pIn) + ps; %Center of the left windows
    rwc = round(R_pIn) + ps; %Center of the right windows
    
    L_p = zeros(2,4);
    R_p = zeros(2,4);
    
    winRng = -40:40;
    n = length(winRng);
    
    imgL = uint8(zeros(n,4*n));
    imgR = uint8(zeros(n,4*n));
    
    for i=1:4
        lwx = winRng + lwc(1,i);
        lwy = winRng + lwc(2,i);
        
        rwx = winRng + rwc(1,i);
        rwy = winRng + rwc(2,i);
        
        pWinL = leftImage(lwy,lwx,:);
        pWinR = rightImage(rwy,rwx,:);
                
        pWinL = imcomplement(pWinL);        
        pWinR = imcomplement(pWinR);        
            
        pGrayL = 0.2989 * pWinL(:,:,1) + 0.5870 * pWinL(:,:,2) + 0.1140 * pWinL(:,:,3);
        pGrayR = 0.2989 * pWinR(:,:,1) + 0.5870 * pWinR(:,:,2) + 0.1140 * pWinR(:,:,3);
    
        pWinLF = pWinL(:,:,3) - pGrayL;       % imsubtract(rgb1(:,:,3),gray)
        pWinRF = pWinR(:,:,3) - pGrayR;       % imsubtract(rgb1(:,:,3),gray) 
                
        hautoth = vision.Autothresholder;
        pBWL = step(hautoth,pWinLF);
        pBWR = step(hautoth,pWinRF);
        
        rng = (1:n) + (i-1)*n;
        imgL(:,rng) = pGrayL;
        imgR(:,rng) = pGrayR;
        
        %finds yellow regions in image using BlobAnalysis.
        hblob = vision.BlobAnalysis;

        [aL,cL,~] = step(hblob, pBWL); % get [x y] is a centroid of region
        [aR,cR,~] = step(hblob, pBWR); % get [x y] is a centroid of region
        
        [~, indL] = sort(aL,'descend');
        [~, indR] = sort(aR,'descend');
        
        if(size(cL,1) ~= 0) && (size(cR,1) ~= 0)
            L_p(:,i) = cL(indL(1),:)' + [lwx(1);lwy(1)] - [ps;ps];
            R_p(:,i) = cR(indR(1),:)' + [rwx(1);rwy(1)] - [ps;ps];
        else
            L_p(:,i) = L_pIn(:,i);
            R_p(:,i) = R_pIn(:,i);
        end 
    end
    subplot(2,1,1);imshow(imgL);
    subplot(2,1,2);imshow(imgR);
end