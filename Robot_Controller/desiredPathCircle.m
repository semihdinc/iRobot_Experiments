function [qr] = desiredPathCircle(t)
    x0 = 100; %100 cm radius circular motion
    y0 = 0;
    
    thetar = t*(2*pi/36);

    xr = cos(thetar)*x0 - sin(thetar)*y0; 
    yr = sin(thetar)*x0 + cos(thetar)*y0; 
    
    qr = [xr;yr;thetar];
end

