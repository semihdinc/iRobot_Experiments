function [qr, ur] = desired(t)
%#codegen

    x0 = 4;
    xc = 0.95;
    T = 12;
    
    %Position/Time
    xr = -(x0+xc)/2 - (x0-xc)/2*cos(2*pi*t/T);
    yr = 0;
    zr = 0.245;
    phir = 0;
    thetar = 0;
     
    xrdot = (61*pi*sin((pi*t)/6))/240;
    yrdot = 0;
    
    psir = atan2(yrdot,xrdot);
    
    % desired second derivatives
    xrddot = (61*pi^2*cos((pi*t)/6))/1440;
    yrddot = 0;
    
    % desired theta derivative
    thetardot = (yrddot - xrddot*tan(thetar))/(xrdot*(1+tan(thetar)^2));

    vr     = sqrt(xrdot^2+yrdot^2);
    omegar = thetardot;
    
    qr = [xr yr zr phir thetar psir]';
    ur = [vr omegar]';
end