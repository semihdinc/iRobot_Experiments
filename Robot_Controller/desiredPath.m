function [qr, ur] = desiredPath(t)
    A = 1;
    xL = 10;
    vx = 1;

    xr = vx*t - 30;
    yr = A*sin(2*pi*xr/xL) + 2;

    % desired x and y derivatives
    xrdot = vx;
    yrdot = A*(2*pi/xL)*cos(2*pi*xr/xL);

    % desired theta
    thetar = atan2(yrdot,xrdot);

    % desired second derivatives
    xrddot = 0;
    yrddot = -A*(2*pi/xL)^2*sin(2*pi*xr/xL);

    % desired theta derivative
    thetardot = (yrddot - xrddot*tan(thetar))/(xrdot*(1+tan(thetar)^2));

    vr     = sqrt(xrdot^2+yrdot^2);
    omegar = thetardot;

    qr = [xr;yr;thetar];
    ur = [vr;omegar];
end