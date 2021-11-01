%Robot Motion Test
clc;clear;
syms ts Ts;

T = 24;
t = 0:0.01:T;

x0 = 4;
xc = 0.95;

%Position/Time
x = -(x0+xc)/2 - (x0-xc)/2*cos(2*pi*ts/Ts);
xx = double(subs(x,{'ts','Ts'},{t,T}));

subplot(1,2,1);plot(t,xx);
legend('position / time');

%Velocity/Time
xd = diff(x,'ts');
xxd = double(subs(xd,{'ts','Ts'},{t,T}));

subplot(1,2,2);plot(t,xxd);
legend('velocity / time');

% hold all;

% plot([0 1.25],[0.08 0.58]);
% plot([1.25 2.5],[0.58 1.5]);

% xp = [0 1.25 2.5]';
% yp = [0.1 0.58 1.5]';
% 
% f2 = fit(xp,yp,'poly2');
% plot(f2,xp,yp);