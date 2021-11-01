%Robot Motion Test
clc;clear;
syms ts Ts;

T = 12;
t = 0:0.1:T;

x0 = -3/2; A = -3/2;

x = x0 + A*cos(2*pi*ts/T);
xd = diff(x,'ts');
l = 5.68*ts/T + 0.08;

% f = matlabFunction(xd-l);
% fminsearch(f, [10, 0.5]);

xxd = double(subs(xd,'ts',t));
xx = double(subs(x,'ts',t));
ll = double(subs(l,'ts',t));
ll = ll(ll<1.5);
tl = t(ll<1.5);

plot(t,xxd); %hold all;plot(tl,ll);

hold all;
plot([0 2.5],[0.08 1.5]);
hold all;
plot([0 1.25],[0.08 0.58]);