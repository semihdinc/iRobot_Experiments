%Robot Motion Test
clc;clear;
%%
% SimulinkRealTime.copyFileToHost('q.dat');

% t = datetime('now');
% new_q_fileName = strcat('.\experiments\',datestr(t,'yyyy_mm_dd_HH_MM_SS'),'_q');
% new_qr_fileName = strcat('.\experiments\',datestr(t,'yyyy_mm_dd_HH_MM_SS'),'_qr');
fileName = './experiments/expResults';

% movefile('q.dat',fileName);

qRaw = SimulinkRealTime.utils.getFileScopeData(fileName);

ind = 566:1050;

t =      qRaw.data(ind,8);
xd =     qRaw.data(ind,1);
x =      qRaw.data(ind,7);
yd =     qRaw.data(ind,3);
y =      qRaw.data(ind,4);
thetad = qRaw.data(ind,5);
theta =  qRaw.data(ind,6);

t = t - qRaw.data(566,8);

% set(figure(1),'PaperPositionMode', 'auto');
% 
% subplot(1,3,1);plot(t,xd),hold all, plot(t,x);
% legend('x^d','x','Location','northeast'); 
% ylabel('x(m)'); xlabel('time(sec)'); ylim([-6,2]);
% 
% subplot(1,3,2);plot(t,yd),hold all, plot(t,y);
% legend('y^d','y','Location','northeast'); 
% ylabel('y(m)'); xlabel('time(sec)'); ylim([-2,2]);
% 
% subplot(1,3,3);plot(t,thetad),hold all, plot(t,theta);
% legend('\theta^d','\theta','Location','northeast'); 
% ylabel('\theta(radians)'); xlabel('time(sec)'); ylim([-pi,pi]);

% set(gcf,'PaperUnits','inches','PaperPosition',[0 0 8 2]);
% print -depsc '..\realExperimentResults.eps';

ex = sqrt(sum((x - xd).^2));
ey = sqrt(sum((y - yd).^2));
etheta = sqrt(sum((theta - thetad).^2));

%%

%%
% new_q_fileName = './experiments/2015_09_30_18_28_26_q';
% new_qr_fileName = './experiments/2015_09_30_18_28_26_qr';
% qRaw = SimulinkRealTime.utils.getFileScopeData(new_q_fileName);
% qrRaw = SimulinkRealTime.utils.getFileScopeData(new_qr_fileName);
% 
% subplot(1,2,1);plot(qRaw.data(:,1)),hold all, plot(qrRaw.data(:,1));
% subplot(1,2,2);plot(qRaw.data(:,2)),hold all, plot(qrRaw.data(:,2));

%%
% f=SimulinkRealTime.fileSystem;
% qf =fopen(f,'q.dat');
% qrf=fopen(f,'qr.dat');
% 
% data_q = fread(f,qf);
% data_qr = fread(f,qrf);
% 
% f.fclose(qf);
% f.fclose(qrf);
% 
% % Unpack the data.
% q = SimulinkRealTime.utils.getFileScopeData(data_q);
% qr = SimulinkRealTime.utils.getFileScopeData(data_qr);
% 
% q = q.data;
% qr = qr.data;
% 
% 
% subplot(1,2,1);plot(q(:,1));hold all;plot(qr(:,1));
% subplot(1,2,2);plot(q(:,2));hold all;plot(qr(:,2));
%%

% x0 = 4;
% xc = 1;
% T = 48;
% 
% t = 0:0.3:T;
% s = length(t);
% 
% %Position/Time
% xr = -(x0+xc)/2 - (x0-xc)/2*cos(2*pi*t/T);
% yr = zeros(1,s);
% zr = 0.25;
% phir = 0;
% thetar = 0;
% 
% xrdot = (61*pi*sin((2*pi*t)/T))/(20*T);%(61*pi*sin((pi*t)/6))/240;
% yrdot = 0;
% 
% psir = atan2(yrdot,xrdot);
% psir(psir == pi) = 0;
% 
% vr = sqrt(xrdot.^2+yrdot.^2);
% 
% figure;
% subplot(3,1,1);plot(t,xr);
% subplot(3,1,2);plot(t,xrdot);
% subplot(3,1,3);plot(t,vr);