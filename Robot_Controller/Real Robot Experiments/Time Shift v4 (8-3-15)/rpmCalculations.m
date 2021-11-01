clc;clear;


% load velocity.mat;
% 
% [f e] = fit(velocity(:,1),velocity(:,2),'poly1');
% disp(e)
% 
% plot(f,velocity(:,1),velocity(:,1));

%%
r = 9; %Radius of the wheels 

% --- Motor power units
% PWM = [800 900 1000 1100 1200 1300 1325 1350 1400 1500 1600 1625 1650 1700 1800 1900 2000]';
% PWM = (1000:100:2000)';

% ---- Rotation per minute (RPM) measurements
% leftRpm =  [-180.1 -180.0 -174.5 -109 -56.1 -22.0 -14.6  -9.0 0 0 0 12.0 17.7 31.2 72.0 126.0 184.4]';
% rightRpm = [-177.7 -176.8 -176.8 -109 -59.0 -23.0 -16.1 -10.5 0 0 0 11.2 14.8 29.1 66.8 120.0 183.3]';
% leftRpm =  [-168 -103.5 -53.6 -21.3 0 0 0 30.3 68 120 176]';
% rightRpm = [-171.2 -105 -57 -22.1 0 0 0 28.2 65 115 175]';

load PWM_RPM.mat
PWM = PWM_RPM(:,1);
leftRpm = PWM_RPM(:,2);
rightRpm = PWM_RPM(:,3);


%Linear velocity wrt RPM and radius
vL = 2*pi*r*(leftRpm/60)/100;
vR = 2*pi*r*(rightRpm/60)/100;

ind1 = 2:10;    %Negative part of the curve (  <0 )
ind2 = 14:22;   %Positive part of the curve ( >=0 )

%Left Wheel
fL1 = fit(vL(ind1),PWM(ind1),'poly3');
fL2 = fit(vL(ind2),PWM(ind2),'poly3');

%Right Wheel
fR1 = fit(vR(ind1),PWM(ind1),'poly3');
fR2 = fit(vR(ind2),PWM(ind2),'poly3');

%plot fitting data
clf;
subplot(1,2,1);plot(fL2,vL(ind2),PWM(ind2));hold all;
subplot(1,2,1);plot(fL1,vL(ind1),PWM(ind1));legend('off');
subplot(1,2,2);plot(fR2,vR(ind2),PWM(ind2));hold all;
subplot(1,2,2);plot(fR1,vR(ind1),PWM(ind1));legend('off');

% disp([num2str(fL1.p1),' ',num2str(fL1.p2),' ',num2str(fL1.p3),' '...
%      ,num2str(fL1.p4),' ',num2str(fL1.p5),' ',num2str(fL1.p6)]);
%  
% disp([num2str(fL2.p1),' ',num2str(fL2.p2),' ',num2str(fL2.p3),' '...
%      ,num2str(fL2.p4),' ',num2str(fL2.p5),' ',num2str(fL2.p6)]);
%  
% disp([num2str(fR1.p1),' ',num2str(fR1.p2),' ',num2str(fR1.p3),' '...
%      ,num2str(fR1.p4),' ',num2str(fR1.p5),' ',num2str(fR1.p6)]);
%  
% disp([num2str(fR2.p1),' ',num2str(fR2.p2),' ',num2str(fR2.p3),' '...
%      ,num2str(fR2.p4),' ',num2str(fR2.p5),' ',num2str(fR2.p6)]);
%             
% disp(['PWM_L = ',num2str(fL2.p1),'*vL^3 + '...
%                 ,num2str(fL2.p2),'*vL^2 + '...
%                 ,num2str(fL2.p3),'*vL + '...
%                 ,num2str(fL2.p4),';']);
%             
% disp(['PWM_R = ',num2str(fR1.p1),'*vR^3 + '...
%                 ,num2str(fR1.p2),'*vR^2 + '...
%                 ,num2str(fR1.p3),'*vR + '...
%                 ,num2str(fR1.p4),';']);
%             
% disp(['PWM_R = ',num2str(fR2.p1),'*vR^3 + '...
%                 ,num2str(fR2.p2),'*vR^2 + '...
%                 ,num2str(fR2.p3),'*vR + '...
%                 ,num2str(fR2.p4),';']);