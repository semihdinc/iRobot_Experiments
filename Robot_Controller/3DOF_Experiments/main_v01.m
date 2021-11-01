% Main program that initializes variables and calls the SIMULINK model and
% plots the results
clear all;
clc;

addpath(genpath('Mirage'));
addpath(genpath('EPNP'));
addpath(genpath('POSIT'));

q0 = [-32;-5;0];

sim('Robot_Control_v01.mdl',25); % Run model

% %Trajectory Tracking Figure in 2D
% trajectoryTrackingFigure(q(:,1),q(:,2),qr(:,1),qr(:,2));
% set(gcf,'PaperUnits','inches','PaperPosition',[0 11 8 4]);
% print -depsc '..\..\Latex Sources\figures\trajectoryTracking_test.eps';

% for i=1:59
%     qi = [q(i,1) q(i,2) 0 0 0 q(i,3)];
%     qri = [qr(i,1) qr(i,2) 0 0 0 qr(i,3)];
% %     drawScene(qi,qri);
%     leftCamView(qi,qri);
%     drawnow;
% end

% set(gcf,'PaperUnits','inches','PaperPosition',[0 0 4 4]);
% print -depsc '..\..\Latex Sources\figures\x_test.eps';