% q = [-32;-2;0]; %initial pose
% 
% t = 0:0.15:20; %The simulation runs 20 secs
% 
% qr = zeros(3,size(t,2));
% ur = zeros(2,size(t,2));
% 
% for iter=1:size(t,2)
%     [qr(:,iter), ur(:,iter)] = desiredPath(t(iter));
% end
clear; clc;
t = 0:0.1:36; %The simulation runs 36 secs

qr = zeros(3,size(t,2));
for iter=1:size(t,2)
    [qr(:,iter)] = desiredPathCircle(t(iter));
end

l = 23.5; %Assume wheel base = 30cm
R = 100; %Distance to the rotational center = 100 cm

for iter=2:size(t,2)
    
    Vr = (R + l/2)*(qr(3,iter)-qr(3,iter-1)); 
    Vl = (R - l/2)*(qr(3,iter)-qr(3,iter-1));
    
    disp(['Left wheel must go ' , num2str(Vl) , ' cm in 0.1 seconds']);
    disp(['Right wheel must go ' , num2str(Vr) , ' cm in 0.1 seconds']);
end