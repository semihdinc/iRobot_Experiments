function [] = drawScene(q,qr)

h0 = subplot(2,2,[1 2]);
delete(h0);

h1 = subplot(2,2,3);
delete(h1);

h2 = subplot(2,2,4);
delete(h2);

p1x = 0;
p1y = 0;
p1z = 0;

P(:,1) = [p1x, p1y, p1z, 1];
P(:,2) = [p1x, p1y+8, p1z, 1];
P(:,3) = [p1x+8, p1y+4, p1z+3, 1];
P(:,4) = [p1x, p1y+4, p1z+8, 1];

ind = [1 2 3 1 4 3 2 4]; % index order to draw target

subplot(2,2,[1 2]);
plot3(P(1,ind),P(2,ind),P(3,ind),'color','red','linewidth',2);


text(P(1,1),P(2,1),P(3,1),'P_1');
text(P(1,2),P(2,2),P(3,2),'P_2');
text(P(1,3),P(2,3),P(3,3),'P_3');
text(P(1,4),P(2,4),P(3,4),'P_4');
   
%% ------Draw Robot Position---------------------------
RobotPose.x = q(1);
RobotPose.y = q(2);
RobotPose.z = q(3);
RobotPose.phi = q(4);
RobotPose.theta = q(5);
RobotPose.psi = q(6);

M = (EulerTrans(RobotPose));

M(1:3,4)=q(1:3);

q1 = [0;0;0;1];
q2 = [1;+1.7;1;1];
q3 = [1;-1.7;1;1];

target = [q1 q2 q3];

target_new = M * target;

hold on;

plot3(target_new(1,[1 2 1 3]), target_new(2,[1 2 1 3]),target_new(3,[1 2 1 3]),'color','blue','linewidth',2);
plot3(target_new(1,[2 3]), target_new(2,[2 3]),target_new(3,[2 3]),'color','red','linewidth',2,'LineStyle',':');



%% ------Draw Desired Robot Position---------------------------

DesRobotPose.x = qr(1);
DesRobotPose.y = qr(2);
DesRobotPose.z = qr(3);
DesRobotPose.phi = qr(4);
DesRobotPose.theta = qr(5);
DesRobotPose.psi = qr(6);

M = (EulerTrans(DesRobotPose));

M(1:3,4)=qr(1:3);

qr1 = [0;0;0;1];
qr2 = [1;+1.7;1;1];
qr3 = [1;-1.7;1;1];

target = [qr1 qr2 qr3];

target_new = M * target;

hold on;

plot3(target_new(1,[1 2 1 3]), target_new(2,[1 2 1 3]),target_new(3,[1 2 1 3]),'color','green','linewidth',2);
plot3(target_new(1,[2 3]), target_new(2,[2 3]),target_new(3,[2 3]),'color','yellow','linewidth',2,'LineStyle',':');
axis equal;
axis([-35 10 -2 8 0 8]);

xlabel('X');
ylabel('Y');
zlabel('Z');

%%

[L_p R_p] = getPixelsV4(q);

p = L_p;
target = [p(:,1) p(:,2) p(:,3) p(:,4) p(:,1) p(:,3) p(:,2) p(:,4)];
subplot(2,2,3);
plot(target(1,:), target(2,:),'color','red','linewidth',2);        
axis([0 1000 0 1000],'square');

text(p(1,1),p(2,1),'P_1');
text(p(1,2),p(2,2),'P_2');
text(p(1,3),p(2,3),'P_3');
text(p(1,4),p(2,4),'P_4');

p = R_p;
target = [p(:,1) p(:,2) p(:,3) p(:,4) p(:,1) p(:,3) p(:,2) p(:,4)];
subplot(2,2,4);
plot(target(1,:), target(2,:),'color','red','linewidth',2);        
axis([0 1000 0 1000],'square');
    
text(p(1,1),p(2,1),'P_1');
text(p(1,2),p(2,2),'P_2');
text(p(1,3),p(2,3),'P_3');
text(p(1,4),p(2,4),'P_4');
    
%%

[L_p R_p] = getPixelsV4(qr);

p = L_p;
target = [p(:,1) p(:,2) p(:,3) p(:,4) p(:,1) p(:,3) p(:,2) p(:,4)];
subplot(2,2,3);
hold on;
plot(target(1,:), target(2,:),'-.','color','green','linewidth',2);        
axis([0 1000 0 1000],'square');

text(p(1,1),p(2,1),'P_1');
text(p(1,2),p(2,2),'P_2');
text(p(1,3),p(2,3),'P_3');
text(p(1,4),p(2,4),'P_4');

p = R_p;
target = [p(:,1) p(:,2) p(:,3) p(:,4) p(:,1) p(:,3) p(:,2) p(:,4)];
subplot(2,2,4);
hold on;
plot(target(1,:), target(2,:),'-.','color','green','linewidth',2);        
axis([0 1000 0 1000],'square');
    
text(p(1,1),p(2,1),'P_1');
text(p(1,2),p(2,2),'P_2');
text(p(1,3),p(2,3),'P_3');
text(p(1,4),p(2,4),'P_4');


