function leftCamView(q,qr)

h = gca;
delete(h);

[L_p R_p] = getPixelsV4(q);

p = L_p;
target = [p(:,1) p(:,2) p(:,3) p(:,4) p(:,1) p(:,3) p(:,2) p(:,4)];

plot(target(1,:), target(2,:),'color','blue','linewidth',1);        
axis([0 550 0 550],'square');

text(p(1,1),p(2,1),'P_1');
text(p(1,2),p(2,2),'P_2');
text(p(1,3),p(2,3),'P_3');
text(p(1,4),p(2,4),'P_4');
  
%%

[L_p R_p] = getPixelsV4(qr);

p = L_p;
target = [p(:,1) p(:,2) p(:,3) p(:,4) p(:,1) p(:,3) p(:,2) p(:,4)];

hold on;
plot(target(1,:), target(2,:),'--','color','red','linewidth',1);        
axis([0 550 0 550],'square');

text(p(1,1),p(2,1),'P_1');
text(p(1,2),p(2,2),'P_2');
text(p(1,3),p(2,3),'P_3');
text(p(1,4),p(2,4),'P_4');

legend('Actual View','Desired View','Orientation','horizontal');
xlabel('x')
ylabel('y')

end


