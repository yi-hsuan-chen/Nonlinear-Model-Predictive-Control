%%... control forces u = [f1 f2 f3 f4]
fig(1) = figure(); 
subplot(411); ax(1) = gca;
p(1) = stairs(t,u_cl(:,1),'b'); hold on;
p(2:3) = plot(t,umax(1)*ones(length(t),1),'k--',t,umin(1)*ones(length(t),1),'k--');
ylabel('$f_1$ (N)');
subplot(412); ax(2) = gca;
p(4) = stairs(t,u_cl(:,2),'b'); hold on;
p(5:6) = plot(t,umax(2)*ones(length(t),1),'k--',t,umin(2)*ones(length(t),1),'k--');
ylabel('$f_2$ (N)');
subplot(413); ax(3) = gca;
p(7) = stairs(t,u_cl(:,3),'b'); hold on;
p(8:9) = plot(t,umax(3)*ones(length(t),1),'k--',t,umin(3)*ones(length(t),1),'k--');
ylabel('$f_3$ (N)')
subplot(414); ax(4) = gca;
p(10) = stairs(t,u_cl(:,4),'b'); hold on;
p(11:12) = plot(t,umax(4)*ones(length(t),1),'k--',t,umin(4)*ones(length(t),1),'k--');
xlabel('time (sec)'); ylabel('$f_4$ (N)')

%%... tracking performance
time = 0:dt:sim_time; xx = xx';
x = xx(:,1); y = xx(:,2); z = xx(:,3);
phi = xx(:,4); theta = xx(:,5); psi = xx(:,6);

fig(2) = figure();
subplot(231); ax(5) = gca;
p(13:14) = plot(time,x,'b',time,xd*ones(length(time),1),'r--');
xlabel('time (sec)'); ylabel('$x$ (m)');
subplot(232); ax(6) = gca;
p(15:16) = plot(time,y,'b',time,yd*ones(length(time),1),'r--');
xlabel('time (sec)'); ylabel('$y$ (m)');
subplot(233); ax(7) = gca;
p(17:18) = plot(time,z,'b',time,zd*ones(length(time),1),'r--');
xlabel('time (sec)'); ylabel('$z$ (m)')
subplot(234); ax(8) = gca;
p(19) = plot(time,phi,'b');
xlabel('time (sec)'); ylabel('$\phi$ (rad)')
subplot(235); ax(9) = gca;
p(20) = plot(time,theta,'b');
xlabel('time (sec)'); ylabel('$\theta$ (rad)')
subplot(236); ax(10) = gca;
p(21) = plot(time,psi,'b');
xlabel('time (sec)'); ylabel('$\psi$ (rad)');

buffer = 3;
set(p,'Linewidth',1);
set(p(1:3:10),'Linewidth',1.5)
set(p(13:end),'Linewidth',1.5)

set(ax(1:4),'XLim',[0 t(end)],'YLim',[umin(1)-buffer umax(1)+buffer]);
set(ax,'XGrid','on','YGrid','on');

for i = 1:length(ax)
    set(ax(i).XLabel,'Interpreter','latex');
    set(ax(i).YLabel,'Interpreter','latex');
%     RemovePlotWhiteArea(ax(i));
end

% for i = 1:length(fig)
%     print(fig(i),['Figure\' fig(i).Name],'-depsc');
% end