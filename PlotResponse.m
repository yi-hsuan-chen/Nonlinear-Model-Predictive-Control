%%... control forces u = [de T]
fig(1) = figure(); 
subplot(211); ax(1) = gca;
p(1) = stairs(t,u_cl(:,1),'b'); hold on;
p(2:3) = plot(t,umax(1)*ones(length(t),1),'k--',t,umin(1)*ones(length(t),1),'k--');
ylabel('$f_1$ (N)');
subplot(212); ax(2) = gca;
p(4) = stairs(t,u_cl(:,2),'b'); hold on;
p(5:6) = plot(t,umax(2)*ones(length(t),1),'k--',t,umin(2)*ones(length(t),1),'k--');
ylabel('$f_2$ (N)');

%%... regulation performance
time = 0:dt:sim_time; xx = xx';
u = xx(:,1); w = xx(:,2); q = xx(:,3);
theta = xx(:,4); h = xx(:,5);

fig(2) = figure();
subplot(231); ax(3) = gca;
p(7) = plot(time,u,'b');
xlabel('time (sec)'); ylabel('$u$ (m/s)');
subplot(232); ax(4) = gca;
p(8) = plot(time,w,'b');
xlabel('time (sec)'); ylabel('$w$ (m/s)');
subplot(233); ax(5) = gca;
p(9) = plot(time,q,'b');
xlabel('time (sec)'); ylabel('$q$ (rad/s)')
subplot(234); ax(6) = gca;
p(10) = plot(time,theta,'b');
xlabel('time (sec)'); ylabel('$\theta$ (rad)')
subplot(235); ax(7) = gca;
p(11) = plot(time,h,'b');
xlabel('time (sec)'); ylabel('$h$ (m)')

buffer = 3;
set(p,'Linewidth',1);
set(p(1:3:10),'Linewidth',1.5)
set(p(13:end),'Linewidth',1.5)

set(ax(1:2),'XLim',[0 t(end)],'YLim',[umin(1)-buffer umax(1)+buffer]);
set(ax,'XGrid','on','YGrid','on');

for i = 1:length(ax)
    set(ax(i).XLabel,'Interpreter','latex');
    set(ax(i).YLabel,'Interpreter','latex');
%     RemovePlotWhiteArea(ax(i));
end

% for i = 1:length(fig)
%     print(fig(i),['Figure\' fig(i).Name],'-depsc');
% end