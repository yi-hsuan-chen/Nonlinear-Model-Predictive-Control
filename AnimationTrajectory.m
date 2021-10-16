% Define body frame
qlw = 1.5; len = 1;
fig = figure(); ax = gca;
xb = quiver3(0,0,0,len,0,0,'r-','LineWidth',qlw); hold on;
yb = quiver3(0,0,0,0,len,0,'g-','LineWidth',qlw);
zb = quiver3(0,0,0,0,0,len,'b-','LineWidth',qlw);
xlim([-10 10]); ylim([-10 10]); zlim([-5 15]);
% xlim([-5 22]); ylim([-5 5]); zlim([0 8]);
xlabel('$x$  (m)'); ylabel('$y$  (m)'); zlabel('$z$  (m)');
set(ax.XLabel,'Interpreter','latex');
set(ax.YLabel,'Interpreter','latex');
set(ax.ZLabel,'Interpreter','latex');
traj = plot3(x,y,z,'b:','LineWidth',1.2);
tt(1) = plot3(x,y,z,'b:','LineWidth',1.2);
tt(2) = plot3(xd,yd,zd,'r--','LineWidth',1.2);

%% Function handle of rotation matrices
DCM_Z = @(z) [ cos(z)   sin(z)     0;
              -sin(z)   cos(z)     0;
                 0        0        1];
             
DCM_Y = @(y) [ cos(y)     0     -sin(y);
                 0        1        0;
               sin(y)     0     cos(y)];       
           
DCM_X = @(x) [   1        0         0;
                 0      cos(x)   sin(x);
                 0     -sin(x)   cos(x)];
%% Draw Animation
if recordVideo == 1
    writerObj = VideoWriter(videoName,'MPEG-4');
    writerObj.FrameRate = 30;
    writerObj.Quality = 100;
    open(writerObj);
% text information
    H_axes = title(sprintf('Time = %.2f(s),  Roll = %.2f(deg),  Pitch = %.2f(deg),  Yaw = %.2f(deg)',...
            t, rad2deg(phi(i)), rad2deg(theta(i)), rad2deg(psi(i))),'Interpreter','latex'); 
end

divider = 8;
RT = zeros(4,4);
for i = 1:divider:length(t)
    T = [x(i) y(i) z(i)]';
    % Compute rotation matrix Cbv
    C1v = DCM_Z(psi(i));
    C21 = DCM_Y(theta(i));
    Cb2 = DCM_X(phi(i));
    Cbv = Cb2*C21*C1v;
    DCM = Cbv';
    RT = [ DCM T ];
    RT = [ RT; [ 0 0 0 1 ] ]; % homogeneous rotation matrix
    xr = RT*[1;0;0;0]; xr(4,:) = [];
    yr = RT*[0;1;0;0]; yr(4,:) = [];
    zr = RT*[0;0;1;0]; zr(4,:) = [];
    
    if recordVideo == 1
        set(H_axes,'String', sprintf('t = %.2f(s),  Roll = %.2f(deg),  Pitch = %.2f(deg),  Yaw = %.2f(deg)',...
                                     t(i), rad2deg(wrapToPi(phi(i))), rad2deg(wrapToPi(theta(i))), rad2deg(wrapToPi(psi(i)))));
        set(xb,'XData',x(i),'YData',y(i),'ZData',z(i),'UData',xr(1),'VData',xr(2),'WData',xr(3));
        set(yb,'XData',x(i),'YData',y(i),'ZData',z(i),'UData',yr(1),'VData',yr(2),'WData',yr(3));
        set(zb,'XData',x(i),'YData',y(i),'ZData',z(i),'UData',zr(1),'VData',zr(2),'WData',zr(3));
        set(traj,'XData',x(1:i),'YData',y(1:i),'ZData',z(1:i));
        drawnow;
    % Get Frame
        writeVideo(writerObj,getframe(fig));
    else
        quiver3(x(i),y(i),z(i),xr(1),xr(2),xr(3),'r-','LineWidth',qlw); hold on;
        quiver3(x(i),y(i),z(i),yr(1),yr(2),yr(3),'g-','LineWidth',qlw);
        quiver3(x(i),y(i),z(i),zr(1),zr(2),zr(3),'b-','LineWidth',qlw);
    end
end
% pstar = plot3(18,0,5,'Marker','p','MarkerSize',16,...
%     'MarkerEdgeColor','k',...
%     'MarkerFaceColor','y');
% legend(pstar,'target point','Interpreter','latex');
legend(tt,'Actual position','Desired position','Interpreter','latex');

if recordVideo == 1
    close(writerObj);
end