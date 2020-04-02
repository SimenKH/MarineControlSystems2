%plotting
hold off

nPos=logsout{9}.Values.data(:,1);
ePos=logsout{9}.Values.data(:,2);
dPos=logsout{9}.Values.data(:,3);
runTime=logsout{8}.Values.time;

nSetPos=logsout{3}.Values.data(:,1);
eSetPos=logsout{3}.Values.data(:,2);
dSetPos=logsout{3}.Values.data(:,3);

figure(1)
plot(ePos,nPos,'b')
hold on
plot(eSetPos,nSetPos,'g')
h=2400;
quiver(ePos(1:h:end),nPos(1:h:end),sin(dPos(1:h:end)),cos(dPos(1:h:end)),'r')
grid on
xlabel('East [m]')
ylabel('North [m]')


hold off

%{
for i=1:length(nPos)
    plot(ePos(1:i),nPos(1:i),'b')
    hold on
    quiver(ePos(i),nPos(i),sin(dPos(i)),cos(dPos(i)),'r')
    legend(num2str(runTime(i)));
    pause(0.00001);
    hold off
end
%}

%%
nSetPos=logsout{5}.Values.data(:,1);
eSetPos=logsout{5}.Values.data(:,2);
dSetPos=logsout{5}.Values.data(:,3);

figure(2)

subplot(3,1,1)
plot(runTime,nPos,'r')
hold on
grid on
plot(runTime,nSetPos,'b')
title('North Position')
xlabel('Time [s]')
ylabel('Position [m]')
legend('Actual pos.','Reference pos.')
legend('Location','best')

subplot(3,1,2)
hold on
grid on
plot(runTime,ePos,'r')
plot(runTime,eSetPos,'b')
title('East Position')
xlabel('Time [s]')
ylabel('Position [m]')
legend('Actual pos.','Reference pos.')
legend('Location','best')

subplot(3,1,3)
hold on
grid on
plot(runTime,dPos,'r')
plot(runTime,dSetPos,'b')
title('Heading')
xlabel('Time [s]')
ylabel('Heading [rad]')
legend('Actual heading','Reference heading')
legend('Location','best')
%%

%Velocities


nVel=logsout{7}.Values.data(:,1);
eVel=logsout{7}.Values.data(:,2);
dVel=logsout{7}.Values.data(:,3);

nSetVel=logsout{2}.Values.data(:,1);
eSetVel=logsout{2}.Values.data(:,2);
dSetVel=logsout{2}.Values.data(:,3);

figure(3)

subplot(3,1,1)
plot(runTime,nVel,'r')
hold on
grid on
plot(runTime,nSetVel,'b')
title('Surge Velocity')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Actual vel.','Reference vel.')
legend('Location','best')

% Bottom plot
subplot(3,1,2)
hold on
grid on
plot(runTime,eVel,'r')
plot(runTime,eSetVel,'b')
title('Sway Velocity')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Actual vel.','Reference vel.')
legend('Location','best')

subplot(3,1,3)
hold on
grid on
plot(runTime,dVel,'r')
plot(runTime,dSetVel,'b')
title('Yaw Angular Velocity')
xlabel('Time [s]')
ylabel('Heading [rad/s]')
legend('Actual ang. vel.','Reference ang. vel.')
legend('Location','best')
