iter = 0;
save iter.mat iter

clear all
clear all
clear all
close all
iter = [];

global RBTSTATEMSG VelCONMSG
ROBOT_Configs =[];

ROS_connection
% Record the data
%track_data

display('waiting data');

 while (isempty(VelCONMSG))
     %display(VelCONMSG)
     pause(1/100)
 end
 display('data received');


% Determine actor velocity
RobotPosX = RBTSTATEMSG(1:3:end,:);
RobotPosY = RBTSTATEMSG(2:3:end,:);
RobotYaw =  RBTSTATEMSG(3:3:end,:);
RobotVelLinControl = VelCONMSG(1:2:end,:);
RobotVelAngControl = VelCONMSG(2:2:end,:);

% Plotting
figure(1)
plot(RobotVelLinControl,'r')
title('velocity')
legend('kalman','butt','velocity-estimated')
hold off

figure(2)
axis([-2 2 0 2])
hold on
plot(RobotPosX(:,1),RobotPosY(:,1))


% Remove the subscriber from the node.
node.removeSubscriber(VelocityControl);
node.removeSubscriber(RobotState);
