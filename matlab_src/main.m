iter = 0;
save iter.mat iter

clear all
clear all
clear all
close all
iter = [];

global RBTSTATEMSG VelCONMSG GoalStateMSG
ROBOT_Configs =[];

ROS_connection
% Record the data
%track_data

display('0/3 ...waiting data...');
 while (isempty(RBTSTATEMSG))
     %display(VelCONMSG)
     pause(1/100)
 end
 display('1/3 RobotState Received');
  while (isempty(VelCONMSG))
     %display(VelCONMSG)
     pause(1/100)
 end
 display('2/3 RobotControl Received');
  while (isempty(GoalStateMSG))
     %display(VelCONMSG)
     pause(1/100)
 end
 display('3/3 GoalData Received');


% Determine actor velocity
RobotPosX = RBTSTATEMSG(1:3:end,:);
RobotPosY = RBTSTATEMSG(2:3:end,:);
RobotYaw =  RBTSTATEMSG(3:3:end,:);
RobotVelLinControl = VelCONMSG(1:2:end,:);
RobotVelAngControl = VelCONMSG(2:2:end,:);
GoalLPosX = GoalStateMSG(1:6:end,:);
GoalLPosY = GoalStateMSG(2:6:end,:);
GoalRPosX = GoalStateMSG(4:6:end,:);
GoalRPosY = GoalStateMSG(5:6:end,:);

GoalLeft = [mean(GoalLPosX);mean(GoalLPosY)];
GoalRight = [mean(GoalRPosX);mean(GoalRPosY)];

% Plotting
figure(1)
plot(RobotVelLinControl,'r')
title('velocity')
legend('linear velocity')
hold off

figure(2)
hold on
axis equal
plot(RobotPosX(:,1),RobotPosY(:,1))
plot(GoalLeft(1),GoalLeft(2),'or')
plot(GoalRight(1),GoalRight(2),'og')

save ELCON002 RBTSTATEMSG VelCONMSG GoalStateMSG

% Remove the subscriber from the node.
node.removeSubscriber(VelocityControl);
node.removeSubscriber(RobotState);