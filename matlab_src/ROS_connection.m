%% Init_ROS with MATLAB

%setenv('ROS_MASTER_URI','http://192.168.43.52:11311')
%setenv('ROS_IP','192.168.43.109')

% Create a new node named /NODE and connect it to the master.
node = rosmatlab.node('matlab', 'http://140.93.69.12:11311');
%node = rosmatlab.node('matlab', 'http://localhost:11311');

% Subscriber 
RobotState = node.addSubscriber('RobotState','std_msgs/Float64MultiArray',1);
RobotState.setOnNewMessageListeners({@robotStateMSG});

RobotControl = node.addSubscriber('VelocityControl','std_msgs/Float64MultiArray',1);
RobotControl.setOnNewMessageListeners({@VelControlMSG});

ROSCONN2 = node.addSubscriber('GoalState','std_msgs/Float64MultiArray',1);
ROSCONN2.setOnNewMessageListeners({@ROSCONN2});

