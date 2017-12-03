%rosinit('192.168.127.128')
%rosinit('192.168.218.128')
[jointTauPub, jtMsg] = rospublisher('/iiwa/PositionJointInterface_trajectory_controller/command');
jointStateSub = rossubscriber('/iiwa/PositionJointInterface_trajectory_controller/state');
n=100;
%for i = 1:n
  jsMsg = receive(jointStateSub);
  jtMsg.JointNames = {'iiwa_joint_1', ...
                                   'iiwa_joint_2', ...
                                   'iiwa_joint_3', ...
                                   'iiwa_joint_4',...
                                   'iiwa_joint_5',...
                                   'iiwa_joint_6',...
                                   'iiwa_joint_7'};
% Point 1
tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = zeros(1,7);
tjPoint1.Velocities = zeros(1,7);
tjPoint1.TimeFromStart.Sec = 1;
tjPoint1.TimeFromStart.Nsec = 0;

% Point 2
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [-0.1415   -1.2593    2.9671   -2.0055    0.9582   -1.5051    0.1944];
tjPoint2.Velocities = zeros(1,7);
tjPoint2.TimeFromStart.Sec = 3;
tjPoint2.TimeFromStart.Nsec = 0;

% Point 3
tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = [-0.2894   -1.2503    2.9671   -1.8721    0.8343   -1.4460    0.1019];
tjPoint3.Velocities = zeros(1,7);
tjPoint3.TimeFromStart.Sec = 5;
tjPoint3.TimeFromStart.Nsec = 0;

% Point 4
tjPoint4 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint4.Positions = [-0.4370   -1.2590    2.9671   -1.6976    0.6953   -1.3327    0.0251];
tjPoint4.Velocities = zeros(1,7);
tjPoint4.TimeFromStart.Sec = 7;
tjPoint4.TimeFromStart.Nsec = 0;

% Point 5
tjPoint5 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint5.Positions = [-0.5388   -1.2856    2.9671   -1.5214    0.6069   -1.1885   -0.0473];
tjPoint5.Velocities = zeros(1,7);
tjPoint5.TimeFromStart.Sec = 9;
tjPoint5.TimeFromStart.Nsec = 0;

jtMsg.Points = [tjPoint1,tjPoint2,tjPoint3,tjPoint4,tjPoint5];

send(jointTauPub,jtMsg);
%rosshutdown