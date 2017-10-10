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
tjPoint2.Positions = [0.0 0.5 0.1 0.0 0.0 0.0 0.0];
tjPoint2.Velocities = zeros(1,7);
tjPoint2.TimeFromStart.Sec = 4;
tjPoint2.TimeFromStart.Nsec = 0;

% Point 3
tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = zeros(1,7);
tjPoint3.Velocities = zeros(1,7);
tjPoint3.TimeFromStart.Sec = 7;
tjPoint3.TimeFromStart.Nsec = 0;

jtMsg.Points = [tjPoint1,tjPoint2,tjPoint3];

send(jointTauPub,jtMsg);
%rosshutdown