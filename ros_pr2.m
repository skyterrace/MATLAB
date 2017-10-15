%rosinit('192.168.218.128')
[jointTauPub, jtMsg] = rospublisher('/r_arm_controller/command');
jointStateSub = rossubscriber('/r_arm_controller/state');
jsMsg0 = receive(jointStateSub);
n=100;
%for i = 1:n
  jsMsg = receive(jointStateSub);
  jtMsg.JointNames = {'r_shoulder_pan_joint', ...
                                   'r_shoulder_lift_joint', ...
                                   'r_upper_arm_roll_joint', ...
                                   'r_elbow_flex_joint',...
                                   'r_forearm_roll_joint',...
                                   'r_wrist_flex_joint',...
                                   'r_wrist_roll_joint'};
% Point 1
tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = zeros(1,7);
tjPoint1.Velocities = zeros(1,7);
tjPoint1.TimeFromStart.Sec = 1;
tjPoint1.TimeFromStart.Nsec = 0;

% Point 2
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [-1.0 0.2 0.1 -1.2 -1.5 -0.3 -0.5];
tjPoint2.Velocities = [1.0 1.2 1.1 1.2 1.5 0.3 0.5];
tjPoint2.TimeFromStart.Sec = 2;
tjPoint2.TimeFromStart.Nsec = 0;

% Point 3
tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = zeros(1,7);
tjPoint3.Velocities = [1.0 1.2 1.1 1.2 1.5 0.3 0.5];%zeros(1,7);
tjPoint3.TimeFromStart.Sec = 3;
tjPoint3.TimeFromStart.Nsec = 0;

jtMsg.Points = [tjPoint1,tjPoint2,tjPoint3,tjPoint2];

send(jointTauPub,jtMsg);

%end
%jtMsg
%jtMsg = jsMsg0;
%send(jointTauPub,jtMsg);
%rosshutdown