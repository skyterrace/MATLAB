urdffilename = fullfile(fileparts(which('LBRTorqueControlExample')), 'data', 'iiwa14.urdf');
lbr = importrobot(urdffilename);
lbr.DataFormat = 'row';
% Set the gravity to be the same as that in Gazebo.
lbr.Gravity = [0 0 -9.80];

% Show home configuration in MATLAB figure.
show(lbr);
view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);

wpfilename = fullfile(fileparts(which('LBRTorqueControlExample')), 'data', 'lbr_waypoints.mat');
load(wpfilename);

cdt = 0.001;
tt = 0:cdt:5;

% The trajectories are generated using |pchip| so that the interpolated
% joint position does not violate joint limits as long as the waypoints do not.
[qDesired, qdotDesired, qddotDesired, tt] = exampleHelperJointTrajectoryGeneration(tWaypoints, qWaypoints, tt);

n = size(qDesired,1);
tauFeedForward = zeros(n,7);
for i = 1:n
    tauFeedForward(i,:) = inverseDynamics(lbr, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
end

[jointTauPub, jtMsg] = rospublisher('/iiwa_matlab_plugin/iiwa_matlab_joint_effort');
jointStateSub = rossubscriber('/iiwa_matlab_plugin/iiwa_matlab_joint_state');

mdlConfigClient = rossvcclient('gazebo/set_model_configuration');

% Compose the required service message. It includes the joint names
% and corresponding joint positions to send to Gazebo. Call the service
% using this message.
msg = rosmessage(mdlConfigClient);
msg.ModelName = 'mw_iiwa';
msg.UrdfParamName = 'robot_description';
msg.JointNames = {'mw_iiwa_joint_1', 'mw_iiwa_joint_2', 'mw_iiwa_joint_3',...
                  'mw_iiwa_joint_4', 'mw_iiwa_joint_5', 'mw_iiwa_joint_6', 'mw_iiwa_joint_7'};
msg.JointPositions = homeConfiguration(lbr);

call(mdlConfigClient, msg)

weights = [0.3,0.8,0.6, 0.6,0.3,0.2,0.1];
Kp = 100*weights;
Kd = 2* weights;

once = 1;

feedForwardTorque = zeros(n, 7);
pdTorque = zeros(n, 7);
timePoints = zeros(n,1);
Q = zeros(n,7);
QDesired = zeros(n,7);

for i = 1:n
    % Get joint state from Gazebo.
    jsMsg = receive(jointStateSub);
    data = jsMsg.Data;

    % Parse the received message.
    % The Data in jsMsg is a 1-by-15 vector.
    % 1:7  - joint positions
    % 8:14 - joint velocities
    % 15   - time (Gazebo sim time) when the joint state is updated
    q = double(data(1:7))';
    qdot = double(data(8:14))';
    t = double(data(15));

    % Set the start time.
    if once
        tStart = t;
        once = 0;
    end

    % Find the corresponding index h in tauFeedForward vector for joint
    % state time stamp t.
    h = ceil((t - tStart + 1e-8)/cdt);
    if h>n
        break
    end

    % Log joint positions data.
    Q(i,:) = q';
    QDesired(i,:) = qDesired(h,:);

    % Inquire feed-forward torque at the time when the joint state is
    % updated (Gazebo sim time).
    tau1 = tauFeedForward(h,:);
    % Log feed-forward torque.
    feedForwardTorque(i,:) = tau1;

    % Compute PD compensation torque based on joint position and velocity
    % errors.
    tau2 = Kp.*(qDesired(h,:) - q) + Kd.*(qdotDesired(h,:) - qdot);
    % Log PD torque.
    pdTorque(i,:) = tau2';

    % Combine the two torques.
    tau = tau1 + tau2;

    % Log the time.
    timePoints(i) = t-tStart;

    % Send torque to Gazebo.
    jtMsg.Data = tau;
    send(jointTauPub,jtMsg);
end
exampleHelperLBRPlot(i-1, timePoints, feedForwardTorque, pdTorque, Q, QDesired )