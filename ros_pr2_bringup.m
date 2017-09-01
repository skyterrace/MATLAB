[rArm, rGoalMsg] = rosactionclient('r_arm_controller/joint_trajectory_action');
waitForServer(rArm);
disp(rGoalMsg)
disp(rGoalMsg.Trajectory)
rGoalMsg.Trajectory.JointNames = {'r_shoulder_pan_joint', ...
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
tjPoint1.TimeFromStart = rosduration(1.0);

% Point 2
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [-1.0 0.2 0.1 -1.2 -1.5 -0.3 -0.5];
tjPoint2.Velocities = zeros(1,7);
tjPoint2.TimeFromStart = rosduration(2.0);

rGoalMsg.Trajectory.Points = [tjPoint1,tjPoint2];

sendGoalAndWait(rArm,rGoalMsg);

[lArm, lGoalMsg] = rosactionclient('l_arm_controller/joint_trajectory_action');
waitForServer(lArm);

lGoalMsg.Trajectory.JointNames = {'l_shoulder_pan_joint', ...
                                   'l_shoulder_lift_joint', ...
                                   'l_upper_arm_roll_joint', ...
                                   'l_elbow_flex_joint',...
                                   'l_forearm_roll_joint',...
                                   'l_wrist_flex_joint',...
                                   'l_wrist_roll_joint'};
tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = [1.0 0.2 -0.1 -1.2 1.5 -0.3 0.5];
tjPoint3.Velocities = zeros(1,7);
tjPoint3.TimeFromStart = rosduration(2.0);

lGoalMsg.Trajectory.Points = tjPoint3;

sendGoalAndWait(lArm,lGoalMsg);

pr2 = exampleHelperWGPR2Kinect;

jointSub = rossubscriber('joint_states');
jntState = receive(jointSub);
% Assign the joint positions from the joint states message to the fields of
% a configuration struct that the |pr2| object understands.
jntPos = exampleHelperJointMsgToStruct(pr2, jntState);
show(pr2,jntPos)
% In this example we will only be controlling the robot's arms. Therefore
% we place tight limits on the torso-lift joint during planning.
torsoJoint = pr2.getBody('torso_lift_link').Joint;
idx = strcmp({jntPos.JointName}, torsoJoint.Name);
torsoJoint.PositionLimits = jntPos(idx).JointPosition + [-1e-3,1e-3];

% Create the |InverseKinematics| object.
ik = robotics.InverseKinematics('RigidBodyTree', pr2);

% Disable random restart to ensure consistent IK solutions
ik.SolverParameters.AllowRandomRestart = false;

% Specify weights for the tolerances on each component of the goal pose.
weights = [0.25 0.25 0.25 1 1 1];
initialGuess = jntPos; % current jnt pos as initial guess

% Specify the name of the end effector
endEffectorName = 'r_gripper_tool_frame';

% Can's initial (current) pose and the desired final pose (There is a
% translation between the two poses along the table top)
TCanInitial = trvec2tform([0.7, 0.0, 0.55]);
TCanFinal = trvec2tform([0.6, -0.5, 0.55]);

% Define the desired relative transform between the end-effector and the can
% when grasping
TGraspToCan = trvec2tform([0,0,0.08])*eul2tform([pi/8,0,-pi]);

TGrasp = TCanInitial*TGraspToCan; % The desired end-effector pose when grasping the can
T1 = TGrasp*trvec2tform([0.,0,-0.1]);
T2 = TGrasp*trvec2tform([0,0,-0.2]);
T3 = TCanFinal*TGraspToCan*trvec2tform([0,0,-0.2]);
TRelease = TCanFinal*TGraspToCan; % The desired end-effector pose when releasing the can
T4 = T3*trvec2tform([-0.1,0,0]);

% Collection of tasks
motionTask = {'Release', T1, TGrasp, 'Grasp', T2, T3, TRelease, 'Release', T4};

% Execute each task specified in motionTask one by one
for i = 1: length(motionTask)
    disp(i)
    if strcmp(motionTask{i}, 'Grasp')
        exampleHelperSendPR2GripperCommand('right',0.0,1000,true);
        continue
    end

    if strcmp(motionTask{i}, 'Release')
        exampleHelperSendPR2GripperCommand('right',0.1,-1,true);
        continue
    end

    Tf = motionTask{i};
    % Get current joint state
    jntState = receive(jointSub);
    jntPos = exampleHelperJointMsgToStruct(pr2, jntState);

    T0 = getTransform(pr2, jntPos, endEffectorName);

    % Interpolating between key waypoints
    numWaypoints = 10;
    TWaypoints = exampleHelperSE3Trajectory(T0, Tf, numWaypoints); % end-effector pose waypoints
    jntPosWaypoints = repmat(initialGuess, numWaypoints, 1); % joint position waypoints

    rArmJointNames = rGoalMsg.Trajectory.JointNames;
    rArmJntPosWaypoints = zeros(numWaypoints, numel(rArmJointNames));

    % Calculate joint position for each end-effector pose waypoint using IK
    for k = 1:numWaypoints
        jntPos = ik(endEffectorName, TWaypoints(:,:,k), weights, initialGuess);
        jntPosWaypoints(k, :) = jntPos;
        initialGuess = jntPos;

        % Extract right arm joint positions from jntPos
        rArmJointPos = zeros(size(rArmJointNames));
        for n = 1:length(rArmJointNames)
            rn = rArmJointNames{n};
            idx = strcmp({jntPos.JointName}, rn);
            rArmJointPos(n) = jntPos(idx).JointPosition;
        end
        rArmJntPosWaypoints(k,:) = rArmJointPos';
    end

    % Time points corresponding to each waypoint
    timePoints = linspace(0,3,numWaypoints);

    % Estimate joint velocity trajectory numerically
    h = diff(timePoints); h = h(1);
    jntTrajectoryPoints = repmat(rosmessage('trajectory_msgs/JointTrajectoryPoint'),1,numWaypoints);
    [~, rArmJntVelWaypoints] = gradient(rArmJntPosWaypoints, h);
    for m = 1:numWaypoints
        jntTrajectoryPoints(m).Positions = rArmJntPosWaypoints(m,:);
        jntTrajectoryPoints(m).Velocities = rArmJntVelWaypoints(m,:);
        jntTrajectoryPoints(m).TimeFromStart = rosduration(timePoints(m));
    end

    % Visualize robot motion and end-effector trajectory in MATLAB(R)
    hold on
    for j = 1:numWaypoints
        show(pr2, jntPosWaypoints(j,:),'PreservePlot', false);
        exampleHelperShowEndEffectorPos(TWaypoints(:,:,j));
        drawnow;
        pause(0.1);
    end

    % Send the right arm trajectory to the robot
    rGoalMsg.Trajectory.Points = jntTrajectoryPoints;
    sendGoalAndWait(rArm, rGoalMsg);

end

exampleHelperSendPR2GripperCommand('r',0.0,-1)
rGoalMsg.Trajectory.Points = tjPoint2;
sendGoal(rArm, rGoalMsg);