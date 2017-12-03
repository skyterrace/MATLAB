%rosinit('192.168.218.128')
pubvel = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
sublaser = rossubscriber('/scan');
subodom = rossubscriber('/odom');
pause(2)
for i=1:20
    %laser = sublaser.receive();
    %odom = subodom.receive();
    % Get laser and odometry sensor data
    [laserData, ~, poseData] = getSensorReadings(sublaser, subodom);
    %laser.Ranges
    %odom.Pose.Pose.Position
    laserData
    poseData
    velmsg = rosmessage(pubvel);
    velmsg.Linear.Z = 0;
    velmsg.Linear.Y = 0;
    velmsg.Linear.X = -0.5;
    velmsg.Angular.Z = 0.0;
    send(pubvel,velmsg);
    pause(1);
end

%laser = rossubscriber('/p3dx/laser/scan')
%scandata = receive(laser,10)
%rosshutdown()

    function [laserData, mindist, poseData] = getSensorReadings(sublaser, subodom)
        %getSensorReadings Wait for next laser reading and get most current odometry reading
        
        laserMsg = receive(sublaser);
        poseMsg = subodom.LatestMessage;
        
        [laserData, mindist] = processLaserMessage(laserMsg);
        poseData = processOdometryMessage(poseMsg);
    end
    
    function [laserData, mindist] = processLaserMessage(message)
        %processLaserMessage Process ROS message with laser scan data
        
        laserData = readCartesian(message) * [0 1; -1 0];
        scan = message.Ranges;
        scan = scan(100:540);
        mindist = min(scan(~isnan(scan)));
    end
    
    function poseData = processOdometryMessage(message)
        %processOdometryMessage Process ROS message with odometry data
        
        if isempty(message)
            poseData = [0 0 0];
            return;
        end
        
        pos = message.Pose.Pose;
        xpos = pos.Position.X;
        ypos = pos.Position.Y;
        quat = pos.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        theta = angles(1);
        poseData = [xpos, ypos, theta];
    end