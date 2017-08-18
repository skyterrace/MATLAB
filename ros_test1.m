%rosinit('192.168.1.105')
velpub = rospublisher('cmd_vel', 'geometry_msgs/Twist')
pause(2)
velmsg = rosmessage(velpub);
velmsg.Linear.Z = 0
velmsg.Linear.Y = 0
velmsg.Linear.X = 0.0
velmsg.Angular.Z = 0.2

send(velpub,velmsg)
%pause(2)

%laser = rossubscriber('/p3dx/laser/scan')
%scandata = receive(laser,10)