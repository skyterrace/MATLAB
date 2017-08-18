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