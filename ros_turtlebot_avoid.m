  gains.goalTargeting = 100;          % Gain for desire to reach goal
  gains.forwardPath = 0;            % Gain for moving forward
  gains.continuousPath = 0;         % Gain for maintaining continuous path
  gains.obstacleAvoid = 5;        % Gain for avoiding obstacles
  
  timerHandles.pub = rospublisher('/mobile_base/commands/velocity'); % Set up publisher
  timerHandles.pubmsg = rosmessage('geometry_msgs/Twist');

  timerHandles.sublaser = rossubscriber('/scan');  % Set up subscribers
  timerHandles.subodom = rossubscriber('/odom');
  timerHandles.subbump = rossubscriber('/mobile_base/sensors/bumper_pointcloud');
  
  odomresetpub = rospublisher('/mobile_base/commands/reset_odometry');  % Reset odometry
  odomresetmsg = rosmessage('std_msgs/Empty');
  send(odomresetpub,odomresetmsg)
  pause(2);     % Wait until odometry is reset
  
  timerHandles.gains = gains;
  
  timer1 = timer('TimerFcn',{@exampleHelperTurtleBotObstacleTimer,timerHandles},'Period',0.1,'ExecutionMode','fixedSpacing');
  timer1.StopFcn = {@exampleHelperTurtleBotStopCallback};
  
  exampleHelperTurtleBotShowGrid(timerHandles.sublaser);
  
  start(timer1);
  while strcmp(timer1.Running, 'on')
    exampleHelperTurtleBotShowGrid(timerHandles.sublaser);
    pause(0.5);
  end
  disp('press any key to quit')
  pause
  delete(timer1)