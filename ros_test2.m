%rosinit('192.168.218.128')
[jointTauPub, jtMsg] = rospublisher('/iiwa_matlab_plugin/iiwa_matlab_joint_effort');
jointStateSub = rossubscriber('/iiwa_matlab_plugin/iiwa_matlab_joint_state');
jsMsg0 = receive(jointStateSub);
n=100;
for i = 1:n
  jsMsg = receive(jointStateSub);
  jtMsg.Data(1) = 100-i;
  %jtMsg.Data(2) = i/100;
  %jtMsg.Data(3) = i/100;
  %jtMsg.Data(4) = i/100;
  %jtMsg.Data(5) = i/10;
  %jtMsg.Data(6) = i/10;
  send(jointTauPub,jtMsg);

end
%jtMsg
%jtMsg = jsMsg0;
%send(jointTauPub,jtMsg);
%rosshutdown