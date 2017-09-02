rosinit
exampleHelperROSStartTfPublisher
tftree = rostf
pause(1);
tftree.AvailableFrames
mountToCamera = getTransform(tftree, 'mounting_point', 'camera_center');
mountToCameraTranslation = mountToCamera.Transform.Translation
quat = mountToCamera.Transform.Rotation
mountToCameraRotationAngles = rad2deg(quat2eul([quat.W quat.X quat.Y quat.Z]))
baseToMount = getTransform(tftree, 'robot_base', 'mounting_point');
baseToMountTranslation = baseToMount.Transform.Translation
baseToMountRotation = baseToMount.Transform.Rotation
waitForTransform(tftree, 'robot_base', 'camera_center');
pt = rosmessage('geometry_msgs/PointStamped');
pt.Header.FrameId = 'camera_center';
pt.Point.X = 3;
pt.Point.Y = 1.5;
pt.Point.Z = 0.2;
tfpt = transform(tftree, 'robot_base', pt)
tfpt.Point
robotToCamera = getTransform(tftree, 'robot_base', 'camera_center')
robotToCamera.Transform.Translation
robotToCamera.Transform.Rotation
tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
tfStampedMsg.ChildFrameId = 'wheel';
tfStampedMsg.Header.FrameId = 'robot_base';
tfStampedMsg.Transform.Translation.X = 0;
tfStampedMsg.Transform.Translation.Y = -0.2;
tfStampedMsg.Transform.Translation.Z = -0.3;

quatrot = axang2quat([0 1 0 deg2rad(30)])
tfStampedMsg.Transform.Rotation.W = quatrot(1);
tfStampedMsg.Transform.Rotation.X = quatrot(2);
tfStampedMsg.Transform.Rotation.Y = quatrot(3);
tfStampedMsg.Transform.Rotation.Z = quatrot(4);
tfStampedMsg.Header.Stamp = rostime('now');
sendTransform(tftree, tfStampedMsg)
tftree.AvailableFrames
exampleHelperROSStopTfPublisher
rosshutdown