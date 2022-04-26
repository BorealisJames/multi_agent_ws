# Quick notes to implement Borealis Teaming Planner

// Publishers
mPhaseAndTimePublisher = mNh.advertise<mt_msgs::phaseAndTime>("/phase_and_time", 10);
mPosePublisher = mNh.advertise<mt_msgs::pose>("/system_pose_from_formation", 10);
mDirectionUtilityPublisher = mNh.advertise<mt_msgs::angleIndexAndUtility>("/direction_utility", 10);
mConvexRegion2DPublisher = mNh.advertise<mt_msgs::convexRegion2D>("/convex_region_2D", 10);
mConvexRegion3DPublisher = mNh.advertise<mt_msgs::convexRegion3D>("/convex_region_3D", 10);
mAssignedVirtualPosePublisher = mNh.advertise<geometry_msgs::PoseStamped>("/assigned_virtual_position", 10);
mAssignedVirtualPoseMapPublisher = mNh.advertise<mt_msgs::posevector>("/assigned_virtual_pose_map", 10);
mControlStatePublisher = mNh.advertise<std_msgs::Int8>("/control_state",10);

// Subscribers 
mGoalSubscriber = mNh.subscribe<mt_msgs::pose>("/goal", 10, &TeamingPlanner::goalCallback, this);
mHumanSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>("/human/system_pose", 10, &TeamingPlanner::humanSystemPoseCallback, this);
mSelfLocalPoseSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>("/system_pose", 10, &TeamingPlanner::mSelfSystemPoseCallback, this);
mSystemPointCloudSubscriber = mNh.subscribe<sensor_msgs::PointCloud>("/pointcloud", 10, &TeamingPlanner::systemPointCloudCallback, this);
mSystemDepthCameraSubscriber = mNh.subscribe<sensor_msgs::PointCloud2>("/depth_camera", 10, &TeamingPlanner::systemDepthCameraCallback, this);
mTaskSubscriber = mNh.subscribe<mt_msgs::mtTask>("/task", 10, &TeamingPlanner::taskCallback, this);

Adjusting for subscriber adjust 

mHumanSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>("/human/system_pose", 10, &TeamingPlanner::humanSystemPoseCallback, this);
mSelfLocalPoseSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>("/system_pose", 10, &TeamingPlanner::mSelfSystemPoseCallback, this);

mTaskSubscriber = mNh.subscribe<mt_msgs::mtTask>("/task", 10, &TeamingPlanner::taskCallback, this); <-- Change msg type to string type so its easier

to the one in borealis

Topic to give emmanuel, disable if see this still works or not
sub_desired_setpoint_topic: /uav1/teaming_planner/assigned_virtual_position
