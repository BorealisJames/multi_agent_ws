#include "../include/teaming_planner/teaming_planner.h"

TeamingPlanner::TeamingPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate):
        mNh(nh),
        mNhPrivate(nhPrivate),
        mConfigFileReader(),
        mModuleState(TeamingPlannerConstants::ModuleState::INITILAISING),
        mAgentsPhaseAndTimeMap(),
        mAgentsPoseMap(),
        mAgentsDirectionUtilityMap(),
        mAgentsConvexRegion2DMap(),
        mAgentsAssignedVirtualPoseMap(),
        mPoseTransformListener(),
        mPointCloudTransformListener(),
        mTask(),
        mHistoryOfHumanPoses(),
        mHistoryOfHumanPosesReceived(false),
        mModuleStateVerbose(false),
        mModuleTaskVerbose(false),

        mHandlerPtr(std::make_shared<DistributedFormation::DistributedMultiRobotFormationHandler>())

    {
        // Configured Variables
        mConfigFileReader.getParam(nhPrivate, "sourceSegmentId", mSourceSegmentId, static_cast<uint32_t>(0));
        mConfigFileReader.getParam(nhPrivate, "modulePeriod", mModulePeriod, 1);
        mConfigFileReader.getParam(nhPrivate, "numOfAgents", mNumOfAgents, 3);
        mConfigFileReader.getParam(nhPrivate, "debugVerbose", mDebugVerbose, false);
        mConfigFileReader.getParam(nhPrivate, "intervalDistance", mIntervalDistance, 0.5);
        mConfigFileReader.getParam(nhPrivate, "planningHorizon", mPlanningHorizon, 25);
        mConfigFileReader.getParam(nhPrivate, "desiredHeight", mDesiredHeight, 25);
        

        mgunTargetPoseRecieved = false;

        // Publishers
        mPhaseAndTimePublisher = mNh.advertise<mt_msgs::phaseAndTime>("/phase_and_time", 10);
        // mSelfSystemPosePublisher = mNh.advertise<geometry_msgs::PoseStamped>("/system_pose", 10);

        mPosePublisher = mNh.advertise<mt_msgs::pose>("/system_pose_from_formation", 10);
        mDirectionUtilityPublisher = mNh.advertise<mt_msgs::angleIndexAndUtility>("/direction_utility", 10);
        mConvexRegion2DPublisher = mNh.advertise<mt_msgs::convexRegion2D>("/convex_region_2D", 10);
        mConvexRegion3DPublisher = mNh.advertise<mt_msgs::convexRegion3D>("/convex_region_3D", 10);
        mAssignedVirtualPosePublisher = mNh.advertise<geometry_msgs::PoseStamped>("/assigned_virtual_position", 10);
        mAssignedVirtualPoseMapPublisher = mNh.advertise<mt_msgs::posevector>("/assigned_virtual_pose_map", 10);
        mControlStatePublisher = mNh.advertise<std_msgs::Int8>("/control_state",10);
        mVoxel_filter_cloudPublisher = mNh.advertise<sensor_msgs::PointCloud>("/voxel_filter_cloud",10);

        // Subscribers 
        mGoalSubscriber = mNh.subscribe<mt_msgs::pose>("/goal", 10, &TeamingPlanner::goalCallback, this);
        mHumanSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/human_input_pose", 10, &TeamingPlanner::humanSystemPoseCallback, this);
        mSelfSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/system_pose", 10, &TeamingPlanner::selfSystemPoseCallback, this);

        // Changed to point cloud 
        // mSystemPointCloud2Subscriber = mNh.subscribe<sensor_msgs::PointCloud2>("/pointcloud", 10, &TeamingPlanner::systemPointCloud2Callback, this);
        mSystemPointCloudSubscriber = mNh.subscribe<sensor_msgs::PointCloud>("/pointcloud", 10, &TeamingPlanner::systemPointCloudCallback, this);

        mTaskSubscriber = mNh.subscribe<mt_msgs::mtTask>("/task", 10, &TeamingPlanner::taskCallback, this);

        for (int i = 1; i <= mNumOfAgents; i++)
        {
            if (i == mSourceSegmentId)
            {
                // Subscribe to command pose giver (gun)
                std::string gunTargetPoseTopic = "/uav" + std::to_string(i) + "/command/pose";
                mGunTargetPoseSubscriber = mNh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(gunTargetPoseTopic, 10, &TeamingPlanner::gunCommandPoseCallback, this);
                continue;
            }

            std::string systemPoseTopic = "/uav" + std::to_string(i) + "/teaming_planner/system_pose";
            // std::string systemPoseTopic = "/UAV" + std::to_string(i) + "Pose";

            std::string phaseAndTimeTopic = "/uav" + std::to_string(i) + "/teaming_planner/phase_and_time";
            std::string directionUtilityTopic = "/uav" + std::to_string(i) + "/teaming_planner/direction_utility";
            std::string convexRegion2DTopic = "/uav" + std::to_string(i) + "/teaming_planner/convex_region_2D";
            std::string convexRegion3DTopic = "/uav" + std::to_string(i) + "/teaming_planner/convex_region_3D";
            std::string assignedVirtualPoseMapTopic = "/uav" + std::to_string(i) + "/teaming_planner/assigned_virtual_pose_map";

            ros::Subscriber systemPoseSubscriber = mNh.subscribe<mt_msgs::pose>(systemPoseTopic, 10, &TeamingPlanner::systemPoseCallback, this);
            ros::Subscriber phaseAndTimeSubscriber = mNh.subscribe<mt_msgs::phaseAndTime>(phaseAndTimeTopic, 10, &TeamingPlanner::phaseTimeCallback, this);
            ros::Subscriber directionUtilitySubscriber = mNh.subscribe<mt_msgs::angleIndexAndUtility>(directionUtilityTopic, 10, &TeamingPlanner::directionUtilityCallback, this);
            ros::Subscriber convexRegion2DSubscriber = mNh.subscribe<mt_msgs::convexRegion2D>(convexRegion2DTopic, 10, &TeamingPlanner::convexRegion2DCallback, this);
            ros::Subscriber convexRegion3DSubscriber = mNh.subscribe<mt_msgs::convexRegion3D>(convexRegion3DTopic, 10, &TeamingPlanner::convexRegion3DCallback, this);
            ros::Subscriber assignedVirtualPoseMapSubscriber = mNh.subscribe<mt_msgs::posevector>(assignedVirtualPoseMapTopic, 10, &TeamingPlanner::assignedVirtualPoseMapCallback, this);

            mUAVSystemPoseSubscriberVector.push_back(systemPoseSubscriber);
            mUAVPhaseAndTimeSubscriberVector.push_back(phaseAndTimeSubscriber);
            mUAVDirectionUtilitySubscriberVector.push_back(directionUtilitySubscriber);
            mUAVConvexRegion2DSubscriberVector.push_back(convexRegion2DSubscriber);
            mUAVConvexRegion3DSubscriberVector.push_back(convexRegion3DSubscriber);
            mUAVAssignedVirtualPoseMapSubscriberVector.push_back(assignedVirtualPoseMapSubscriber);
        }

        // Timers
        mModuleLoopTimer = mNh.createTimer(ros::Duration(mModulePeriod), &TeamingPlanner::moduleLoopCallback, this);
    }

TeamingPlanner::~TeamingPlanner()
{
    // Destructor
}
