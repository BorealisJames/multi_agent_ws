#include "../include/teaming_planner/teaming_planner.h"

// Subscriber Functions
void TeamingPlanner::goalCallback(const mt_msgs::pose::ConstPtr& aGoal)
{
    DistributedFormation::Common::Pose pose;

    pose.position.x = aGoal->position.x;
    pose.position.y = aGoal->position.y;
    pose.position.z = aGoal->position.z;
    pose.headingRad = aGoal->headingRad;

    //ROS_INFO("[Teaming Planner %d]: Goal Message Received\n", mSourceSegmentId);
}

// void TeamingPlanner::taskCallback(const mt_msgs::mtTask::ConstPtr& aTask)
// {
//     mTask = *aTask;
    
//     mModuleState = TeamingPlannerConstants::ModuleState::RUNNING;
//     mModuleStateVerbose = false;
//     mModuleTaskVerbose = false;

//     //ROS_INFO("[Teaming Planner %d]: Task Message Received\n", mSourceSegmentId);
// }

// human array systempose callback input-> pose array stamped vector, convert to std::vector<DistributedFormation::Common::Pose> and assign it to mHistoryOfHumanPoses; 
// mHistoryOfHumanPoses
void TeamingPlanner::humanSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aHumanSystemPose)
{
    geometry_msgs::PoseStamped tmp_pose; 
    tmp_pose.header =  aHumanSystemPose->header;
    tmp_pose.pose =  aHumanSystemPose->pose.pose;

    Common::Entity::Pose tmp(tmp_pose);
    mHumanSystemPose.position.x = tmp.position.x;
    mHumanSystemPose.position.y = tmp.position.y;
    mHumanSystemPose.position.z = tmp.position.z;
    mHumanSystemPose.headingRad = tmp.yaw;

    // comment this guy out
    while(mHistoryOfHumanPoses.size() > (mPlanningHorizon/mIntervalDistance) - 1)
    {
        mHistoryOfHumanPoses.erase(mHistoryOfHumanPoses.begin());    
    }

    DistributedFormation::Common::Pose tPose;
    tPose.position.x = tmp.position.x;
    tPose.position.y = tmp.position.y;
    tPose.position.z = tmp.position.z;
    tPose.headingRad = tmp.yaw;

    if (checkAndAddHumanSystemPose(mHistoryOfHumanPoses, tPose))
    {
        mHistoryOfHumanPosesReceived = true;
    }
    else
    {
        // do nothing.
    }

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Human System Pose Received\n", mSourceSegmentId);
    }
    // comment this guy out
}

// void TeamingPlanner::selfSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& PoseWithCovarianceStamped)
// {
//     geometry_msgs::PoseStamped tmp_pose;
//     tmp_pose.header = PoseWithCovarianceStamped->header;
//     tmp_pose.pose = PoseWithCovarianceStamped->pose.pose;

//     Common::Entity::Pose tmp(tmp_pose);
//     mSelfSystemPose.position.x = tmp.position.x;
//     mSelfSystemPose.position.y = tmp.position.y;
//     mSelfSystemPose.position.z = tmp.position.z;
//     mSelfSystemPose.headingRad = tmp.yaw;

//     mAgentsPoseMap[mSourceSegmentId] = mSelfSystemPose;

//     pubPoseRFH(mSourceSegmentId, mSelfSystemPose);

//     // if (mDebugVerbose)
//     // {
//     //     ROS_INFO("[Teaming Planner %d]: Self System Pose Received\n", mSourceSegmentId);
//     // } 
// }

void TeamingPlanner::selfSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& PoseStamped)
{
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.header = PoseStamped->header;
    tmp_pose.pose = PoseStamped->pose;

    Common::Entity::Pose tmp(tmp_pose);
    mSelfSystemPose.position.x = tmp.position.x;
    mSelfSystemPose.position.y = tmp.position.y;
    mSelfSystemPose.position.z = tmp.position.z;
    mSelfSystemPose.headingRad = tmp.yaw;

    mAgentsPoseMap[mSourceSegmentId] = mSelfSystemPose;

    pubPoseRFH(mSourceSegmentId, mSelfSystemPose);

    // if (mDebugVerbose)
    // {
    //     ROS_INFO("[Teaming Planner %d]: Self System Pose Received\n", mSourceSegmentId);
    // } 
}

void TeamingPlanner::selfSystemPoseCallbackUWB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& PoseStampedCovar)
{
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.header = PoseStampedCovar->header;
    tmp_pose.pose = PoseStampedCovar->pose.pose;

    Common::Entity::Pose tmp(tmp_pose);
    mSelfSystemPose.position.x = tmp.position.x;
    mSelfSystemPose.position.y = tmp.position.y;
    mSelfSystemPose.position.z = tmp.position.z;
    mSelfSystemPose.headingRad = tmp.yaw;

    mAgentsPoseMap[mSourceSegmentId] = mSelfSystemPose;

    pubPoseRFH(mSourceSegmentId, mSelfSystemPose);

    // if (mDebugVerbose)
    // {
    //     ROS_INFO("[Teaming Planner %d]: Self System Pose Received\n", mSourceSegmentId);
    // } 
}


void TeamingPlanner::selft265SystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& PoseWithCovarianceStamped)
{
    geometry_msgs::PoseStamped tmp_pose;
    mSelft265SystemPose.header = PoseWithCovarianceStamped->header;
    mSelft265SystemPose.pose = PoseWithCovarianceStamped->pose; 
}


void TeamingPlanner::systemPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& aSystemPointCloud)
{
    std::string sourceFrame = "uav" + std::to_string(mSourceSegmentId) + "/os_sensor";
    // sensor_msgs::PointCloud tmpPointClud;
    // tmpPointClud = *aSystemPointCloud;
    // tmpPointClud->header.frame_id = "pseudo_lidar_local"; // change this
    mSystemPointCloud = *aSystemPointCloud;
    // try
    // {
    //     mPointCloudTransformListener.waitForTransform(Common::Entity::SYSTEM_FRAME, sourceFrame, aSystemPointCloud->header.stamp, ros::Duration(0.3));
    //     mPointCloudTransformListener.transformPointCloud(Common::Entity::SYSTEM_FRAME, *aSystemPointCloud, mSystemPointCloud);
    // }
    // catch (tf::TransformException ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    // }

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Point Cloud Received\n", mSourceSegmentId);
    }
}

// This will not be needed once grid map is published in pc1
void TeamingPlanner::systemPointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& aSystemPointCloud2)
{

    std::string sourceFrame = "uav" + std::to_string(mSourceSegmentId) + "/os_sensor";
    
    DistributedFormation::ProcessPointCloud tmpProcessPointCloud;
    sensor_msgs::PointCloud tmp;
    tmpProcessPointCloud.ApplyVoxelFilterAndConvertToPointCloud(*aSystemPointCloud2, tmp);
    tmp.header.stamp = aSystemPointCloud2->header.stamp; 
    tmp.header.frame_id = aSystemPointCloud2->header.frame_id; 

    // tmp.header.frame_id = sourceFrame;
    // tmp.header.stamp = ros::Time::now();

    mVoxel_filter_cloudPublisher.publish(tmp);

    try 
    {        
        mPointCloudTransformListener.waitForTransform(Common::Entity::SYSTEM_FRAME, sourceFrame, tmp.header.stamp, ros::Duration(0.5));
        mPointCloudTransformListener.transformPointCloud(Common::Entity::SYSTEM_FRAME, tmp, mSystemPointCloud);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Point Cloud Received\n", mSourceSegmentId);
    }
}

// Store into map.
void TeamingPlanner::systemPoseCallback(const mt_msgs::pose::ConstPtr& aSystemPose)
{
    DistributedFormation::Common::Pose tmp;
    tmp.position.x = aSystemPose->position.x;
    tmp.position.y = aSystemPose->position.y;
    tmp.position.z = aSystemPose->position.z;
    tmp.headingRad = aSystemPose->headingRad;

    mAgentsPoseMap[aSystemPose->sourceSegmentId] = tmp;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: System Pose Received from Agent: %d\n", mSourceSegmentId, aSystemPose->sourceSegmentId);
    }
}

void TeamingPlanner::phaseTimeCallback(const mt_msgs::phaseAndTime::ConstPtr& aPhaseAndTime)
{
    DistributedFormation::Common::PhaseAndTime tmp;

    tmp.phase = static_cast<DistributedFormation::Common::PHASE>(aPhaseAndTime->phase);
    tmp.timeMicroSecs = aPhaseAndTime->time;
    mAgentsPhaseAndTimeMap[aPhaseAndTime->sourceSegmentId] = tmp;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Phase Time Received from Agent: %d\n", mSourceSegmentId, aPhaseAndTime->sourceSegmentId);
    }
}

void TeamingPlanner::directionUtilityCallback(const mt_msgs::angleIndexAndUtility::ConstPtr& aDirectionUtility)
{
    DistributedFormation::Common::DirectionUtility tmp;

    tmp.angleIndexAndUtility = aDirectionUtility->angleIndexAndUtility;
    mAgentsDirectionUtilityMap[aDirectionUtility->sourceSegmentId] = tmp;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Direction Utility Received from Agent: %d\n", mSourceSegmentId, aDirectionUtility->sourceSegmentId);
    }
}

void TeamingPlanner::convexRegion2DCallback(const mt_msgs::convexRegion2D::ConstPtr& aConvexRegion2D)
{
    DistributedFormation::Common::ConvexRegion2D tmp;

    std::vector<double> vecACol1 = aConvexRegion2D->matrixACol1;
    std::vector<double> vecACol2 = aConvexRegion2D->matrixACol2;
    std::vector<double> vecB = aConvexRegion2D->matrixB;

    tmp.A.conservativeResize(vecACol1.size(), Eigen::NoChange);
    tmp.b.conservativeResize(vecB.size(), Eigen::NoChange);

    tmp.A.col(0) = Eigen::Map<Eigen::VectorXd>(&vecACol1[0], vecACol1.size());
    tmp.A.col(1) = Eigen::Map<Eigen::VectorXd>(&vecACol2[0], vecACol2.size());
    tmp.b = Eigen::Map<Eigen::VectorXd>(&vecB[0], vecB.size());

    mAgentsConvexRegion2DMap[aConvexRegion2D->sourceSegmentId] = tmp;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Convex Region Received from Agent: %d\n", mSourceSegmentId, aConvexRegion2D->sourceSegmentId);
    }

    if (mDebugVerbose)
    {
        ROS_INFO("ConvexRegion2DCallback Vector A column 1 Values: ");
        for (auto value : vecACol1)
        {
            ROS_INFO("%d ", value);
        }
        ROS_INFO("\n");

        ROS_INFO("ConvexRegion2DCallback Vector A column 2 Values: ");
        for (auto value : vecACol2)
        {
            ROS_INFO("%d ", value);
        }
        ROS_INFO("\n");

        ROS_INFO("ConvexRegion2DCallback Vector B Values: ");
        for (auto value : vecB)
        {
            ROS_INFO("%d ", value);
        }
        ROS_INFO("\n");
    }
}

void TeamingPlanner::convexRegion3DCallback(const mt_msgs::convexRegion3D::ConstPtr& aConvexRegion3D)
{
    DistributedFormation::Common::ConvexRegion3D tmp;

    std::vector<double> vecACol1 = aConvexRegion3D->matrixACol1;
    std::vector<double> vecACol2 = aConvexRegion3D->matrixACol2;
    std::vector<double> vecACol3 = aConvexRegion3D->matrixACol3;
    std::vector<double> vecB = aConvexRegion3D->matrixB;

    tmp.A.conservativeResize(vecACol1.size(), Eigen::NoChange);
    tmp.b.conservativeResize(vecB.size(), Eigen::NoChange);

    tmp.A.col(0) = Eigen::Map<Eigen::VectorXd>(&vecACol1[0], vecACol1.size());
    tmp.A.col(1) = Eigen::Map<Eigen::VectorXd>(&vecACol2[0], vecACol2.size());
    tmp.A.col(2) = Eigen::Map<Eigen::VectorXd>(&vecACol3[0], vecACol3.size());
    tmp.b = Eigen::Map<Eigen::VectorXd>(&vecB[0], vecB.size());

    mAgentsConvexRegion3DMap[aConvexRegion3D->sourceSegmentId] = tmp;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Convex Region Received from Agent: %d\n", mSourceSegmentId, aConvexRegion3D->sourceSegmentId);
    }

    if (mDebugVerbose)
    {
        ROS_INFO("ConvexRegion3DCallback Vector A column 1 Values: ");
        for (auto value : vecACol1)
        {
            ROS_INFO("%d ", value);
        }
        ROS_INFO("\n");

        ROS_INFO("ConvexRegion3DCallback Vector A column 2 Values: ");
        for (auto value : vecACol2)
        {
            ROS_INFO("%d ", value);
        }
        ROS_INFO("\n");

        ROS_INFO("ConvexRegion3DCallback Vector A column 3 Values: ");
        for (auto value : vecACol3)
        {
            ROS_INFO("%d ", value);
        }
        ROS_INFO("\n");

        ROS_INFO("ConvexRegion3DCallback Vector B Values: ");
        for (auto value : vecB)
        {
            ROS_INFO("%d ", value);
        }
        ROS_INFO("\n");
    }
}

void TeamingPlanner::assignedVirtualPoseMapCallback(const mt_msgs::posevector::ConstPtr& aAssignedVirtualPoseMap)
{
    std::unordered_map<int32_t, DistributedFormation::Common::Pose> tmpMap;
    for (auto pose : aAssignedVirtualPoseMap->poseVector)
    {
        DistributedFormation::Common::Pose tmp;
        tmp.position.x = pose.position.x;
        tmp.position.y = pose.position.y;
        tmp.position.z = pose.position.z;

        tmp.headingRad = pose.headingRad;

        
        tmpMap[pose.sourceSegmentId] = tmp;
    }
    mAgentsAssignedVirtualPoseMap[aAssignedVirtualPoseMap->sourceSegmentId] = tmpMap;

    if(mDebugVerbose)
    {
        ROS_INFO("Assigned Virtual Position Received from Agent: %d\n", aAssignedVirtualPoseMap->sourceSegmentId);
    }
}

void TeamingPlanner::gunCommandPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aPoseWithCovarianceStamped)
{
    // Update the Target pose
    mGunTargetPose.pose = aPoseWithCovarianceStamped->pose;
    mGunTargetPose.header = aPoseWithCovarianceStamped->header;
    mgunTargetPoseRecieved = true;
}

bool TeamingPlanner::switchToGunTargetPose(const int32_t aAgentId)
{
    bool status = true;
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        // Publish only when target pose is recieved
        if (mgunTargetPoseRecieved == true)
        {
            geometry_msgs::PoseStamped tmp;

            tmp.header.stamp = ros::Time::now();
            tmp.pose.orientation = mGunTargetPose.pose.pose.orientation;
            tmp.pose.position = mGunTargetPose.pose.pose.position;
            tmp.pose.position.z = mDesiredHeight;
            tmp.header.frame_id = "/odom";

            mAssignedVirtualPosePublisher.publish(tmp);
        }

    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

void TeamingPlanner::activatePlannerCallback(const std_msgs::Bool::ConstPtr& aBoolActivatePlanner)
{
    if (mModuleState == TeamingPlannerConstants::ModuleState::INITILAISING)
    {
        ROS_INFO("Activate planner failed, planner still initialising");
    }
    else
    {
        mBoolActivatePlanner.data = aBoolActivatePlanner->data;
        if(mBoolActivatePlanner.data)
        {
            mModuleState = TeamingPlannerConstants::ModuleState::RUNNING;
        }
        else
        {
            mModuleState = TeamingPlannerConstants::ModuleState::DEACTIVATED;
        }
    }
}

void TeamingPlanner::UAVModeCallback(const std_msgs::String::ConstPtr& aUAVmode)
{
    mUAVMode = aUAVmode->data;
    std::string str1 ("Follow_Me");
    std::string str2 ("Go_There");
    
    if (str1.compare(aUAVmode->data.c_str()) == 0) // They compare equal
    {
        mTask.type = Common::Entity::MTTaskEnum::FOLLOW_ME;
    }
    if (str2.compare(aUAVmode->data.c_str()) == 0)
    {
        mTask.type = Common::Entity::MTTaskEnum::GO_THERE;
    }
    ROS_INFO("Agent %i now in this mode %s", mSourceSegmentId, mUAVMode);
}