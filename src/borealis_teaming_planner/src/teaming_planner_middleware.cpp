#include "../include/teaming_planner/teaming_planner.h"

// Publisher Functions
bool TeamingPlanner::pubPhaseAndTime(const int32_t aAgentId, const DistributedFormation::Common::PhaseAndTime aPhaseAndTime)
{
    bool status = true;

    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::phaseAndTime tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.phase = static_cast<uint8_t>(aPhaseAndTime.phase);
        tmp.time = aPhaseAndTime.timeMicroSecs;

        mPhaseAndTimePublisher.publish(tmp);
    }
    else 
    {
        ROS_ERROR("[TeamingPlanner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubPose(const int32_t aAgentId, const DistributedFormation::Common::Pose aPose)
{
    bool status = true;

    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::pose tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.position.x = aPose.position.x;
        tmp.position.y = aPose.position.y;
        tmp.position.z = aPose.position.z;
        tmp.headingRad = aPose.headingRad;

        mPosePublisher.publish(tmp);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubDirectionUtility(const int32_t aAgentId, const DistributedFormation::Common::DirectionUtility aDirectionUtility)
{
    bool status = true;
    
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::angleIndexAndUtility tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.angleIndexAndUtility = aDirectionUtility.angleIndexAndUtility;

        mDirectionUtilityPublisher.publish(tmp);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubConvexRegion2D(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion2D aConvexRegion2D)
{
    bool status = true;
    
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::convexRegion2D tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        Eigen::VectorXd matrixACol1Eigen = aConvexRegion2D.A.col(0);
        std::vector<double> matrixACol1(matrixACol1Eigen.data(), matrixACol1Eigen.data() + matrixACol1Eigen.size());
        tmp.matrixACol1 = matrixACol1;
        Eigen::VectorXd matrixACol2Eigen = aConvexRegion2D.A.col(1);
        std::vector<double> matrixACol2(matrixACol2Eigen.data(), matrixACol2Eigen.data() + matrixACol2Eigen.size());
        tmp.matrixACol2 = matrixACol2;
        std::vector<double> matrixB(aConvexRegion2D.b.data(), aConvexRegion2D.b.data() + aConvexRegion2D.b.size());
        tmp.matrixB = matrixB;
        
        mConvexRegion2DPublisher.publish(tmp);

        if (mDebugVerbose)
        {
            ROS_INFO("pubConvexRegion2D Vector A column 1 Values: ");
            for (auto value : matrixACol1)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D Vector A column 2 Values: ");
            for (auto value : matrixACol2)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D Matrix A Values: ");

            ROS_INFO("pubConvexRegion2D Vector B Values: ");
            for (auto value : matrixB)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D Matrix B Values: ");
        }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubConvexRegion3D(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion3D aConvexRegion3D)
{
    bool status = true;
    
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::convexRegion3D tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        Eigen::VectorXd matrixACol1Eigen = aConvexRegion3D.A.col(0);
        std::vector<double> matrixACol1(matrixACol1Eigen.data(), matrixACol1Eigen.data() + matrixACol1Eigen.size());
        tmp.matrixACol1 = matrixACol1;
        Eigen::VectorXd matrixACol2Eigen = aConvexRegion3D.A.col(1);
        std::vector<double> matrixACol2(matrixACol2Eigen.data(), matrixACol2Eigen.data() + matrixACol2Eigen.size());
        tmp.matrixACol2 = matrixACol2;
        Eigen::VectorXd matrixACol3Eigen = aConvexRegion3D.A.col(2);
        std::vector<double> matrixACol3(matrixACol3Eigen.data(), matrixACol3Eigen.data() + matrixACol3Eigen.size());
        tmp.matrixACol3 = matrixACol3;
        std::vector<double> matrixB(aConvexRegion3D.b.data(), aConvexRegion3D.b.data() + aConvexRegion3D.b.size());
        tmp.matrixB = matrixB;
        
        mConvexRegion3DPublisher.publish(tmp);

        if (mDebugVerbose)
        {
            ROS_INFO("pubConvexRegion3D Vector A column 1 Values: ");
            for (auto value : matrixACol1)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D Vector A column 2 Values: ");
            for (auto value : matrixACol2)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D Vector A column 3 Values: ");
            for (auto value : matrixACol3)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D Vector B Values: ");
            for (auto value : matrixB)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");
        }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubAssignedPose(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose)
{
    bool status = true;

    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        geometry_msgs::PoseStamped tmp;
        geometry_msgs::PoseStamped tmp_local;

        const ros::Time refTime = ros::Time::now();

        tmp.header.frame_id = "/odom";
        tmp.header.stamp = refTime;
        tmp_local.header.frame_id = "/odom";

        tmp.pose.position.x = aAssignedVirtualPose.position.x;
        tmp.pose.position.y = aAssignedVirtualPose.position.y;
        tmp.pose.position.z = aAssignedVirtualPose.position.z;

        tf::Quaternion tQuat = tf::createQuaternionFromYaw(aAssignedVirtualPose.headingRad);

        tmp.pose.orientation.w = tQuat.getW();
        tmp.pose.orientation.x = tQuat.getX();
        tmp.pose.orientation.y = tQuat.getY();
        tmp.pose.orientation.z = tQuat.getZ();

        std::string systemFrame = "/odom";
        // uav2/t265_odom_frame
        std::string targetFrame = "uav" + std::to_string(mSourceSegmentId) + "/t265_pose_frame";

        if(!mPoseTransformListener.waitForTransform(targetFrame,systemFrame,refTime ,ros::Duration(0.7)))
        {
            ROS_WARN("Wait for transform timed out, using last available transform instead.");
        }

        mPoseTransformListener.transformPose(targetFrame,tmp,tmp_local);
        // need to do a transform from system to local frame. 
        tmp_local.header.frame_id = targetFrame;
        tmp_local.header.stamp = refTime;
        mAssignedVirtualPosePublisher.publish(tmp_local);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubAssignedPoseMap(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedFormation::Common::Pose> aAssignedPoseMap)
{
    bool status = true;

    status = status && aAgentId == mSourceSegmentId;

    if (status)
    {
        mt_msgs::posevector tmpPoseVector;
        tmpPoseVector.header.stamp = ros::Time::now();
        tmpPoseVector.sourceSegmentId = mSourceSegmentId;
        
        for (auto assignedPose : aAssignedPoseMap)
        {
            mt_msgs::pose tmpPose;
            tmpPose.position.x = assignedPose.second.position.x;
            tmpPose.position.y = assignedPose.second.position.y;
            tmpPose.position.z = assignedPose.second.position.z;

            tmpPose.headingRad = assignedPose.second.headingRad;
            
            tmpPose.sourceSegmentId = assignedPose.first;

            tmpPose.header.stamp = ros::Time::now();

            tmpPoseVector.poseVector.emplace_back(tmpPose);
        }

        if (!tmpPoseVector.poseVector.empty())
        {
            mAssignedVirtualPoseMapPublisher.publish(tmpPoseVector);
            status = status && true;
        }
        else
        {
            status = false;
            ROS_WARN("[Teaming Planner %d]: Assigned Virtual Pose Map empty\n", mSourceSegmentId);

        }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubControlState(const Common::Entity::ControlState aControlState)
{
    bool status = true;
    std_msgs::Int8 tmp;
    tmp.data = static_cast<int8_t>(aControlState);

    mControlStatePublisher.publish(tmp);
    return status;
}

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

void TeamingPlanner::taskCallback(const mt_msgs::mtTask::ConstPtr& aTask)
{
    mTask = *aTask;
    
    mModuleState = TeamingPlannerConstants::ModuleState::RUNNING;
    mModuleStateVerbose = false;
    mModuleTaskVerbose = false;

    //ROS_INFO("[Teaming Planner %d]: Task Message Received\n", mSourceSegmentId);
}

void TeamingPlanner::humanSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aHumanSystemPose)
{
    Common::Entity::Pose tmp(*aHumanSystemPose);
    mHumanSystemPose.position.x = tmp.position.x;
    mHumanSystemPose.position.y = tmp.position.y;
    mHumanSystemPose.position.z = tmp.position.z;
    mHumanSystemPose.headingRad = tmp.yaw;
    ROS_INFO("aHuman system quat is x:%f, y:%f, z:%f, w:%f. ", aHumanSystemPose->pose.orientation.x, aHumanSystemPose->pose.orientation.y, aHumanSystemPose->pose.orientation.z, aHumanSystemPose->pose.orientation.w) ;
    ROS_INFO("mHuman system pose heading is x:%f, y:%f, z:%f, heading:%f. ", mHumanSystemPose.position.x, mHumanSystemPose.position.y, mHumanSystemPose.position.z, mHumanSystemPose.headingRad);
    ROS_INFO("tmp yaw is %f", tmp.yaw);

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
}

void TeamingPlanner::mSelfLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aSelfLocalPose)
{

    geometry_msgs::PoseStamped tmpSelfSystemPose; 
    geometry_msgs::PoseStamped tmpSelfLocalPose = *aSelfLocalPose; 

    std::string systemFrame = "/odom";
    // uav2/t265_odom_frame
    std::string targetFrame = "uav" + std::to_string(mSourceSegmentId) + "/t265_pose_frame";

    tmpSelfLocalPose.header.frame_id = systemFrame;

    if(!mPoseTransformListener.waitForTransform(targetFrame,systemFrame, aSelfLocalPose->header.stamp ,ros::Duration(0.7)))
    {
        ROS_WARN("Wait for transform timed out, using last available transform instead.");
    }

    mPoseTransformListener.transformPose(targetFrame,tmpSelfLocalPose,tmpSelfSystemPose);
    mSelfSystemPosePublisher.publish(tmpSelfSystemPose); // Publish own pose in ROS format

    Common::Entity::Pose tmp(tmpSelfSystemPose);
    mSelfSystemPose.position.x = tmp.position.x;
    mSelfSystemPose.position.y = tmp.position.y;
    mSelfSystemPose.position.z = tmp.position.z;
    mSelfSystemPose.headingRad = tmp.yaw;

    mAgentsPoseMap[mSourceSegmentId] = mSelfSystemPose;

    pubPose(mSourceSegmentId, mSelfSystemPose);

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Self System Pose Received\n", mSourceSegmentId);
    }    
}

void TeamingPlanner::systemPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& aSystemPointCloud)
{
    std::string sourceFrame = "uav" + std::to_string(mSourceSegmentId) + "/os_lidar";
    
    try
    {
        mPointCloudTransformListener.waitForTransform(Common::Entity::SYSTEM_FRAME, sourceFrame, ros::Time::now(), ros::Duration(0.7));
        mPointCloudTransformListener.transformPointCloud(Common::Entity::SYSTEM_FRAME, *aSystemPointCloud, mSystemPointCloud);
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

void TeamingPlanner::systemPointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& aSystemPointCloud2)
{

    std::string sourceFrame = "uav" + std::to_string(mSourceSegmentId) + "/os_lidar";
    
    DistributedFormation::ProcessPointCloud tmpProcessPointCloud;
    sensor_msgs::PointCloud tmp;
    tmpProcessPointCloud.ApplyVoxelFilterAndConvertToPointCloud(*aSystemPointCloud2, tmp);
    mVoxel_filter_cloudPublisher.publish(tmp);
    tmp.header.frame_id = sourceFrame;
    tmp.header.stamp = ros::Time::now();

    try 
    {
        mPointCloudTransformListener.waitForTransform(Common::Entity::SYSTEM_FRAME, sourceFrame, tmp.header.stamp, ros::Duration(0.7));
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


void TeamingPlanner::systemDepthCameraCallback(const sensor_msgs::PointCloud2::ConstPtr& aSystemDepthCamera)
{
    sensor_msgs::PointCloud tmp;
    sensor_msgs::convertPointCloud2ToPointCloud(*aSystemDepthCamera, tmp);
    std::string sourceFrame=  "uav" + std::to_string(mSourceSegmentId) + "/camera_link";

    try 
    {
        mPointCloudTransformListener.waitForTransform(Common::Entity::SYSTEM_FRAME, sourceFrame, ros::Time::now(), ros::Duration(0.7));
        mPointCloudTransformListener.transformPointCloud(Common::Entity::SYSTEM_FRAME, tmp, mSystemDepthCamera);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Depth Camera Received\n", mSourceSegmentId);
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

// Get Functions
bool TeamingPlanner::getOwnAgentId(int32_t& ownAgentID)
{
    bool status = true;

    ownAgentID = mSourceSegmentId;

    return status;
}

bool TeamingPlanner::getHistoryOfHumanPoses(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses)
{
    bool status(false);

    if (!mHistoryOfHumanPoses.empty())
    {
        historyOfHumanPoses = mHistoryOfHumanPoses;
        status = true;
        mHistoryOfHumanPosesReceived = false;
    }
    else
    {
        status = false;
    }

    return status;
}

bool TeamingPlanner::getPhaseAndTimeMap(std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime>& phaseAndTimeMap)
{
    bool status = true;

    if (!mAgentsPhaseAndTimeMap.empty())
    {
        phaseAndTimeMap = mAgentsPhaseAndTimeMap;
    }
    else
    {
        // phaseAndTimeMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Phase and Time map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getPoseMap(std::unordered_map<int32_t, DistributedFormation::Common::Pose>& poseMap)
{
    bool status = true;

    if (!mAgentsPoseMap.empty())
    {
        poseMap = mAgentsPoseMap;
    }
    else
    {
        // poseMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Position map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getDirectionUtilityMap(std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility>& directionUtilityMap)
{
    bool status = true;

    if (!mAgentsDirectionUtilityMap.empty())
    {
        directionUtilityMap = mAgentsDirectionUtilityMap;
    }
    else
    {
        // directionUtilityMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Direction Utility map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getOwnAgentLidarPointCloud(sensor_msgs::PointCloud& cloud)
{

    bool status = false;
    if (!mSystemPointCloud.points.empty())
    {
        cloud = mSystemPointCloud;
        status = true;
    }
    else
    {
        ROS_WARN("[Teaming Planner %d]: Agents Point Cloud empty", mSourceSegmentId);
        status = false;
    }
    
    return status;
}

bool TeamingPlanner::getOwnAgentDepthCamera(sensor_msgs::PointCloud& depthCamera)
{
    bool status = true;
    depthCamera = mSystemDepthCamera;
    return status;
}

bool TeamingPlanner::getConvexRegion2DMap(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>& convexRegion2DMap)
{
    bool status = true;

    if (!mAgentsConvexRegion2DMap.empty())
    {
        convexRegion2DMap = mAgentsConvexRegion2DMap;
    }
    else
    {
        // convexRegion2DMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Convex Region 2D map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getConvexRegion3DMap(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>& convexRegion3DMap)
{
    bool status = true;

    if (!mAgentsConvexRegion3DMap.empty())
    {
        convexRegion3DMap = mAgentsConvexRegion3DMap;
    }
    else
    {
        // convexRegion3DMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Convex Region 3D map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getAssignedVirtualPoseMap(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>>& assignedVirtualPoseMap)
{
    bool status = true;

    if (!mAgentsAssignedVirtualPoseMap.empty())
    {
        assignedVirtualPoseMap = mAgentsAssignedVirtualPoseMap;
    }
    else
    {
        // assignedVirtualPoseMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Assigned Virtual Pose map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getHumanSystemPose(DistributedFormation::Common::Pose& aHumanSystemPose)
{
    bool status = true;

    if (status)
    {
        aHumanSystemPose = mHumanSystemPose;
    }

    return status;
}

bool TeamingPlanner::getOwnUAVSystemPose(DistributedFormation::Common::Pose& aUAVSystemPose)
{
    bool status = true;

    if (status)
    {
        aUAVSystemPose = mSelfSystemPose;
    }

    return status;
}


void TeamingPlanner::clearPhaseAndTimeMap()
{
    mAgentsPhaseAndTimeMap.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Phase Time Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearPoseMap()
{
    mAgentsPoseMap.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Pose Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearDirectionUtilityMap()
{
    mAgentsDirectionUtilityMap.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Direction Utility Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearConvexRegion2DMap()
{
    mAgentsConvexRegion2DMap.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Convex Region 2D Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearConvexRegion3DMap()
{
    mAgentsConvexRegion3DMap.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Convex Region 3D Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearAssignedVirtualPoseMap()
{
    mAgentsAssignedVirtualPoseMap.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents AssignedVirtualPose Map cleared", mSourceSegmentId);
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
            ROS_INFO("Publishing tmp pose: %i", tmp.pose.position.x);
            mAssignedVirtualPosePublisher.publish(tmp);
        }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;

}
