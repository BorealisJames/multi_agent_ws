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

// Published Assigned Pose of the Agent into the /t265_pose_frame
bool TeamingPlanner::pubAssignedPose(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose)
{
    bool status = true;

    status = (status && aAgentId == mSourceSegmentId);
    if (status)
    {
        geometry_msgs::PoseStamped tmp;
        tmp.header.frame_id = "/odom";
        
        tmp.pose.position.x = aAssignedVirtualPose.position.x;
        tmp.pose.position.y = aAssignedVirtualPose.position.y;
        tmp.pose.position.z = aAssignedVirtualPose.position.z;

        tf::Quaternion tQuat = tf::createQuaternionFromYaw(aAssignedVirtualPose.headingRad);

        tmp.pose.orientation.w = tQuat.getW();
        tmp.pose.orientation.x = tQuat.getX();
        tmp.pose.orientation.y = tQuat.getY();
        tmp.pose.orientation.z = tQuat.getZ();

        std::string systemFrame = "/odom";
        // std::string targetFrame = "uav" + std::to_string(mSourceSegmentId) + "/t265_pose_frame";

        ros::Time tm;
        std::string err_string;

        mAssignedVirtualPosePublisher.publish(tmp);

        // Transform from UWB to t265 frame manually
        if (mTransformMethod == 1)
        {
            // find vector diff 
            geometry_msgs::PoseStamped vector_diff;
            geometry_msgs::PoseStamped t265Assignedpose;
            geometry_msgs::PoseStamped uwbSystemPose;

            uwbSystemPose.pose.position.x = mSelfSystemPose.position.x;
            uwbSystemPose.pose.position.y = mSelfSystemPose.position.y;
            uwbSystemPose.pose.position.z = mSelfSystemPose.position.z;

            tf::Quaternion tmpQuat = tf::createQuaternionFromYaw(mSelfSystemPose.headingRad);
            uwbSystemPose.pose.orientation.w = tmpQuat.getW();
            uwbSystemPose.pose.orientation.x = tmpQuat.getX();
            uwbSystemPose.pose.orientation.y = tmpQuat.getY();
            uwbSystemPose.pose.orientation.z = tmpQuat.getZ();

            vector_diff = TeamingPlanner::subtractPoseStamped(uwbSystemPose, tmp);
            t265Assignedpose = TeamingPlanner::addPoseStamped(vector_diff, mSelft265SystemPose);
            mAssignedt265VirtualPosePublisher.publish(t265Assignedpose);
        }
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

//     pubPose(mSourceSegmentId, mSelfSystemPose);

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

    pubPose(mSourceSegmentId, mSelfSystemPose);

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

    pubPose(mSourceSegmentId, mSelfSystemPose);

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
    tmp.header.frame_id = sourceFrame;
    tmp.header.stamp = ros::Time::now();
    mVoxel_filter_cloudPublisher.publish(tmp);
    // tmp.header.stamp = aSystemPointCloud2->header.stamp; // The correct way

    try 
    {        
        mPointCloudTransformListener.waitForTransform(Common::Entity::SYSTEM_FRAME, sourceFrame, tmp.header.stamp, ros::Duration(0.3));
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

// Unused
void TeamingPlanner::systemDepthCameraCallback(const sensor_msgs::PointCloud2::ConstPtr& aSystemDepthCamera)
{
    sensor_msgs::PointCloud tmp;
    sensor_msgs::convertPointCloud2ToPointCloud(*aSystemDepthCamera, tmp);
    std::string sourceFrame=  "uav" + std::to_string(mSourceSegmentId) + "/camera_link";

    try 
    {
        mPointCloudTransformListener.waitForTransform(Common::Entity::SYSTEM_FRAME, sourceFrame, ros::Time::now(), ros::Duration(0.3));
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

// def pose_diff(pose_stamped_previous, pose_staped_now):
//     q1_inv = np.zeros(4)
//     q2 = np.zeros(4)
//     vector_diff = PoseStamped()

//     # position vector diff 
//     vector_diff.pose.position.x = pose_staped_now.pose.position.x - pose_stamped_previous.pose.position.x
//     vector_diff.pose.position.y = pose_staped_now.pose.position.y - pose_stamped_previous.pose.position.y
//     vector_diff.pose.position.z = pose_staped_now.pose.position.z - pose_stamped_previous.pose.position.z

//     q1_inv[0] = pose_stamped_previous.pose.orientation.x
//     q1_inv[1] = pose_stamped_previous.pose.orientation.y
//     q1_inv[2] = pose_stamped_previous.pose.orientation.z
//     q1_inv[3] = -pose_stamped_previous.pose.orientation.w # Negate for inverse

//     q2[0] = pose_staped_now.pose.orientation.x
//     q2[1] = pose_staped_now.pose.orientation.y
//     q2[2] = pose_staped_now.pose.orientation.z
//     q2[3] = pose_staped_now.pose.orientation.w

//     qr = quaternion_multiply(q2, q1_inv)

//     vector_diff.pose.orientation.x = qr[0]
//     vector_diff.pose.orientation.y = qr[1]
//     vector_diff.pose.orientation.z = qr[2]
//     vector_diff.pose.orientation.w = qr[3]

//     return vector_diff


geometry_msgs::PoseStamped TeamingPlanner::subtractPoseStamped(geometry_msgs::PoseStamped previous, geometry_msgs::PoseStamped current)
{
    geometry_msgs::PoseStamped vector_diff;
    tf2::Quaternion q1_inv;
    tf2::Quaternion q2;
    tf2::Quaternion qr;


    vector_diff.pose.position.x = current.pose.position.x - previous.pose.position.x;
    vector_diff.pose.position.y = current.pose.position.y - previous.pose.position.y;
    vector_diff.pose.position.z = current.pose.position.z - previous.pose.position.z;

    q1_inv.setX(previous.pose.orientation.x);
    q1_inv.setY(previous.pose.orientation.y);
    q1_inv.setZ(previous.pose.orientation.z);
    q1_inv.setW(-previous.pose.orientation.w); // Negative to invert it

    q2.setX(current.pose.orientation.x);
    q2.setY(current.pose.orientation.y);
    q2.setZ(current.pose.orientation.z);
    q2.setW(current.pose.orientation.w); // Negative to invert it

    qr = q2 * q1_inv;

    vector_diff.pose.orientation.x = qr.getX();
    vector_diff.pose.orientation.y = qr.getY();
    vector_diff.pose.orientation.z = qr.getZ();
    vector_diff.pose.orientation.w = qr.getW();

    return vector_diff;

}

geometry_msgs::PoseStamped TeamingPlanner::addPoseStamped(geometry_msgs::PoseStamped vector_pose, geometry_msgs::PoseStamped current)
{

    geometry_msgs::PoseStamped new_pose_stamped;
    tf2::Quaternion q1_rot;
    tf2::Quaternion q2_origin;
    tf2::Quaternion q_new;

    new_pose_stamped.pose.position.x = vector_pose.pose.position.x + current.pose.position.x;
    new_pose_stamped.pose.position.y = vector_pose.pose.position.y + current.pose.position.y;
    new_pose_stamped.pose.position.z = vector_pose.pose.position.z + current.pose.position.z;

    q2_origin.setX(current.pose.orientation.x);
    q2_origin.setY(current.pose.orientation.y);
    q2_origin.setZ(current.pose.orientation.z);
    q2_origin.setW(current.pose.orientation.w);

    q1_rot.setX(vector_pose.pose.orientation.x);
    q1_rot.setY(vector_pose.pose.orientation.y);
    q1_rot.setZ(vector_pose.pose.orientation.z);
    q1_rot.setW(vector_pose.pose.orientation.w);

    q_new = q1_rot * q2_origin;

    new_pose_stamped.pose.orientation.x = q_new.getX();
    new_pose_stamped.pose.orientation.y = q_new.getY();
    new_pose_stamped.pose.orientation.z = q_new.getZ();
    new_pose_stamped.pose.orientation.w = q_new.getW();

    return new_pose_stamped;
}

