#include "../include/teaming_planner/teaming_planner.h"

// Functions decleration for the formation handler pointer

bool TeamingPlanner::pubPose_mrf(const int32_t aAgentId, const DistributedFormation::Common::Pose aPose)
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

bool TeamingPlanner::pubDirectionUtility_mrf(const int32_t aAgentId, const DistributedFormation::Common::DirectionUtility aDirectionUtility)
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

bool TeamingPlanner::pubConvexRegion2D_mrf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion2D aConvexRegion2D)
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
            ROS_INFO("pubConvexRegion2D_mrf Vector A column 1 Values: ");
            for (auto value : matrixACol1)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D_mrf Vector A column 2 Values: ");
            for (auto value : matrixACol2)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D_mrf Matrix A Values: ");

            ROS_INFO("pubConvexRegion2D_mrf Vector B Values: ");
            for (auto value : matrixB)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D_mrf Matrix B Values: ");
        }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubConvexRegion3D_mrf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion3D aConvexRegion3D)
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
            ROS_INFO("pubConvexRegion3D_mrf Vector A column 1 Values: ");
            for (auto value : matrixACol1)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D_mrf Vector A column 2 Values: ");
            for (auto value : matrixACol2)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D_mrf Vector A column 3 Values: ");
            for (auto value : matrixACol3)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D_mrf Vector B Values: ");
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
bool TeamingPlanner::pubAssignedPose_mrf(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose)
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
    }

    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubAssignedPoseMap_mrf(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedFormation::Common::Pose> aAssignedPoseMap)
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

bool TeamingPlanner::getPosesForFormationToTrack_mrf(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses)
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

bool TeamingPlanner::getPoseMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::Pose>& poseMap)
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

bool TeamingPlanner::getDirectionUtilityMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility>& directionUtilityMap)
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

bool TeamingPlanner::getOwnAgentDepthCamera(sensor_msgs::PointCloud& depthCamera)
{
    bool status = true;
    depthCamera = mSystemDepthCamera;
    return status;
}

bool TeamingPlanner::getConvexRegion2DMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>& convexRegion2DMap)
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

bool TeamingPlanner::getConvexRegion3DMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>& convexRegion3DMap)
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

bool TeamingPlanner::getAssignedVirtualPoseMap_mrf(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>>& assignedVirtualPoseMap)
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

bool TeamingPlanner::getHumanSystemPose_mrf(DistributedFormation::Common::Pose& aHumanSystemPose)
{
    bool status = true;

    if (status)
    {
        aHumanSystemPose = mHumanSystemPose;
    }

    return status;
}

bool TeamingPlanner::getOwnUAVSystemPose_mrf(DistributedFormation::Common::Pose& aUAVSystemPose)
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

bool TeamingPlanner::getPhaseAndTimeMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime>& phaseAndTimeMap)
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

bool TeamingPlanner::pubPhaseAndTime_mrf(const int32_t aAgentId, const DistributedFormation::Common::PhaseAndTime aPhaseAndTime)
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
