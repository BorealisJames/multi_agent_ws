#include "../include/teaming_planner/teaming_planner.h"

// Functions decleration for the formation handler pointers

bool TeamingPlanner::pubPhaseAndTime_rf(const int32_t aAgentId, const DistributedFormation::Common::PhaseAndTime aPhaseAndTime)
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

        mPhaseAndTimePublisher_rf.publish(tmp);
    }
    else 
    {
        ROS_ERROR("[TeamingPlanner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubPose_rf(const int32_t aAgentId, const DistributedFormation::Common::Pose aPose)
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

        mPosePublisher_rf.publish(tmp);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubDirectionUtility_rf(const int32_t aAgentId, const DistributedFormation::Common::DirectionUtility aDirectionUtility)
{
    bool status = true;
    
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::angleIndexAndUtility tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.angleIndexAndUtility = aDirectionUtility.angleIndexAndUtility;

        mDirectionUtilityPublisher_rf.publish(tmp);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubConvexRegion2D_rf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion2D aConvexRegion2D)
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
        
        mConvexRegion2DPublisher_rf.publish(tmp);

        if (mDebugVerbose)
        {
            ROS_INFO("pubConvexRegion2D_rf Vector A column 1 Values: ");
            for (auto value : matrixACol1)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D_rf Vector A column 2 Values: ");
            for (auto value : matrixACol2)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D_rf Matrix A Values: ");

            ROS_INFO("pubConvexRegion2D_rf Vector B Values: ");
            for (auto value : matrixB)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion2D_rf Matrix B Values: ");
        }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubConvexRegion3D_rf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion3D aConvexRegion3D)
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
        
        mConvexRegion3DPublisher_rf.publish(tmp);

        if (mDebugVerbose)
        {
            ROS_INFO("pubConvexRegion3D_rf Vector A column 1 Values: ");
            for (auto value : matrixACol1)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D_rf Vector A column 2 Values: ");
            for (auto value : matrixACol2)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D_rf Vector A column 3 Values: ");
            for (auto value : matrixACol3)
            {
                ROS_INFO("%d ", value);
            }
            ROS_INFO("\n");

            ROS_INFO("pubConvexRegion3D_rf Vector B Values: ");
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
bool TeamingPlanner::pubAssignedPose_rf(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose)
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

        mAssignedVirtualPosePublisher_rf.publish(tmp);
    }

    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubAssignedPoseMap_rf(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedFormation::Common::Pose> aAssignedPoseMap)
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
            mAssignedVirtualPoseMapPublisher_rf.publish(tmpPoseVector);
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

bool TeamingPlanner::getPosesForFormationToTrack_rf(std::vector<DistributedFormation::Common::Pose>& posesForFormationToTrack)
{
    bool status(false);

    if (mTask.type == Common::Entity::MTTaskEnum::FOLLOW_ME )
    {
        if (!mHistoryOfHumanPoses_rf.empty())
        {
            posesForFormationToTrack = mHistoryOfHumanPoses_rf;
            status = true;
            mHistoryOfHumanPosesReceived = false;
        }
        else
        {
            status = false;
        }
    }
    
    if (mTask.type == Common::Entity::MTTaskEnum::GO_THERE)
    {
        if (!mProcessedGoTherePath.empty())
        {
            // mGoTherePath_cp
            // posesForFormationToTrack = ;
            posesForFormationToTrack = mProcessedGoTherePath;
            status = true;
        }
        else
        {
            status = false;
            ROS_ERROR("[Teaming Planner %d]: Agent ID No go there poses for formation to track! \n", mSourceSegmentId);
        }
    }

    return status;
}

bool TeamingPlanner::getPhaseAndTimeMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime>& phaseAndTimeMap)
{
    bool status = true;

    if (!mAgentsPhaseAndTimeMap_rf.empty())
    {
        // Check if there is any discepency between the agent vector and phaseSyncMap
        // for (auto phase_time : mAgentsPhaseAndTimeMap_rf)
        // {
        //     bool thisNumberExists = false;
        //     int number_to_erase; 
        //     for (auto number : mAgentsInTeamVector)
        //     {
        //         number_to_erase = number;
        //         if (phase_time.first ==  number)
        //         {
        //             thisNumberExists = true;
        //             break;
        //         }
        //     }
        //     if (!thisNumberExists)
        //     {
        //         mAgentsPhaseAndTimeMap_rf.erase(number_to_erase);
        //         ROS_WARN("Agent%d CP: mAgentsPhaseAndTimeMap_rf contains agent id %d but mAgentsInTeamVector does not! !", mSourceSegmentId, number_to_erase);
        //     }
        // }

        if (mAgentsPhaseAndTimeMap_rf.size() != mTeamSize)
        {
            ROS_WARN("[Teaming Planner]: Agent ID %d discrepency in mAgentsPhaseAndTimeMap_rf.size() = %d and  mTeamSize = %d detected!", mSourceSegmentId, mAgentsPhaseAndTimeMap_rf.size(), mTeamSize);
            for (auto agent : mAgentsPhaseAndTimeMap_rf)
            {
                ROS_INFO("mAgentsPhaseAndTimeMap_rf contains agent %d ", agent.first);
            }
        }

        phaseAndTimeMap = mAgentsPhaseAndTimeMap_rf;
    }
    else
    {
        // phaseAndTimeMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Phase and Time map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getPoseMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::Pose>& poseMap)
{
    bool status = true;

    if (!mAgentsPoseMap_rf.empty())
    {
        poseMap = mAgentsPoseMap_rf;
    }
    else
    {
        // poseMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Position map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getDirectionUtilityMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility>& directionUtilityMap)
{
    bool status = true;

    if (!mAgentsDirectionUtilityMap_rf.empty())
    {
        directionUtilityMap = mAgentsDirectionUtilityMap_rf;
    }
    else
    {
        // directionUtilityMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Direction Utility map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getConvexRegion2DMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>& convexRegion2DMap)
{
    bool status = true;

    if (!mAgentsConvexRegion2DMap_rf.empty())
    {
        convexRegion2DMap = mAgentsConvexRegion2DMap_rf;
    }
    else
    {
        // convexRegion2DMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Convex Region 2D map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getConvexRegion3DMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>& convexRegion3DMap)
{
    bool status = true;

    if (!mAgentsConvexRegion3DMap_rf.empty())
    {
        convexRegion3DMap = mAgentsConvexRegion3DMap_rf;
    }
    else
    {
        // convexRegion3DMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Convex Region 3D map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getAssignedVirtualPoseMap_rf(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>>& assignedVirtualPoseMap)
{
    bool status = true;

    if (!mAgentsAssignedVirtualPoseMap_rf.empty())
    {
        assignedVirtualPoseMap = mAgentsAssignedVirtualPoseMap_rf;
    }
    else
    {
        // assignedVirtualPoseMap.clear();

        ROS_WARN("[Teaming Planner %d]: Agents Assigned Virtual Pose map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getHumanSystemPose_rf(DistributedFormation::Common::Pose& aHumanSystemPose)
{
    bool status = true;

    if (status)
    {
        aHumanSystemPose = mHumanSystemPose_rf;
    }

    return status;
}

bool TeamingPlanner::getOwnUAVSystemPose_rf(DistributedFormation::Common::Pose& aUAVSystemPose)
{
    bool status = true;

    if (status)
    {
        aUAVSystemPose = mSelfSystemPose_rf;
    }

    return status;
}

void TeamingPlanner::clearPhaseAndTimeMap_rf()
{
    mAgentsPhaseAndTimeMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Phase Time Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearPoseMap_rf()
{
    mAgentsPoseMap_rf.clear();
    clearPhaseAndTimeMap_rf(); // also clear
    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Pose Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearDirectionUtilityMap_rf()
{
    mAgentsDirectionUtilityMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Direction Utility Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearConvexRegion2DMap_rf()
{
    mAgentsConvexRegion2DMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Convex Region 2D Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearConvexRegion3DMap_rf()
{
    mAgentsConvexRegion3DMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Convex Region 3D Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearAssignedVirtualPoseMap_rf()
{
    mAgentsAssignedVirtualPoseMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents AssignedVirtualPose Map cleared", mSourceSegmentId);
    }
}

