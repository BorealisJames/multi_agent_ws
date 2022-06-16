#include "../include/teaming_planner/teaming_planner.h"

bool TeamingPlanner::getGoTherePathCPH(std::vector<DistributedGlobalPathPlanner::Common::Pose>& goTherePath)
{
    bool status = true;

    if (!m_goTherePath.empty())
    {
        goTherePath = m_goTherePath;
    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get m_goTherePath empty!", mSourceSegmentId);
    }

    return status;
}

bool TeamingPlanner::getPhasesAndTimeRecordOfAgentsCPH(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PhaseAndTime>& phasesAndTimeRecordOfAgents)
{
    bool status = true;

    if (!m_phasesAndTimeRecordOfAgents.empty())
    {
        phasesAndTimeRecordOfAgents = m_phasesAndTimeRecordOfAgents;
    }
    else
    {
        ROS_WARN("Agent%d CPH: get m_phasesAndTimeRecordOfAgents empty!", mSourceSegmentId);
    }

    return status;
}

bool getOwnAgentPoseCPH(DistributedGlobalPathPlanner::Common::Pose& ownAgentPose)
{
    bool status = true;

    ownAgentPose = m_ownAgentPose;
    
    return true;
}


bool TeamingPlanner::pubOwnPhaseAndTimeCPH(const int32_t, const DistributedGlobalPathPlanner::Common::PhaseAndTime& ownAgentPhaseAndTime)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}

bool TeamingPlanner::pubOwnPoseFuncCPH(const int32_t ownAgentID, const DistributedGlobalPathPlanner::Common::Pose& ownAgentPose)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}

bool TeamingPlanner::getOtherAgentsPoseCPH(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::Pose>& agentsPose)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}


bool TeamingPlanner::getAgentsPathAndWaypointProgressCPH(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgress)
{
    bool status = true;

    if (!m_agentsPathAndWaypointProgress.empty())
    {
        agentsGoTherePathAndWaypointProgress = m_agentsPathAndWaypointProgress;
    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get m_agentsPathAndWaypointProgress empty!", mSourceSegmentId);
    }

    return status;
}


bool TeamingPlanner::pubOwnPathAndWaypointProgressCPH(const int32_t, const DistributedGlobalPathPlanner::Common::PathAndWaypointProgress& goTherePathAndWaypointProgress)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}


bool TeamingPlanner::getAgentsPlannedPathCPH(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath)
{
    bool status = true;

    if (!m_AgentsPlannedPath.empty())
    {
        agentsPlannedPath = m_AgentsPlannedPath;
    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get m_AgentsPlannedPath empty!", mSourceSegmentId);
    }
    return status;
}


bool TeamingPlanner::pubOwnPlannedPathCPH(const int32_t, const std::vector<Eigen::Vector3d>& ownPlannedPath)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}


bool TeamingPlanner::getAgentsProcessedPathOfAgentsCPH(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>>& agentsProcessedPathOfAgents)
{
    bool status = true;

    if (!m_AgentsProcessedPathOfAgents.empty())
    {
        agentsProcessedPathOfAgents = m_AgentsProcessedPathOfAgents;
    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get m_AgentsProcessedPathOfAgents empty!", mSourceSegmentId);
    }

    return status;
}


bool TeamingPlanner::pubOwnProcessedPathOfAgentsCPH(const int32_t, const std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>& ownProcessedPathOfAgents)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}


bool TeamingPlanner::getAgentsBestProcessedPathCPH(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath)
{
    bool status = true;

    if (!m_AgentsBestProcessedPath.empty())
    {
        agentsBestProcessedPath = m_AgentsBestProcessedPath;
    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get m_AgentsBestProcessedPath empty!", mSourceSegmentId);
    }

    return status;
}


bool TeamingPlanner::pubOwnBestProcessedPathCPH(const int32_t, const std::vector<Eigen::Vector3d>& ownBestProcessedPath)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}

bool TeamingPlanner::pubProcessedGoTherePathCPH(const int32_t, const std::vector<DistributedGlobalPathPlanner::Common::Pose>& processedGoTherePath)
{
    bool status = true;

    if (status)
    {

    }

    return status;
}


void TeamingPlanner::clearAgentsPoseBufferCPH()
{
    m_agentsPose.clear();
}

void TeamingPlanner::clearAgentsProcessedPathOfAgentsBufferCPH()
{
    m_AgentsProcessedPathOfAgents.clear();
}

void TeamingPlanner::clearAgentsPlannedPathBufferCPH()
{
    m_AgentsPlannedPath.clear();
}

void TeamingPlanner::clearAgentsPathAndWaypointProgressBufferCPH()
{
    m_agentsPathAndWaypointProgress.clear();
}