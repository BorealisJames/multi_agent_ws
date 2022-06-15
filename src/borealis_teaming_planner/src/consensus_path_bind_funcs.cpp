#include "../include/teaming_planner/teaming_planner.h"

    // std::function<bool(int32_t& numberOfAgentsInTeam)> m_getNumberOfAgentsInTeam;
    // std::function<bool(int32_t& ownAgentID)> m_getOwnAgentID;
    // std::function<bool(std::vector<Common::Pose>& goTherePath)> m_getGoTherePath;

    // std::function<bool(std::unordered_map<int32_t, Common::PhaseAndTime>& phasesAndTimeRecordOfAgents)>  m_getPhasesAndTimeRecordOfAgents;
    // std::function<bool(const int32_t, const Common::PhaseAndTime& ownAgentPhaseAndTime)>  m_pubOwnPhaseAndTime;

    // std::function<void()>  m_clearAgentsPoseBuffer;
    // std::function<bool(const int32_t ownAgentID, const Common::Pose& ownAgentPose)>  m_pubOwnPoseFunc;
    // std::function<bool(Common::Pose& ownAgentPose)>  m_getOwnAgentPose;
    // std::function<bool(std::unordered_map<int32_t, Common::Pose>& agentsPose)>  m_getAgentsPose;

    // std::function<void()>m_clearAgentsPathAndWaypointProgressBuffer;
    // std::function<bool(std::unordered_map<int32_t, Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgress)> m_getAgentsPathAndWaypointProgress;
    // std::function<bool(const int32_t, const Common::PathAndWaypointProgress& goTherePathAndWaypointProgress)> m_pubOwnPathAndWaypointProgress;

    // std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentLidarPointCloud;
    // std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentCameraPointCloud;

    // std::function<void()>  m_clearAgentsPlannedPathBuffer;
    // std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath)> m_getAgentsPlannedPath;
    // std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownPlannedPath)> m_pubOwnPlannedPath;

    // std::function<void()>  m_clearAgentsProcessedPathOfAgentsBuffer;
    // std::function<bool(std::unordered_map<int32_t, std::unordered_map<int32_t, Common::PathAndCost>>& agentsProcessedPathOfAgents)> m_getAgentsProcessedPathOfAgents;
    // std::function<bool(const int32_t, const std::unordered_map<int32_t, Common::PathAndCost>& ownProcessedPathOfAgents)> m_pubOwnProcessedPathOfAgents;

    // std::function<void()> m_clearAgentsBestProcessedPathBuffer;
    // std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath)> m_getAgentsBestProcessedPath;
    // std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownBestProcessedPath)> m_pubOwnBestProcessedPath;

    // std::function<bool(const int32_t, const std::vector<Common::Pose>& processedGoTherePath)>  m_pubProcessedGoTherePath;
