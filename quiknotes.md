
    // std::function<bool(const int32_t, const Common::PhaseAndTime& ownAgentPhaseAndTime)>  m_pubOwnPhaseAndTime;
    // std::function<bool(const int32_t ownAgentID, const Common::Pose& ownAgentPose)>  m_pubOwnPoseFunc;
    // std::function<bool(Common::Pose& ownAgentPose)>  m_getOwnAgentPose;
    // std::function<bool(std::unordered_map<int32_t, Common::Pose>& agentsPose)>  m_getAgentsPose;

    // std::function<bool(std::unordered_map<int32_t, Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgress)> m_getAgentsPathAndWaypointProgress;
    // std::function<bool(const int32_t, const Common::PathAndWaypointProgress& goTherePathAndWaypointProgress)> m_pubOwnPathAndWaypointProgress;

    // std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath)> m_getAgentsPlannedPath;
    // std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownPlannedPath)> m_pubOwnPlannedPath;

    // std::function<bool(std::unordered_map<int32_t, std::unordered_map<int32_t, Common::PathAndCost>>& agentsProcessedPathOfAgents)> m_getAgentsProcessedPathOfAgents;
    // std::function<bool(const int32_t, const std::unordered_map<int32_t, Common::PathAndCost>& ownProcessedPathOfAgents)> m_pubOwnProcessedPathOfAgents;
    // std::function<void()> m_clearAgentsBestProcessedPathBuffer;
    // std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath)> m_getAgentsBestProcessedPath;
    // std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownBestProcessedPath)> m_pubOwnBestProcessedPath;
    // std::function<bool(const int32_t, const std::vector<Common::Pose>& processedGoTherePath)>  m_pubProcessedGoTherePath;

    // std::function<void()>  m_clearAgentsPoseBuffer;
    // std::function<void()>  m_clearAgentsProcessedPathOfAgentsBuffer;
    // std::function<void()>  m_clearAgentsPlannedPathBuffer;
    // std::function<void()>m_clearAgentsPathAndWaypointProgressBuffer;

bool TeamingPlanner::getPhasesAndTimeRecordOfAgentsCPH(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PhaseAndTime>& phasesAndTimeRecordOfAgents)
bool TeamingPlanner::pubOwnPhaseAndTimeCPH(const int32_t, const DistributedGlobalPathPlanner::Common::PhaseAndTime& ownAgentPhaseAndTime)
bool TeamingPlanner::pubOwnPoseFuncCPH(const int32_t ownAgentID, const DistributedGlobalPathPlanner::Common::Pose& ownAgentPose)

bool TeamingPlanner::getOwnAgentPoseCPH(DistributedGlobalPathPlanner::DistributedGlobalPathPlanner::Common::Pose& ownAgentPose)
bool TeamingPlanner::getAgentPoseCPH(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::Pose>& agentsPose)
bool TeamingPlanner::getAgentsPathAndWaypointProgressCPH(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgress)
bool TeamingPlanner::pubOwnPathAndWaypointProgressCPH(const int32_t, const DistributedGlobalPathPlanner::Common::PathAndWaypointProgress& goTherePathAndWaypointProgress)
bool TeamingPlanner::getAgentsPlannedPathCPH(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath)
bool TeamingPlanner::pubOwnPlannedPathCPH(const int32_t, const std::vector<Eigen::Vector3d>& ownPlannedPath)
bool TeamingPlanner::getAgentsProcessedPathOfAgentsCPH(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>>& agentsProcessedPathOfAgents)
bool TeamingPlanner::pubOwnProcessedPathOfAgentsCPH(const int32_t, const std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>& ownProcessedPathOfAgents)
bool TeamingPlanner::getAgentsBestProcessedPathCPH(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath)
bool TeamingPlanner::pubOwnBestProcessedPathCPH(const int32_t, const std::vector<Eigen::Vector3d>& ownBestProcessedPath)
bool TeamingPlanner::pubProcessedGoTherePathCPH(const int32_t, const std::vector<DistributedGlobalPathPlanner::Common::Pose>& processedGoTherePath)
void TeamingPlanner::clearAgentsPoseBufferCPH()
void TeamingPlanner::clearAgentsProcessedPathOfAgentsBufferCPH()
void TeamingPlanner::clearAgentsPlannedPathBufferCPH()
void TeamingPlanner::clearAgentsPathAndWaypointProgressBufferCPH()

