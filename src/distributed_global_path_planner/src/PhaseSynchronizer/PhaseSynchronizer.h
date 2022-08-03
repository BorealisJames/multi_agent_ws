//
// Created by benson on 13/1/21.
//

#include <chrono>
#include <sensor_msgs/PointCloud.h>
#include <unordered_map>

#include "../Common/Common.h"
#include "../DistributedGlobalPathPlannerHandler.h"
#include "../GoTherePathTracker/GoTherePathTracker.h"
#include "../ProcessPointCloud/ProcessPointCloud.h"
#include "../JumpPointSearch/PathPlanningParams.h"
#include "../JumpPointSearch/PathPlanning2DHandle.h"
#include "../JumpPointSearch/PathPlanning3DHandle.h"

#pragma once

namespace DistributedGlobalPathPlanner
{

/*
 Phase 1: convex hull of robot position
 Phase 2: direction of motion
 Phase 3: intersection of convex regions
 Phase 4: assignment of formation position
 */

class PhaseSynchronizer
{
public:

    PhaseSynchronizer();
    PhaseSynchronizer(const Common::DistributedGlobalPathParams& distributedGlobalPathParams,
                      const pathplanning::PathPlanningParams& pathPlanningParams);

    void SetDistributedGlobalPathPlannerParams(const Common::DistributedGlobalPathParams& distributedGlobalPathParams,
                                               const pathplanning::PathPlanningParams& pathPlanningParams);

    void AttachHandler(const std::shared_ptr<DistributedGlobalPathPlannerHandler>& handlerPtr);

    void SyncPhasesOfAgents();

    void ResetAndClearTeamAgentsPoseAndPhases();

private:
    DistributedGlobalPathPlannerHandler::Ptr m_handlerPtr;

    void OnEnterPhase1();
    void OnEnterPhase2();
    void OnEnterPhase3();
    void OnEnterPhase4();
    void OnEnterPhase5();


    bool TransitingFromPhase1();
    bool TransitingFromPhase2();
    bool TransitingFromPhase3();
    bool TransitingFromPhase4();
    bool TransitingFromPhase5();


    void OnExitPhase1();
    void OnExitPhase2();
    void OnExitPhase3();
    void OnExitPhase4();
    void OnExitPhase5();


    void DoPhase1();
    void DoPhase2();
    void DoPhase3();
    void DoPhase4();
    void DoPhase5();

    void UpdateNumberOfAgentsInTeam();
    void UpdateGoTherePath();
    void UpdatePhasesOfAgentsInTeam();
    void UpdatePositionsOfAgentsInTeam();

    void ResetPhase();

    bool CheckIfAnyAgentIsStillInPreviousPhase();

    void Vizualize();

    bool m_outputObtainedForPhase1;
    bool m_outputObtainedForPhase2;
    bool m_outputObtainedForPhase3;
    bool m_outputObtainedForPhase4;
    bool m_outputObtainedForPhase5;

    Common::DIMENSION m_dimension;
    int64_t m_expiryDurationMicroSec;
    double m_pointRemovalRadius;
    double m_agentRadius;
    double m_waypointReachedBoundary;
    double m_desiredHeight;

    pathplanning::PathPlanningParams m_pathPlanningParams;
    pathplanning::PathPlanning2DHandle m_pathPlanning2DHandle;
    pathplanning::PathPlanning3DHandle m_pathPlanning3DHandle;

    Common::PHASE m_phase;
    bool m_transitingPhase;

    std::unordered_map<int32_t, Common::PhaseAndTime> m_phasesAndTimeRecordOfAgents;
    std::unordered_map<int32_t, Common::PHASE> m_phasesOfAgentsInTeam;
    std::unordered_map<int32_t, Common::Pose> m_poseOfAgentsInTeam;

    std::unordered_map<int32_t, Common::Pose> m_agentsPose;
    std::unordered_map<int32_t, Common::PathAndWaypointProgress> m_agentsPathAndWaypointProgress;
    std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> m_agentsPlannedPath;
    std::unordered_map<int32_t, std::unordered_map<int32_t, Common::PathAndCost>> m_agentsProcessedPathOfAgents;
    std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> m_agentsBestProcessedPath;

    int m_numberOfAgentsInTeam;
    std::vector<Common::Pose> m_goTherePath;
    int32_t m_ownAgentID;
    Common::Pose m_ownAgentPose;
    Common::PathAndWaypointProgress m_ownAgentPathAndWaypointProgress;
    sensor_msgs::PointCloud m_ownPointCloud;
    std::vector<Eigen::Vector3d> m_ownPlannedPath;
    std::unordered_map<int32_t, Common::PathAndCost> m_ownProcessedPathOfAgents;
    std::vector<Eigen::Vector3d> m_ownBestProcessedPath;

    Common::Pose m_avgOfExtremaPose;

    GoTherePathTracker m_goTherePathTracker;
    ProcessPointCloud m_processPointCloud;
};

}  // namespace DistributedGlobalPathPlanner