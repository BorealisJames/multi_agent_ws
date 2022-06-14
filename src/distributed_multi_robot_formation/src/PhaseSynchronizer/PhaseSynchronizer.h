//
// Created by benson on 13/1/21.
//

#pragma once

#include <chrono>
#include <math.h>
#include <sensor_msgs/PointCloud.h>
#include <set>
#include <unordered_map>
#include <vector>

#include "../Common/Common.h"
#include "../ConvexHullOfRobotPosition/ConvexHullOfRobotPosition.h"
#include "../DirectionOfMotion/DirectionOfMotion.h"
#include "../VirtualPositionAssignment/VirtualPositionAssignment.h"
#include "../GenerateConvexRegions/GenerateConvexRegions.h"
#include "../DistributedMultiRobotFormationHandler.h"
#include "../Formation2D/Formation2D.h"
#include "../Formation3D/Formation3D.h"
#include "../ProcessPointCloud/ProcessPointCloud.h"
#include "../FollowMeGoalGenerator/FollowMeGoalGenerator.h"

namespace DistributedFormation
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
    PhaseSynchronizer(const Common::DistributedFormationParameters& params);

    void SetDistributedFormationParameters(const Common::DistributedFormationParameters& params);

    void AttachHandler(const std::shared_ptr<DistributedMultiRobotFormationHandler>& handlerPtr);

    void SyncPhasesOfAgents();

private:
    DistributedMultiRobotFormationHandler::Ptr m_handlerPtr;

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

    void UpdateWorkspaceDimension();

    void UpdateNumberOfAgentsInTeam();
    void UpdatePhasesOfAgentsInTeam();
    void UpdatePositionsOfAgentsInTeam();
    void UpdatePositionOfHuman();

    void ResetPhase();

    bool CheckIfAnyAgentIsStillInPreviousPhase();

    void Vizualize();

    int64_t m_expiryDurationMicroSec;
    unsigned int m_numberOfAzimuthDiscreteAnglesOnASide;
    double m_resolutionAzimuthAngleRad;
    unsigned int m_numberOfElevationDiscreteAnglesOnASide;
    double m_resolutionElevationAngleRad;
    double m_distanceToFollowBehind;

    double m_localBoundingBoxForPathAlongX;
    double m_localBoundingBoxForPathAlongY;
    double m_localBoundingBoxForPathAlongZ;
    double m_pointRemovalRadius;
    double m_desiredDistanceInTriFormation;
    double m_desiredDistanceInLineFormation;
    double m_incrementOffsetToFormationYaw;
    double m_agentRadius;
    double m_waypointReachedBoundary;
    double m_weightForGoal;
    double m_weightForRotation;
    double m_weightForSize;
    double m_desiredHeight;
    double m_priorityPenalty;

    Common::WORKSPACE m_workspace;
    Common::DIMENSION m_dimension;
    Common::PHASE m_phase;
    bool m_transitingPhase;
    Common::Pose m_goalAlongPoses;
    Common::Pose m_goal;

    std::unordered_map<int32_t, Common::PhaseAndTime> m_phasesAndTimeRecordOfAgents;
    std::unordered_map<int32_t, Common::PHASE> m_phasesOfAgentsInTeam;
    std::unordered_map<int32_t, Common::Pose> m_poseOfAgentsInTeam;

    std::unordered_map<int32_t, Common::Pose> m_agentsPose;
    std::unordered_map<int32_t, Common::DirectionUtility> m_agentsAngleIndexUtility;
    std::unordered_map<int32_t, Common::ConvexRegion2D> m_agents2DConvexRegion;
    std::unordered_map<int32_t, Common::ConvexRegion3D> m_agents3DConvexRegion;
    std::unordered_map<int32_t, std::unordered_map<int32_t, Common::Pose>> m_agentsTaskAssignments;

    int32_t m_numberOfAgentsInTeam;
    int32_t m_ownAgentID;
    Common::Pose m_ownAgentPose;
    Common::Pose m_humanPose;
    Common::DirectionUtility m_ownAgentAngleIndexUtility;
    Common::ConvexRegion2D m_ownAgent2DConvexRegion;
    Common::ConvexRegion3D m_ownAgent3DConvexRegion;
    sensor_msgs::PointCloud m_ownPointCloud;
    std::unordered_map<int32_t, Common::Pose> m_ownTaskAssignments;

    std::vector<Common::Pose> m_agentPosesThatFormConvexhull;
    Common::Pose m_avgOfExtremaPose;
    double m_desiredExpansionAzimuthAngleRad;
    double m_desiredExpansionElevationAngleRad;

    bool m_convexRegionOfAgentPoseObtained;
    bool m_consensusDirectionObtained;
    bool m_taskAllocationObtained;
    bool m_taskAssignmentsVerified;
    bool m_allAgentHasReachedLastPhase;

    ConvexHullOfRobotPosition m_convexHullOfRobotPosition;
    DirectionOfMotion m_directionOfMotion;
    GenerateConvexRegions m_generateConvexRegions;
    ProcessPointCloud m_processPointCloud;
    FollowMeGoalGenerator m_followMeGoalGenerator;

    //for viz
    vec_E<Polyhedron<2>> m_polys2DViz;
    vec_E<Polyhedron<3>> m_polys3DViz;
};

}  // namespace DistributedFormation