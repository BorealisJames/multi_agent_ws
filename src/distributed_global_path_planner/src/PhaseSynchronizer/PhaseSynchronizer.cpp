//
// Created by benson on 13/1/21.
//

#include "PhaseSynchronizer.h"

namespace DistributedGlobalPathPlanner
{
    PhaseSynchronizer::PhaseSynchronizer()
    : m_phase(Common::PHASE::PHASE_1)
    , m_transitingPhase(true)
    , m_handlerPtr(std::make_shared<DistributedGlobalPathPlannerHandler>())

    , m_dimension(Common::DIMENSION::DIM_2)
    , m_expiryDurationMicroSec(15*1000000)
    , m_pointRemovalRadius(0.45)
    , m_agentRadius(0.3)
    , m_waypointReachedBoundary(1.5)
    , m_desiredHeight(2.0)
    , m_pathPlanningParams()
    {
        //not the best way but for prototyping so will set the values here
        m_pathPlanningParams.mapMinBoundsX = -100.0;
        m_pathPlanningParams.mapMinBoundsY = -100.0;
        m_pathPlanningParams.mapMinBoundsZ = 0.0;
        m_pathPlanningParams.mapMaxBoundsX = 100.0;
        m_pathPlanningParams.mapMaxBoundsY = 100.0;
        m_pathPlanningParams.mapMaxBoundsZ = 3.0;
        m_pathPlanningParams.mapResolution = 0.25;
        m_pathPlanningParams.radius = m_agentRadius;
        m_pathPlanningParams.cylinderHeight = m_agentRadius;
        m_pathPlanningParams.hCostWeight = 1.0;
        m_pathPlanningParams.potentialRadius = 3.0;
        m_pathPlanningParams.searchRadius = 1.5;
        m_pathPlanningParams.performJPS = true;

        if (m_dimension == Common::DIMENSION::DIM_2)
        {
            m_pathPlanning2DHandle.setParams(m_pathPlanningParams);
        }
        else
        {
            m_pathPlanning3DHandle.setParams(m_pathPlanningParams);
        }
    }

    PhaseSynchronizer::PhaseSynchronizer(const Common::DistributedGlobalPathParams& distributedGlobalPathParams,
                                         const pathplanning::PathPlanningParams& pathPlanningParams)
    : m_phase(Common::PHASE::PHASE_1)
    , m_transitingPhase(true)
    , m_handlerPtr(std::make_shared<DistributedGlobalPathPlannerHandler>())

    , m_dimension(distributedGlobalPathParams.dimension)
    , m_expiryDurationMicroSec(distributedGlobalPathParams.expiryDurationMicroSec)
    , m_pointRemovalRadius(distributedGlobalPathParams.pointRemovalRadius)
    , m_agentRadius(distributedGlobalPathParams.agentRadius)
    , m_waypointReachedBoundary(distributedGlobalPathParams.waypointReachedBoundary)
    , m_desiredHeight(distributedGlobalPathParams.desiredHeight)
    , m_pathPlanningParams(pathPlanningParams)
    {
        if (m_dimension == Common::DIMENSION::DIM_2)
        {
            m_pathPlanning2DHandle.setParams(m_pathPlanningParams);
        }
        else
        {
            m_pathPlanning3DHandle.setParams(m_pathPlanningParams);
        }
    }

    void PhaseSynchronizer::SetDistributedGlobalPathPlannerParams(const Common::DistributedGlobalPathParams& distributedGlobalPathParams,
                                                                  const pathplanning::PathPlanningParams& pathPlanningParams)
    {
        m_dimension = distributedGlobalPathParams.dimension;
        m_expiryDurationMicroSec = distributedGlobalPathParams.expiryDurationMicroSec;
        m_pointRemovalRadius = distributedGlobalPathParams.pointRemovalRadius;
        m_agentRadius = distributedGlobalPathParams.agentRadius;
        m_waypointReachedBoundary = distributedGlobalPathParams.waypointReachedBoundary;
        m_desiredHeight = distributedGlobalPathParams.desiredHeight;
        m_pathPlanningParams = pathPlanningParams;

        if (m_dimension == Common::DIMENSION::DIM_2)
        {
            m_pathPlanning2DHandle.setParams(m_pathPlanningParams);
        }
        else
        {
            m_pathPlanning3DHandle.setParams(m_pathPlanningParams);
        }
    }

    void PhaseSynchronizer::AttachHandler(const std::shared_ptr<DistributedGlobalPathPlannerHandler>& handlerPtr)
    {
        m_handlerPtr = handlerPtr;

        m_handlerPtr->m_getGoTherePath(m_goTherePath);
        m_handlerPtr->m_getOwnAgentID(m_ownAgentID);

        m_goTherePathTracker.InitPathToTrack(m_goTherePath);
    }

    void PhaseSynchronizer::SyncPhasesOfAgents()
    {
        //Viz
        Vizualize();

        // check if reset is needed
        UpdateNumberOfAgentsInTeam();
        UpdateGoTherePath();
        UpdatePhasesOfAgentsInTeam();
        UpdatePositionsOfAgentsInTeam();

        Common::PhaseAndTime ownPhaseAndTime;
        ownPhaseAndTime.phase = m_phase;
        auto timeNow = std::chrono::system_clock::now();
        auto timeNowMicroSecs = std::chrono::duration_cast<std::chrono::microseconds>(timeNow.time_since_epoch()).count();
        ownPhaseAndTime.timeMicroSecs = timeNowMicroSecs;
        m_handlerPtr->m_pubOwnPhaseAndTime(m_ownAgentID, ownPhaseAndTime);

        switch(m_phase)
        {
            case Common::PHASE::PHASE_1:
            {
                std::cout << "Agent_cp " << m_ownAgentID << ": Phase1" << std::endl;

                if (m_transitingPhase)          // transiting in
                {
                    OnEnterPhase1();
                    m_transitingPhase = false;
                }
                if (TransitingFromPhase1())     //transiting out to next phase
                {
                    m_transitingPhase = true;
                    OnExitPhase1();
                }
                else                            //remaining in
                {
                    DoPhase1();
                }
                break;
            }
            case Common::PHASE::PHASE_2:
            {
                std::cout << "Agent_cp " << m_ownAgentID << ": Phase2" << std::endl;

                if (m_transitingPhase)          // transiting in
                {
                    OnEnterPhase2();
                    m_transitingPhase = false;
                }
                if (TransitingFromPhase2())     //transiting out to next phase
                {
                    m_transitingPhase = true;
                    OnExitPhase2();
                }
                else                            //remaining in
                {
                    DoPhase2();
                }
                break;
            }
            case Common::PHASE::PHASE_3:
            {
                std::cout << "Agent_cp " << m_ownAgentID << ": Phase3" << std::endl;

                if (m_transitingPhase)          // transiting in
                {
                    OnEnterPhase3();
                    m_transitingPhase = false;
                }
                if (TransitingFromPhase3())     //transiting out to next phase
                {
                    m_transitingPhase = true;
                    OnExitPhase3();
                }
                else                            //remaining in
                {
                    DoPhase3();
                }
                break;
            }
            case Common::PHASE::PHASE_4:
            {
                std::cout << "Agent_cp " << m_ownAgentID << ": Phase4" << std::endl;

                if (m_transitingPhase)          // transiting in
                {
                    OnEnterPhase4();
                    m_transitingPhase = false;
                }
                if (TransitingFromPhase4())     //transiting out to next phase
                {
                    m_transitingPhase = true;
                    OnExitPhase4();
                }
                else                            //remaining in
                {
                    DoPhase4();
                }
                break;
            }
            case Common::PHASE::PHASE_5:
            {
                std::cout << "Agent_cp " << m_ownAgentID << ": Phase5" << std::endl;

                if (m_transitingPhase)          // transiting in
                {
                    OnEnterPhase5();
                    m_transitingPhase = false;
                }
                if (TransitingFromPhase5())     //transiting out to next phase
                {
                    m_transitingPhase = true;
                    OnExitPhase4();
                }
                else                            //remaining in
                {
                    DoPhase5();
                }
                break;
            }

            default:
            {
                std::cout << "Agent_cp " << m_ownAgentID << ": switch statement phase do not exist" << std::endl;
                ResetPhase();
            }
        }
    }

    void PhaseSynchronizer::UpdateNumberOfAgentsInTeam()
    {
        m_handlerPtr->m_getNumberOfAgentsInTeam(m_numberOfAgentsInTeam);
    }

    void PhaseSynchronizer::UpdateGoTherePath()
    {
        std::vector<Common::Pose> goTherePath;
        m_handlerPtr->m_getGoTherePath(goTherePath);

        if (goTherePath.size() < 1)
        {
            std::cout << "Agent_cp " << m_ownAgentID << ": reset as go there path has less than 1 points" << std::endl;

            ResetPhase();
            return;
        }

        bool resetPath = false;

        if (goTherePath.size() != m_goTherePath.size())
        {
            resetPath = true;
        }
        else
        {
            for (int i = 0; i < goTherePath.size(); i++)
            {
                if (std::abs(goTherePath.at(i).position.norm() - m_goTherePath.at(i).position.norm()) > FLT_EPSILON ||
                    std::abs(goTherePath.at(i).headingRad - m_goTherePath.at(i).headingRad) > FLT_EPSILON)
                {
                    resetPath = true;
                    break;
                }
            }
        }

        if (resetPath)
        {
            m_goTherePath = goTherePath;
            m_goTherePathTracker.InitPathToTrack(m_goTherePath);

            std::cout << "Agent_cp " << m_ownAgentID << ": reset as path has changed" << std::endl;

            ResetPhase();
            return;
        }
    }

    void PhaseSynchronizer::UpdatePhasesOfAgentsInTeam()
    {
        m_handlerPtr->m_getPhasesAndTimeRecordOfAgents(m_phasesAndTimeRecordOfAgents);

        std::cout << "Agent_cp " << m_ownAgentID << ": Getting m_phasesAndTimeRecordOfAgents" << " The size is " << m_phasesAndTimeRecordOfAgents.size() << std::endl;

        auto timeNow = std::chrono::system_clock::now();
        auto timeNowMicroSecs = std::chrono::duration_cast<std::chrono::microseconds>(timeNow.time_since_epoch()).count();
        
        // update own phase and time to record in case it was not added
        Common::PhaseAndTime ownPhaseAndTime;
        ownPhaseAndTime.phase = m_phase;
        ownPhaseAndTime.timeMicroSecs = timeNowMicroSecs;
        m_phasesAndTimeRecordOfAgents[m_ownAgentID] = ownPhaseAndTime;

        //reset if team members are not equal to number of expecting agents
        if (m_phasesAndTimeRecordOfAgents.size() != m_numberOfAgentsInTeam)
        {
            std::cout << "Agent_cp " << m_ownAgentID << ": reset as expected number of agents do not match" << std::endl;
            std::cout << "Agent_cp " << m_ownAgentID << ": m_phasesAndTimeRecordOfAgents.size: " << m_phasesAndTimeRecordOfAgents.size() << ", m_numberOfAgentsInTeam: " <<  m_numberOfAgentsInTeam << std::endl;
            ResetPhase();
            return;
        }

        std::unordered_map<int32_t, Common::PHASE> latestPhasesOfAgentsInTeam;
        int32_t largestPhase = INT32_MIN;
        int32_t smallestPhase= INT32_MAX;
        std::set<int32_t> latestAgentsInTeam;
        std::set<int32_t> lastKnownAgentsInTeam;
        for (auto&&phaseAndTimeOfAgent :  m_phasesAndTimeRecordOfAgents)
        {
            if (std::abs(timeNowMicroSecs - phaseAndTimeOfAgent.second.timeMicroSecs) < m_expiryDurationMicroSec)
            {
                latestPhasesOfAgentsInTeam[phaseAndTimeOfAgent.first] = phaseAndTimeOfAgent.second.phase;
                latestAgentsInTeam.insert(phaseAndTimeOfAgent.first);

                if (static_cast<int32_t>(phaseAndTimeOfAgent.second.phase) < smallestPhase)
                {
                    smallestPhase = static_cast<int32_t>(phaseAndTimeOfAgent.second.phase);
                }
                if (static_cast<int32_t>(phaseAndTimeOfAgent.second.phase) > largestPhase)
                {
                    largestPhase = static_cast<int32_t>(phaseAndTimeOfAgent.second.phase);
                }
            }
        }

        for (auto&& agent: m_phasesOfAgentsInTeam)
        {
            lastKnownAgentsInTeam.insert(agent.first);
        }

        m_phasesOfAgentsInTeam = latestPhasesOfAgentsInTeam;

        //reset if team member changes or phases more than 2 steps apart
        if (m_phasesOfAgentsInTeam.empty() ||
            latestAgentsInTeam != lastKnownAgentsInTeam ||
            std::abs(largestPhase - smallestPhase)%(5-1)>=2)
        {
            std::cout << "Agent_cp " << m_ownAgentID << ": reset as phases are not in sync" << std::endl;

            ResetPhase();
            return;
        }
    }

    void PhaseSynchronizer::UpdatePositionsOfAgentsInTeam()
    {
        m_handlerPtr->m_getOwnAgentPose(m_ownAgentPose);
        //publish phase 1 message, agent's position (will send out id and position)
        m_handlerPtr->m_pubOwnPoseFunc(m_ownAgentID, m_ownAgentPose);

        //get other agent's position
        m_handlerPtr->m_getAgentsPose(m_agentsPose);
        std::cout << "Agent get" << m_ownAgentID << " agent pose map size " << m_agentsPose.size() << std::endl;
        for (auto agentPose : m_agentsPose)
        {
            std::cout << "Agent_cp " << m_ownAgentID << " agent pose map contains " << agentPose.first << std::endl;
        }

        //set own position in team
        m_agentsPose[m_ownAgentID] = m_ownAgentPose;

        for(auto&&agent : m_phasesOfAgentsInTeam)
        {
            int32_t agentID = agent.first;
            if (m_agentsPose.find(agentID) != m_agentsPose.end())
            {
                m_poseOfAgentsInTeam[agentID] = m_agentsPose[agentID];
            }
        }
    }

    void PhaseSynchronizer::ResetPhase()
    {
        std::cout << "Agent_cp " << m_ownAgentID << ": ResetPhase" << std::endl;

        m_phase = Common::PHASE::PHASE_1;
        m_transitingPhase = true;

        m_poseOfAgentsInTeam.clear();
        m_phasesAndTimeRecordOfAgents.clear();

        m_agentsPose.clear();
        m_agentsPlannedPath.clear();
        m_agentsProcessedPathOfAgents.clear();
        m_agentsBestProcessedPath.clear();

        m_outputObtainedForPhase1 = false;
        m_outputObtainedForPhase2 = false;
        m_outputObtainedForPhase3 = false;
        m_outputObtainedForPhase4 = false;
        m_outputObtainedForPhase5 = false;

    }

    bool PhaseSynchronizer::CheckIfAnyAgentIsStillInPreviousPhase()
    {
        bool ret_val = false;
        for (auto&&agent : m_phasesOfAgentsInTeam)
        {
            if (static_cast<int>(agent.second) == static_cast<int>(m_phase) - 1 &&
                m_phase != Common::PHASE::PHASE_1)
            {
                ret_val = true;
                break;
            }
        }

        return ret_val;
    }

    /*for phase 1*/
    void PhaseSynchronizer::OnEnterPhase1()
    {
        std::cout << "Agent_cp " << m_ownAgentID << ": reset in OnEnterPhase1" << std::endl;

        ResetPhase();
        m_handlerPtr->m_clearAgentsPoseBuffer();
        m_handlerPtr->m_clearPhasesAndTimeRecordOfAgentsBuffer();
    }

    bool PhaseSynchronizer::TransitingFromPhase1()
    {
        if(m_outputObtainedForPhase1)
        {
            m_phase = Common::PHASE::PHASE_2;
            return true;
        }
        else
        {
            return false;
        }
    }

    void PhaseSynchronizer::OnExitPhase1()
    {
        m_outputObtainedForPhase1 = false;
    }

    void PhaseSynchronizer::DoPhase1()
    {
        if (m_poseOfAgentsInTeam.size() == m_phasesOfAgentsInTeam.size() && m_phasesOfAgentsInTeam.size()>0)
        {
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            //get points that form the convex region
            //input: m_positionOfAgentsInTeam
            //output: A* path plan

            double maxX = -DBL_MAX;
            double minX = DBL_MAX;
            double maxY = -DBL_MAX;
            double minY = DBL_MAX;
            double maxZ = -DBL_MAX;
            double minZ = DBL_MAX;
            double sumOfSin = 0.0;
            double sumOfCos = 0.0;
            m_avgOfExtremaPose.position(0) = 0;
            m_avgOfExtremaPose.position(1) = 0;
            m_avgOfExtremaPose.position(2) = 0;
            m_avgOfExtremaPose.headingRad = 0;

            int numberOfAgents = m_poseOfAgentsInTeam.size();
            if (numberOfAgents <= 0)
            {
                std::cout << "Agent_cp " << m_ownAgentID << ": reset cause unable to get at least 1 agent to form avg of extrema" << std::endl;

                ResetPhase();
                return;
            }

            for (auto &&agentPose : m_poseOfAgentsInTeam)
            {
                maxX = std::max(maxX, agentPose.second.position.x());
                minX = std::min(minX, agentPose.second.position.x());
                maxY = std::max(maxY, agentPose.second.position.y());
                minY = std::min(minY, agentPose.second.position.y());
                maxZ = std::max(maxZ, agentPose.second.position.z());
                minZ = std::min(minZ, agentPose.second.position.z());

                sumOfSin += sin(agentPose.second.headingRad);
                sumOfCos += cos(agentPose.second.headingRad);
            }
            m_avgOfExtremaPose.headingRad = atan2(sumOfSin, sumOfCos);
            m_avgOfExtremaPose.position(0) = (maxX + minX) / 2;
            m_avgOfExtremaPose.position(1) = (maxY + minY) / 2;
            m_avgOfExtremaPose.position(2) = (maxZ + minZ) / 2;


            m_outputObtainedForPhase1 = true;
        }
    }

    /*for phase 2*/
    void PhaseSynchronizer::OnEnterPhase2()
    {
        m_handlerPtr->m_clearAgentsPathAndWaypointProgressBuffer();

        m_ownAgentPathAndWaypointProgress.poses = m_goTherePathTracker.getPathToTrack();
        m_ownAgentPathAndWaypointProgress.waypointProgress = m_goTherePathTracker.getWPIndex();
    }

    bool PhaseSynchronizer::TransitingFromPhase2()
    {
        if(m_outputObtainedForPhase2)
        {
            m_phase = Common::PHASE::PHASE_3;
            return true;
        }
        else
        {
            return false;
        }
    }

    void PhaseSynchronizer::OnExitPhase2()
    {
        m_outputObtainedForPhase2 = false;
    }

    void PhaseSynchronizer::DoPhase2()
    {
        if (CheckIfAnyAgentIsStillInPreviousPhase())
        {
            //publish phase 1 message, agent's position (will send out id and position)
            m_handlerPtr->m_pubOwnPoseFunc(m_ownAgentID, m_ownAgentPose);
        }
        else
        {
            //pub phase 2 message, agent's own planned path
            m_handlerPtr->m_pubOwnPathAndWaypointProgress(m_ownAgentID, m_ownAgentPathAndWaypointProgress);

            //get other agent's planned path
            m_handlerPtr->m_getAgentsPathAndWaypointProgress(m_agentsPathAndWaypointProgress);
            m_agentsPathAndWaypointProgress[m_ownAgentID] = m_ownAgentPathAndWaypointProgress;

            std::unordered_map<int32_t, Common::PathAndWaypointProgress> agentsGoTherePathAndWaypointProgressCopy;
            for(auto&&agent : m_phasesOfAgentsInTeam)
            {
                int32_t agentID = agent.first;
                if (m_agentsPathAndWaypointProgress.find(agentID) != m_agentsPathAndWaypointProgress.end())
                {
                    agentsGoTherePathAndWaypointProgressCopy[agentID] = m_agentsPathAndWaypointProgress[agentID];
                }
            }

            if (agentsGoTherePathAndWaypointProgressCopy.size() == m_phasesOfAgentsInTeam.size())
            {
                //check that every agent has the same path
                bool pathIsSimilar = true;
                for (auto&& agentPath1 : agentsGoTherePathAndWaypointProgressCopy)
                {
                    if (!pathIsSimilar)
                    {
                        break;
                    }

                    for (auto&& agentPath2 : agentsGoTherePathAndWaypointProgressCopy)
                    {

                        if (agentPath1.first >= agentPath2.first)
                        {
                            continue;
                        }

                        if (agentPath1.second.poses.size() != agentPath2.second.poses.size())
                        {
                            pathIsSimilar = false;
                            break;
                        }

                        for (int i=0; i<agentPath1.second.poses.size(); i++)
                        {
                            if (std::abs(agentPath1.second.poses.at(i).position.norm() - agentPath2.second.poses.at(i).position.norm()) > FLT_EPSILON ||
                                std::abs(agentPath1.second.poses.at(i).headingRad - agentPath2.second.poses.at(i).headingRad) > FLT_EPSILON)
                            {
                                pathIsSimilar = false;
                                break;
                            }
                        }
                    }
                }

                if (!pathIsSimilar)
                {
                    std::cout << "Agent_cp " << m_ownAgentID << ": best path from agents are not in consensus" << std::endl;

                    ResetPhase();
                    return;
                }

                int largestWPIndex = 0;
                for (auto&& agentPath : agentsGoTherePathAndWaypointProgressCopy)
                {
                    largestWPIndex = std::max(largestWPIndex, agentPath.second.waypointProgress);
                }
                m_goTherePathTracker.setWPIndex(largestWPIndex);

                m_outputObtainedForPhase2 = true;
            }
        }
    }

    /*for phase 3*/
    void PhaseSynchronizer::OnEnterPhase3()
    {
        m_handlerPtr->m_clearAgentsPlannedPathBuffer();

        //get path to track
        std::vector<Common::Pose> updatedPath;
        m_goTherePathTracker.GetUpdatedPath(m_avgOfExtremaPose, updatedPath);
        if (updatedPath.empty())
        {
            std::cout << "Agent_cp " << m_ownAgentID << ": global path is empty" << std::endl;

            ResetPhase();
            return;
        }
        else if (updatedPath.size() == 1)
        {
            updatedPath.insert(updatedPath.begin(), m_avgOfExtremaPose);
        }


        //get point cloud and process it
        m_ownPointCloud.points.clear();
        m_ownPointCloud.channels.clear();
        if (!m_handlerPtr->m_getOwnAgentLidarPointCloud(m_ownPointCloud))
        {
            std::cout << "Agent_cp " << m_ownAgentID << ": unable to get both lidar point cloud" << std::endl;

            ResetPhase();
            return;
        }

//        //wall hack
//        m_ownPointCloud.points.clear();
//        m_ownPointCloud.channels.clear();
//        geometry_msgs::Point32 pt;
//        if (m_ownAgentID == 0 || m_ownAgentID == 1)
//        {
//            for (double x = -1.0; x<=1.0; x=x+0.5)
//            {
//                for (double y = -3.5; y<=3.0; y=y+0.5)
//                {
//                    for (double z = 0.0; z<=4.0; z=z+0.5)
//                    {
//                        pt.x = x; pt.y = y; pt.z = z;
//                        m_ownPointCloud.points.push_back(pt);
//                    }
//                }
//            }
//        }
//        if (m_ownAgentID == 0)
//        {
//            //corridor hack
//            for (double x = 8.0; x<=10.0; x=x+0.5)
//            {
//                for (double y = -3.5; y<=6.5; y=y+0.5)
//                {
//                    for (double z = 0.0; z<=4.0; z=z+0.5)
//                    {
//                        pt.x = x; pt.y = y; pt.z = z;
//                        m_ownPointCloud.points.push_back(pt);
//                    }
//                }
//            }
//            for (double x = 8.0; x<=10.0; x=x+0.5)
//            {
//                for (double y = -9.5; y<=-5.5; y=y+0.5)
//                {
//                    for (double z = 0.0; z<=4.0; z=z+0.5)
//                    {
//                        pt.x = x; pt.y = y; pt.z = z;
//                        m_ownPointCloud.points.push_back(pt);
//                    }
//                }
//            }
//        }

        if (m_dimension == Common::DIMENSION::DIM_2)
        {
            sensor_msgs::PointCloud tmpOutputPointCloud;
            for(auto&& agentPose : m_poseOfAgentsInTeam)
            {
                m_processPointCloud.RemovePointsWithinARadiusAndFromGroundFromPointCloud2D(m_ownPointCloud,
                                                                                           agentPose.second.position,
                                                                                           m_pointRemovalRadius,
                                                                                           tmpOutputPointCloud);
                m_ownPointCloud = tmpOutputPointCloud;
            }

            // reset map with new point cloud
            m_pathPlanning2DHandle.resetMap();
            m_pathPlanning2DHandle.addInflatedObstaclesPoints(m_ownPointCloud);

            // plan path with new map and updatedPath
            std::vector<Eigen::Vector2d> inputPathWaypoints;
            for (auto pose : updatedPath)
            {
                inputPathWaypoints.push_back(Eigen::Vector2d(pose.position.x(), pose.position.y()));
            }
            std::vector<Eigen::Vector2d> outputPlannedPathWaypoints;
            m_pathPlanning2DHandle.plan(inputPathWaypoints, outputPlannedPathWaypoints);

            m_ownPlannedPath.clear();
            for (auto&& waypoint : outputPlannedPathWaypoints)
            {
                m_ownPlannedPath.push_back(Eigen::Vector3d(waypoint.x(), waypoint.y(), m_desiredHeight));

            }
        }
        else
        {
            sensor_msgs::PointCloud tmpOutputPointCloud;
            for(auto&& agentPose : m_poseOfAgentsInTeam)
            {
                m_processPointCloud.RemovePointsWithinARadiusPointCloud3D(m_ownPointCloud,
                                                                          agentPose.second.position,
                                                                          m_pointRemovalRadius,
                                                                          tmpOutputPointCloud);
                m_ownPointCloud = tmpOutputPointCloud;
            }

            // reset map with new point cloud
            m_pathPlanning3DHandle.resetMap();
            m_pathPlanning3DHandle.addInflatedObstaclesPoints(m_ownPointCloud);

            // plan path with new map and updatedPath
            std::vector<Eigen::Vector3d> inputPathWaypoints;
            for (auto pose : updatedPath)
            {
                inputPathWaypoints.push_back(pose.position);
            }
            std::vector<Eigen::Vector3d> outputPlannedPathWaypoints;
            m_pathPlanning3DHandle.plan(inputPathWaypoints, outputPlannedPathWaypoints);

            m_ownPlannedPath.clear();
            m_ownPlannedPath = outputPlannedPathWaypoints;
        }
    }

    bool PhaseSynchronizer::TransitingFromPhase3()
    {
        if(m_outputObtainedForPhase3)
        {
            m_phase = Common::PHASE::PHASE_4;
            return true;
        }
        else
        {
            return false;
        }
    }

    void PhaseSynchronizer::OnExitPhase3()
    {
        m_outputObtainedForPhase3 = false;
    }

    void PhaseSynchronizer::DoPhase3()
    {
        if (CheckIfAnyAgentIsStillInPreviousPhase())
        {
            //publish phase 1 message, agent's position (will send out id and position)
            m_handlerPtr->m_pubOwnPathAndWaypointProgress(m_ownAgentID, m_ownAgentPathAndWaypointProgress);
        }
        else
        {
            //pub phase 2 message, agent's own planned path
            m_handlerPtr->m_pubOwnPlannedPath(m_ownAgentID, m_ownPlannedPath);

            //get other agent's planned path
            m_handlerPtr->m_getAgentsPlannedPath(m_agentsPlannedPath);
            m_agentsPlannedPath[m_ownAgentID] = m_ownPlannedPath;

            std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> agentsPlannedPathCopy;
            for(auto&&agent : m_phasesOfAgentsInTeam)
            {
                int32_t agentID = agent.first;
                if (m_agentsPlannedPath.find(agentID) != m_agentsPlannedPath.end())
                {
                    agentsPlannedPathCopy[agentID] = m_agentsPlannedPath[agentID];
                }
            }

            if (agentsPlannedPathCopy.size() == m_phasesOfAgentsInTeam.size())
            {
                m_ownProcessedPathOfAgents.clear();

                //perform check on every path here to shorten it if needed here
                std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> m_agentsProcessedPlannedPath;
                for (auto&& agentPlannedPath : agentsPlannedPathCopy)
                {
                    std::vector<Eigen::Vector3d> agentProcessedPlannedPath;
                    if (!agentPlannedPath.second.empty())
                    {
                        agentProcessedPlannedPath.push_back(agentPlannedPath.second.at(0));
                    }

                    bool collisionDetected = false;
                    for (int i = 1; i < agentPlannedPath.second.size() && !collisionDetected; i++)
                    {
                        Eigen::Vector3d point3DBeforeCollision;

                        if (m_dimension == Common::DIMENSION::DIM_2)
                        {
                            Eigen::Vector2d start (agentPlannedPath.second.at(i-1).x(), agentPlannedPath.second.at(i-1).y());
                            Eigen::Vector2d end (agentPlannedPath.second.at(i).x(), agentPlannedPath.second.at(i).y());
                            Eigen::Vector2d point2DBeforeCollision;
                            collisionDetected = m_pathPlanning2DHandle.collisionFoundBetween2Points(start, end, point2DBeforeCollision);
                            point3DBeforeCollision(0) = point2DBeforeCollision.x();
                            point3DBeforeCollision(1) = point2DBeforeCollision.y();
                            point3DBeforeCollision(2) = m_desiredHeight;
                        }
                        else
                        {
                            Eigen::Vector3d start = agentPlannedPath.second.at(i-1);
                            Eigen::Vector3d end = agentPlannedPath.second.at(i);
                            collisionDetected = m_pathPlanning3DHandle.collisionFoundBetween2Points(start, end, point3DBeforeCollision);
                        }

                        if (collisionDetected)
                        {
                            agentProcessedPlannedPath.push_back(point3DBeforeCollision);
                        }
                        else
                        {
                            agentProcessedPlannedPath.push_back(agentPlannedPath.second.at(i));
                        }
                    }

                    double progress = 0.0;
                    if (!agentProcessedPlannedPath.empty())
                    {
                        m_goTherePathTracker.GetProgressOfPointALongPathToTrack(agentProcessedPlannedPath.back(), progress);
                    }

                    Common::PathAndCost pathAndCost;
                    pathAndCost.positions = agentProcessedPlannedPath;
                    pathAndCost.cost = 1.0 / (progress+1.0);
                    m_ownProcessedPathOfAgents[agentPlannedPath.first] = pathAndCost;
                }

                m_outputObtainedForPhase3 = true;
            }
        }
    }

    /*for phase 4*/
    void PhaseSynchronizer::OnEnterPhase4()
    {
        m_handlerPtr->m_clearAgentsProcessedPathOfAgentsBuffer();
    }

    bool PhaseSynchronizer::TransitingFromPhase4()
    {
        if(m_outputObtainedForPhase4)
        {
            m_phase = Common::PHASE::PHASE_5;
            return true;
        }
        else
        {
            return false;
        }
    }

    void PhaseSynchronizer::OnExitPhase4()
    {
        m_outputObtainedForPhase4 = false;
    }

    void PhaseSynchronizer::DoPhase4()
    {
        if (CheckIfAnyAgentIsStillInPreviousPhase())
        {
            //publish phase 1 message, agent's planned path (will send out id and planned path)
            m_handlerPtr->m_pubOwnPlannedPath(m_ownAgentID, m_ownPlannedPath);
        }
        else
        {
            //pub phase 2 message (will send out id and something)
            m_handlerPtr->m_pubOwnProcessedPathOfAgents(m_ownAgentID, m_ownProcessedPathOfAgents);

            //get other agent's processed path of agents
            m_handlerPtr->m_getAgentsProcessedPathOfAgents(m_agentsProcessedPathOfAgents);
            m_agentsProcessedPathOfAgents[m_ownAgentID] = m_ownProcessedPathOfAgents;

            std::unordered_map<int32_t, std::unordered_map<int32_t, Common::PathAndCost>> agentsProcessedPathOfAgentsCopy;
            for(auto&&agent : m_phasesOfAgentsInTeam)
            {
                int32_t agentID = agent.first;
                if (m_agentsProcessedPathOfAgents.find(agentID) != m_agentsProcessedPathOfAgents.end())
                {
                    agentsProcessedPathOfAgentsCopy[agentID] = m_agentsProcessedPathOfAgents[agentID];
                }
            }

            if (agentsProcessedPathOfAgentsCopy.size() == m_phasesOfAgentsInTeam.size())
            {
                for (auto&& processedPathOfAgents : agentsProcessedPathOfAgentsCopy)
                {
                    if (processedPathOfAgents.first == m_ownAgentID)
                    {
                        continue;
                    }

                    if (processedPathOfAgents.second.size() == m_ownProcessedPathOfAgents.size())
                    {
                        for (auto&& processedPath : processedPathOfAgents.second)
                        {
                            if (m_ownProcessedPathOfAgents.find(processedPath.first) != m_ownProcessedPathOfAgents.end())
                            {
                                if (m_ownProcessedPathOfAgents[processedPath.first].cost < processedPath.second.cost)
                                {
                                    m_ownProcessedPathOfAgents[processedPath.first] = processedPath.second;
                                }
                            }
                            else
                            {
                                std::cout << "Agent_cp " << m_ownAgentID << ": unable to match agents id" << std::endl;

                                ResetPhase();
                                return;
                            }
                        }
                    }
                    else
                    {
                        std::cout << "Agent_cp " << m_ownAgentID << ": unable to match agents id" << std::endl;

                        ResetPhase();
                        return;
                    }
                }

                m_ownBestProcessedPath.clear();
                double lowestCost = DBL_MAX;
                int32_t lowestCostID = -1;
                for (auto&& processedPathOfAgent : m_ownProcessedPathOfAgents)
                {
                    if ((processedPathOfAgent.second.cost < lowestCost) ||
                        ((processedPathOfAgent.second.cost == lowestCost) && (processedPathOfAgent.first > lowestCostID)))
                    {
                        lowestCostID = processedPathOfAgent.first;
                        lowestCost = processedPathOfAgent.second.cost;
                        m_ownBestProcessedPath = processedPathOfAgent.second.positions;
                    }
                }

                m_outputObtainedForPhase4 = true;
            }
        }
    }

    /*for phase 5*/
    void PhaseSynchronizer::OnEnterPhase5()
    {
        m_handlerPtr->m_clearAgentsBestProcessedPathBuffer();
    }

    bool PhaseSynchronizer::TransitingFromPhase5()
    {
        if(m_outputObtainedForPhase5)
        {
            m_phase = Common::PHASE::PHASE_1;
            return true;
        }
        else
        {
            return false;
        }
    }

    void PhaseSynchronizer::OnExitPhase5()
    {
        m_outputObtainedForPhase5 = false;
    }

    void PhaseSynchronizer::DoPhase5()
    {
        if (CheckIfAnyAgentIsStillInPreviousPhase())
        {
            //publish message from previous phase
            m_handlerPtr->m_pubOwnProcessedPathOfAgents(m_ownAgentID, m_ownProcessedPathOfAgents);
        }
        else
        {
            //pub current phase message calculated in pevious phase (will send out id and something)
            m_handlerPtr->m_pubOwnBestProcessedPath(m_ownAgentID, m_ownBestProcessedPath);

            //get other agent's processed path of agents
            m_handlerPtr->m_getAgentsBestProcessedPath(m_agentsBestProcessedPath);
            m_agentsBestProcessedPath[m_ownAgentID] = m_ownBestProcessedPath;

            std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> agentsBestProcessedPathCopy;
            for(auto&&agent : m_phasesOfAgentsInTeam)
            {
                int32_t agentID = agent.first;
                if (m_agentsBestProcessedPath.find(agentID) != m_agentsBestProcessedPath.end())
                {
                    agentsBestProcessedPathCopy[agentID] = m_agentsBestProcessedPath[agentID];
                }
            }

            if (agentsBestProcessedPathCopy.size() == m_phasesOfAgentsInTeam.size())
            {
                //check that every agent has the same path
                bool pathIsSimilar = true;
                for (auto&& agentPath1 : agentsBestProcessedPathCopy)
                {
                    if (!pathIsSimilar)
                    {
                        break;
                    }

                    for (auto&& agentPath2 : agentsBestProcessedPathCopy)
                    {

                        if (agentPath1.first >= agentPath2.first)
                        {
                            continue;
                        }

                        if (agentPath1.second.size() != agentPath2.second.size())
                        {
                            pathIsSimilar = false;
                            break;
                        }

                        for (int i=0; i<agentPath1.second.size(); i++)
                        {
                            if (std::abs(agentPath1.second.at(i).norm() - agentPath2.second.at(i).norm()) > FLT_EPSILON)
                            {
                                pathIsSimilar = false;
                                break;
                            }
                        }
                    }
                }

                if (!pathIsSimilar)
                {
                    std::cout << "Agent_cp " << m_ownAgentID << ": best path from agents are not in consensus" << std::endl;

                    ResetPhase();
                    return;
                }

                std::vector<Common::Pose> processedGoTherePath;
                double headingRad = 0.0;
                for (int i = 0; i < m_ownBestProcessedPath.size(); i++)
                {
                    Common::Pose pose;
                    pose.position = m_ownBestProcessedPath.at(i);

                    if (i+1 < m_ownBestProcessedPath.size())
                    {
                        headingRad = std::atan2(m_ownBestProcessedPath.at(i+1).y() - m_ownBestProcessedPath.at(i).y(),
                                                m_ownBestProcessedPath.at(i+1).x() - m_ownBestProcessedPath.at(i).x());
                    }
                    pose.headingRad = headingRad;

                    processedGoTherePath.push_back(pose);
                }

                m_handlerPtr->m_pubProcessedGoTherePath(m_ownAgentID, processedGoTherePath);

                m_outputObtainedForPhase5 = true;
            }
        }
    }

    void PhaseSynchronizer::Vizualize()
    {
        m_handlerPtr->PubProcessedPointCloudViz(m_ownPointCloud);
        m_handlerPtr->PubOwnPlannedPathViz(m_ownAgentID, m_ownPlannedPath);
        m_handlerPtr->PubProcessedGoTherePathViz(m_ownAgentID, m_ownBestProcessedPath);
    }

}