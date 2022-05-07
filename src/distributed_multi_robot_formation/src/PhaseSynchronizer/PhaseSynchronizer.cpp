//
// Created by benson on 13/1/21.
//

#include "PhaseSynchronizer.h"

namespace DistributedFormation
{
    PhaseSynchronizer::PhaseSynchronizer()
    : m_workspace(Common::WORKSPACE::DIM_2_WITH_YAW)
    , m_phase(Common::PHASE::PHASE_1)
    , m_transitingPhase(true)
    , m_handlerPtr(std::make_shared<DistributedMultiRobotFormationHandler>())

    , m_expiryDurationMicroSec(15*1000000)
    , m_numberOfAzimuthDiscreteAnglesOnASide(0)
    , m_resolutionAzimuthAngleRad(0.0)
    , m_numberOfElevationDiscreteAnglesOnASide(0)
    , m_resolutionElevationAngleRad(0.0)
    , m_distanceToFollowBehind(0)
    , m_localBoundingBoxForPathAlongX(3.0)
    , m_localBoundingBoxForPathAlongY(3.0)
    , m_localBoundingBoxForPathAlongZ(2.0)
    , m_pointsMinRange(0.5)
    , m_pointsMaxRange (10.0)
    , m_pointRemovalRadius(0.85)
    , m_desiredDistanceInTriFormation(2.5)
    , m_desiredDistanceInLineFormation(2)
    , m_incrementOffsetToFormationYaw(0)
    //, m_incrementOffsetToFormationYaw(M_PI / 2.0)
    , m_agentRadius(0.2) // 0.35 actual borealis 0.45 to be safe
    , m_waypointReachedBoundary(1.5)
    , m_weightForGoal(0.3)
    , m_weightForRotation(0.3)
    , m_weightForSize(0.4)
    , m_desiredHeight(1.2)
    , m_priorityPenalty(1.0)
    , m_expectedNumberOfAgents(2)
    {
        // Workaround should use config file reader instead 
        double tmp_distance;
        double tmp_agentradius;
        double desiredDistanceinTri;
        double desiredDistanceinLine;
        
        m_handlerPtr->m_nh.getParam("/follow_distance", tmp_distance);
        m_handlerPtr->m_nh.getParam("/agent_radius", tmp_agentradius);
        m_handlerPtr->m_nh.getParam("/desired_tri_length", desiredDistanceinTri);
        m_handlerPtr->m_nh.getParam("/desired_line_length", desiredDistanceinLine);

        m_distanceToFollowBehind = tmp_distance;
        m_agentRadius = tmp_agentradius;
        m_desiredDistanceInTriFormation = desiredDistanceinTri;
        m_desiredDistanceInLineFormation = desiredDistanceinLine;
        std::cout << "agentradius init to be " << m_agentRadius << std::endl;

        if (m_workspace == Common::WORKSPACE::DIM_2_WITH_YAW ||
            m_workspace == Common::WORKSPACE::DIM_2_WITHOUT_YAW)
        {
            m_dimension = Common::DIMENSION::DIM_2;
        }
        else if (m_workspace == Common::WORKSPACE::DIM_3_WITH_ROT ||
                 m_workspace == Common::WORKSPACE::DIM_3_WITHOUT_ROT ||
                 m_workspace == Common::WORKSPACE::DIM_3_WITH_ONLY_YAW)
        {
            m_dimension = Common::DIMENSION::DIM_3;
        }
    }

    void PhaseSynchronizer::AttachHandler(const std::shared_ptr<DistributedMultiRobotFormationHandler>& handlerPtr)
    {
        m_handlerPtr = handlerPtr;

        m_handlerPtr->m_getOwnAgentID(m_ownAgentID);
    }

    void PhaseSynchronizer::SyncPhasesOfAgents()
    {
        //Viz
        Vizualize();

        // check if reset is needed
        UpdatePhasesOfAgentsInTeam();
        UpdatePositionsOfAgentsInTeam();
        UpdatePositionOfHuman();

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
                std::cout << "Agent" << m_ownAgentID << ": Phase1" << std::endl;

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
                std::cout << "Agent" << m_ownAgentID << ": Phase2" << std::endl;

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
                std::cout << "Agent" << m_ownAgentID << ": Phase3" << std::endl;

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
                std::cout << "Agent" << m_ownAgentID << ": Phase4" << std::endl;

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
                std::cout << "Agent" << m_ownAgentID << ": Phase5" << std::endl;

                if (m_transitingPhase)          // transiting in
                {
                    OnEnterPhase5();
                    m_transitingPhase = false;
                }
                if (TransitingFromPhase5())     //transiting out to next phase
                {
                    m_transitingPhase = true;
                    OnExitPhase5();
                }
                else                            //remaining in
                {
                    DoPhase5();
                }
                break;
            }
            default:
            {
                std::cout << "Agent" << m_ownAgentID << ": switch statement phase do not exist" << std::endl;
                ResetPhase();
            }
        }
    }

    void PhaseSynchronizer::UpdatePhasesOfAgentsInTeam()
    {
        m_handlerPtr->m_getPhasesAndTimeRecordOfAgents(m_phasesAndTimeRecordOfAgents);

        auto timeNow = std::chrono::system_clock::now();
        auto timeNowMicroSecs = std::chrono::duration_cast<std::chrono::microseconds>(timeNow.time_since_epoch()).count();

        // update own phase and time to record in case it was not added
        Common::PhaseAndTime ownPhaseAndTime;
        ownPhaseAndTime.phase = m_phase;
        ownPhaseAndTime.timeMicroSecs = timeNowMicroSecs;
        m_phasesAndTimeRecordOfAgents[m_ownAgentID] = ownPhaseAndTime;

        //reset if team members are not equal to number of expecting agents
        if (m_phasesAndTimeRecordOfAgents.size() != m_expectedNumberOfAgents)
        // if (2 != m_expectedNumberOfAgents)
        {
            std::cout << "Agent" << m_ownAgentID << ": reset in update" << std::endl;
            ROS_INFO("Expected Number of agents is: %i, ActualAgentsNumber: %i", m_expectedNumberOfAgents, m_phasesAndTimeRecordOfAgents.size());
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
            std::cout << "Agent" << m_ownAgentID << ": reset in update" << std::endl;

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

    void PhaseSynchronizer::UpdatePositionOfHuman()
    {
        m_handlerPtr->m_getHumanPose(m_humanPose);
    }

    void PhaseSynchronizer::ResetPhase()
    {
        std::cout << "Agent" << m_ownAgentID << ": ResetPhase" << std::endl;

        m_phase = Common::PHASE::PHASE_1;
        m_transitingPhase = true;

        m_poseOfAgentsInTeam.clear();

        m_agentsPose.clear();
        m_agentsAngleIndexUtility.clear();
        m_agents2DConvexRegion.clear();
        m_agents3DConvexRegion.clear();
        m_agentsTaskAssignments.clear();

        m_convexRegionOfAgentPoseObtained = false;
        m_consensusDirectionObtained = false;
        m_taskAllocationObtained = false;
        m_taskAssignmentsVerified = false;
        m_allAgentHasReachedLastPhase = false;

        m_convexHullOfRobotPosition = ConvexHullOfRobotPosition();
        m_directionOfMotion = DirectionOfMotion();
        m_followMeGoalGenerator = FollowMeGoalGenerator();
        m_followMeGoalGenerator.SetParams(
                std::max(m_desiredDistanceInTriFormation, m_desiredDistanceInLineFormation) + m_agentRadius,
                m_distanceToFollowBehind);
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
        std::cout << "Agent" << m_ownAgentID << ": reset in OnEnterPhase1" << std::endl;

        ResetPhase();
        m_handlerPtr->m_clearAgentsPoseBuffer();
    }

    bool PhaseSynchronizer::TransitingFromPhase1()
    {
        if(m_convexRegionOfAgentPoseObtained)
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
        m_convexRegionOfAgentPoseObtained = false;
    }

    void PhaseSynchronizer::DoPhase1()
    {
        if (m_poseOfAgentsInTeam.size() == m_phasesOfAgentsInTeam.size() && m_phasesOfAgentsInTeam.size()>0)
        {
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            //get points that form the convex region
            //input: m_positionOfAgentsInTeam
            //output: agentPositionsThatFormConvexhull
            if (m_dimension == Common::DIMENSION::DIM_2)
            {
                //get convex hull
                Eigen::Matrix<double, Eigen::Dynamic, 2> agentsInputPosition2D (m_poseOfAgentsInTeam.size(), 2);
                Eigen::Matrix<double, Eigen::Dynamic, 2> agentsOutputPosition2D;
                double sumOfSin = 0.0;
                double sumOfCos = 0.0;
                int agentIndex = 0;
                for (auto&& agentInputPosition : m_poseOfAgentsInTeam)
                {
                    sumOfSin += sin(agentInputPosition.second.headingRad);
                    sumOfCos += cos(agentInputPosition.second.headingRad);

                    agentsInputPosition2D(agentIndex,0) = agentInputPosition.second.position.x;
                    agentsInputPosition2D(agentIndex,1) = agentInputPosition.second.position.y;
                    agentIndex++;
                }
                m_convexHullOfRobotPosition.Convexhull2D(agentsInputPosition2D, agentsOutputPosition2D);

                double avgAngleRad = atan2(sumOfSin,sumOfCos);

                m_agentPosesThatFormConvexhull.clear();
                for (int i=0; i<agentsOutputPosition2D.rows(); i++)
                {
                    Common::Pose pose;
                    pose.position.x = agentsOutputPosition2D(i, 0);
                    pose.position.y = agentsOutputPosition2D(i, 1);
                    pose.position.z = m_desiredHeight;
                    pose.headingRad = avgAngleRad;
                    m_agentPosesThatFormConvexhull.push_back(pose);
                }

                m_convexRegionOfAgentPoseObtained = true;
            }
            else if (m_dimension == Common::DIMENSION::DIM_3)
            {
                //get convex hull
                Eigen::Matrix<double, Eigen::Dynamic, 3> agentsInputPosition3D (m_poseOfAgentsInTeam.size(), 3);
                Eigen::Matrix<double, Eigen::Dynamic, 3> agentsOutputPosition3D;
                double sumOfSin = 0.0;
                double sumOfCos = 0.0;
                int agentIndex = 0;
                for (auto&& agentInputPosition : m_poseOfAgentsInTeam)
                {
                    sumOfSin += sin(agentInputPosition.second.headingRad);
                    sumOfCos += cos(agentInputPosition.second.headingRad);

                    agentsInputPosition3D(agentIndex,0) = agentInputPosition.second.position.x;
                    agentsInputPosition3D(agentIndex,1) = agentInputPosition.second.position.y;
                    agentsInputPosition3D(agentIndex,2) = agentInputPosition.second.position.z;
                    agentIndex++;
                }
                m_convexHullOfRobotPosition.Convexhull3D(agentsInputPosition3D, agentsOutputPosition3D);

                double avgAngleRad = atan2(sumOfSin,sumOfCos);

                m_agentPosesThatFormConvexhull.clear();
                for (int i=0; i<agentsOutputPosition3D.rows(); i++)
                {
                    Common::Pose pose;
                    pose.position.x = agentsOutputPosition3D(i, 0);
                    pose.position.y = agentsOutputPosition3D(i, 1);
                    pose.position.z = agentsOutputPosition3D(i, 2);
                    pose.headingRad = avgAngleRad;
                    m_agentPosesThatFormConvexhull.push_back(pose);
                }

                m_convexRegionOfAgentPoseObtained = true;
            }
        }
    }

    /*for phase 2*/
    void PhaseSynchronizer::OnEnterPhase2()
    {
        m_handlerPtr->m_clearAgentsDirectionUtilityBuffer();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        //get direction utility
        //input: avg of extrema points of convex region, human pose position
        //output: ownAgentDirectionUtility

        std::vector<Common::Pose> humanPoses;
        if (!m_handlerPtr->m_getHistoryOfHumanPoses(humanPoses))
        {
            std::cout << "Agent" << m_ownAgentID << ": reset cause unable to get human pose" << std::endl;

            ResetPhase();
            return;
        }

        //Choose to use either m_agentPosesThatFormConvexhull or m_poseOfAgentsInTeam to get avg of extrema points
        //currently the m_agentPosesThatFormConvexhull does not seem to be that stable. Do try again when code is more stable.
        double maxX = -DBL_MAX;
        double minX = DBL_MAX;
        double maxY = -DBL_MAX;
        double minY = DBL_MAX;
        double maxZ = -DBL_MAX;
        double minZ = DBL_MAX;
        double sumOfSin = 0.0;
        double sumOfCos = 0.0;
        m_avgOfExtremaPose.position.x = 0;
        m_avgOfExtremaPose.position.y = 0;
        m_avgOfExtremaPose.position.z = 0;
        m_avgOfExtremaPose.headingRad = 0;

        int numberOfAgents = m_poseOfAgentsInTeam.size();
        if (numberOfAgents <= 0)
        {
            std::cout << "Agent" << m_ownAgentID << ": reset cause unable to get at least 1 agent to form avg of extrema" << std::endl;

            ResetPhase();
            return;
        }


        for (auto &&agentPose : m_poseOfAgentsInTeam)
        {
            maxX = std::max(maxX, agentPose.second.position.x);
            minX = std::min(minX, agentPose.second.position.x);
            maxY = std::max(maxY, agentPose.second.position.y);
            minY = std::min(minY, agentPose.second.position.y);
            maxZ = std::max(maxZ, agentPose.second.position.z);
            minZ = std::min(minZ, agentPose.second.position.z);

            sumOfSin += sin(agentPose.second.headingRad);
            sumOfCos += cos(agentPose.second.headingRad);
        }
        m_avgOfExtremaPose.headingRad = atan2(sumOfSin, sumOfCos);
        m_avgOfExtremaPose.position.x = (maxX + minX) / 2;
        m_avgOfExtremaPose.position.y = (maxY + minY) / 2;

        if (m_dimension == Common::DIMENSION::DIM_2)
        {
            m_avgOfExtremaPose.position.z = m_desiredHeight;
        }
        else if (m_dimension == Common::DIMENSION::DIM_3)
        {
            m_avgOfExtremaPose.position.z = (maxZ + minZ) / 2;
        }

        //get goal for formation avg of bounds to move to
        if (!m_followMeGoalGenerator.GetGoalFromHumanPosesAndAvgOfExtremaPose(humanPoses, m_avgOfExtremaPose,
                                                                              m_goal))
        {
            std::cout << "Agent" << m_ownAgentID << ": reset cause unable to get goal for formation to move to" << std::endl;

            ResetPhase();
            return;
        }

        double distanceToGoal = std::sqrt(std::pow(m_avgOfExtremaPose.position.x - m_goal.position.x, 2) +
                                          std::pow(m_avgOfExtremaPose.position.y - m_goal.position.y, 2) +
                                          std::pow(m_avgOfExtremaPose.position.z - m_goal.position.z, 2) );

        if (distanceToGoal>m_waypointReachedBoundary)
        {
            double ratio = m_waypointReachedBoundary/distanceToGoal;
            m_goal.position.x = ratio*(m_goal.position.x-m_avgOfExtremaPose.position.x) + m_avgOfExtremaPose.position.x;
            m_goal.position.y = ratio*(m_goal.position.y-m_avgOfExtremaPose.position.y) + m_avgOfExtremaPose.position.y;
            m_goal.position.z = ratio*(m_goal.position.z-m_avgOfExtremaPose.position.z) + m_avgOfExtremaPose.position.z;
        }

        //calculating an angle when the position of drone is near a goal (assuming drone and goal are at the same level)
        double expansionAzimuthAngleRad = 0.0;
        if (std::abs(m_goal.position.x - m_avgOfExtremaPose.position.x) < m_waypointReachedBoundary &&
            std::abs(m_goal.position.y - m_avgOfExtremaPose.position.y) < m_waypointReachedBoundary)
        {
            expansionAzimuthAngleRad = m_goal.headingRad;
        }
        else
        {
            expansionAzimuthAngleRad = atan2((m_goal.position.y - m_avgOfExtremaPose.position.y),
                                             (m_goal.position.x - m_avgOfExtremaPose.position.x));
        }

        double expansionElevationAngleRad = 0.0;
        if (m_dimension == Common::DIMENSION::DIM_3 &&
            std::abs(m_goal.position.z - m_avgOfExtremaPose.position.z) > m_waypointReachedBoundary)
        {
            double magnitude = std::sqrt(std::pow(m_goal.position.x - m_avgOfExtremaPose.position.x, 2) +
                                         std::pow(m_goal.position.y - m_avgOfExtremaPose.position.y, 2) +
                                         std::pow(m_goal.position.z - m_avgOfExtremaPose.position.z, 2));
            expansionElevationAngleRad = asin((m_goal.position.z - m_avgOfExtremaPose.position.z) / magnitude);
        }

        m_directionOfMotion.SetStartingAngleAndDiscretizationParams(expansionAzimuthAngleRad,
                                                                    expansionElevationAngleRad,
                                                                    m_numberOfAzimuthDiscreteAnglesOnASide,
                                                                    m_resolutionAzimuthAngleRad,
                                                                    m_numberOfElevationDiscreteAnglesOnASide,
                                                                    m_resolutionElevationAngleRad);

        size_t numberOfAngles = m_directionOfMotion.NumberOfDiscretizedAngles();
        m_ownAgentAngleIndexUtility.angleIndexAndUtility.clear();
        m_ownAgentAngleIndexUtility.angleIndexAndUtility.resize(numberOfAngles);

        for (int i = 0; i < numberOfAngles; i++)
        {
            double utility = m_directionOfMotion.UtilityForAngleIndex(i);
            m_ownAgentAngleIndexUtility.angleIndexAndUtility.at(i) = utility;
        }

        //set own direction utility in team
        m_agentsAngleIndexUtility[m_ownAgentID] = m_ownAgentAngleIndexUtility;
    }

    bool PhaseSynchronizer::TransitingFromPhase2()
    {
        if(m_consensusDirectionObtained)
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
        m_consensusDirectionObtained = false;
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
            //pub phase 2 message, agent's direction utility (will send out id and direction utility)
            m_handlerPtr->m_pubOwnDirectionUtility(m_ownAgentID, m_ownAgentAngleIndexUtility);

            //get other agent's direction utility
            m_handlerPtr->m_getAgentsDirectionUtility(m_agentsAngleIndexUtility);
            //set own agent's direction utility
            m_agentsAngleIndexUtility[m_ownAgentID] = m_ownAgentAngleIndexUtility;

            std::unordered_map<int32_t, Common::DirectionUtility> agentsDirectionUtilityCopy;
            for(auto&&agent : m_phasesOfAgentsInTeam)
            {
                int32_t agentID = agent.first;
                if (m_agentsAngleIndexUtility.find(agentID) != m_agentsAngleIndexUtility.end())
                {
                    agentsDirectionUtilityCopy[agentID] = m_agentsAngleIndexUtility[agentID];
                }
            }

            if (agentsDirectionUtilityCopy.size() == m_phasesOfAgentsInTeam.size())
            {
                //get direction
                //input: agentsDirectionUtility
                //output: m_desiredExpansionAzimuthAngleRad, m_desiredExpansionElevationAngleRad

                if (!m_directionOfMotion.ConsensusOnDirection(m_agentsAngleIndexUtility,
                                                              m_desiredExpansionAzimuthAngleRad, m_desiredExpansionElevationAngleRad))
                {
                    std::cout << "Agent" << m_ownAgentID << ": reset cause consensus on 2D direction failed" << std::endl;

                    ResetPhase();
                    return;
                }

                m_consensusDirectionObtained = true;
            }
        }
    }

    /*for phase 3*/
    void PhaseSynchronizer::OnEnterPhase3()
    {
        //for Dim_2
        m_handlerPtr->m_clearAgentsConvexRegion2DBuffer();
        //for Dim_3
        m_handlerPtr->m_clearAgentsConvexRegion3DBuffer();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////
        //get own convex region
        //input: directionForConvexExpansion, convexRegionOfAgentPosition, obstacles(points from a cube), dynamic obstacles(points from a cube and vel)
        //output: ownAgentObstacleFreeConvexRegion

        sensor_msgs::PointCloud lidarPointCloud;
        sensor_msgs::PointCloud2 lidarPointCloud2;
        sensor_msgs::PointCloud cameraPointCloud;

        // //process latest lidar and camera point cloud with latest agent positions
        // if (!m_handlerPtr->m_getOwnAgentLidarPointCloud2(lidarPointCloud2) &&
        //     !m_handlerPtr->m_getOwnAgentCameraPointCloud(cameraPointCloud))


        if (!m_handlerPtr->m_getOwnAgentLidarPointCloud(lidarPointCloud) &&
            !m_handlerPtr->m_getOwnAgentCameraPointCloud(cameraPointCloud))
        {
            std::cout << "Agent" << m_ownAgentID << ": unable to get both lidar and camera point cloud" << std::endl;

            ResetPhase();
            return;
        }

        // Apply Voxel filter and Convert to pointcloud2
        // ROS_INFO("Phase cppPoint cloud size is %i",lidarPointCloud2.fields.size());

        //Note only using lidar and not depth camera
        //m_processPointCloud.AppendPointClouds(lidarPointCloud, cameraPointCloud, m_ownPointCloud);
        m_ownPointCloud = lidarPointCloud;

        if (m_dimension == Common::DIMENSION::DIM_2)
        {
            sensor_msgs::PointCloud tmpOutputPointCloud;

            // m_processPointCloud.RemovePointsOutsideOfRadiusRangeFromPointCloud2D(m_ownPointCloud,
            //                                                                     m_ownAgentPose.position,
            //                                                                     m_pointsMinRange,
            //                                                                     m_pointsMaxRange,
            //                                                                     tmpOutputPointCloud);
            // m_ownPointCloud = tmpOutputPointCloud;

            for(auto&& agentPose : m_poseOfAgentsInTeam)
            {
                m_processPointCloud.RemovePointsWithinARadiusAndFromGroundFromPointCloud2D(m_ownPointCloud,
                                                                                               agentPose.second.position,
                                                                                               m_pointRemovalRadius,
                                                                                               tmpOutputPointCloud);
                m_ownPointCloud = tmpOutputPointCloud;
            }
            m_processPointCloud.RemovePointsWithinARadiusAndFromGroundFromPointCloud2D(m_ownPointCloud,
                                                                                           m_humanPose.position,
                                                                                           m_pointRemovalRadius,
                                                                                           tmpOutputPointCloud);
            m_ownPointCloud = tmpOutputPointCloud;
        }
        else if (m_dimension == Common::DIMENSION::DIM_3)
        {
            sensor_msgs::PointCloud tmpOutputPointCloud;

            // m_processPointCloud.RemovePointsOutsideOfRadiusRangeFromPointCloud3D(m_ownPointCloud,
            //                                                                     m_ownAgentPose.position,
            //                                                                     m_pointsMinRange,
            //                                                                     m_pointsMaxRange,
            //                                                                     tmpOutputPointCloud);
            // m_ownPointCloud = tmpOutputPointCloud;

            for(auto&& agentPose : m_poseOfAgentsInTeam)
            {
                m_processPointCloud.RemovePointsWithinARadiusPointCloud3D(m_ownPointCloud,
                                                                               agentPose.second.position,
                                                                               m_pointRemovalRadius,
                                                                               tmpOutputPointCloud);
                m_ownPointCloud = tmpOutputPointCloud;
            }
            m_processPointCloud.RemovePointsWithinARadiusPointCloud3D(m_ownPointCloud,
                                                                           m_humanPose.position,
                                                                           m_pointRemovalRadius,
                                                                           tmpOutputPointCloud);
            m_ownPointCloud = tmpOutputPointCloud;
        }

        //add back human position as a point
        geometry_msgs::Point32 humanPosition;
        humanPosition.x = m_humanPose.position.x;
        humanPosition.y = m_humanPose.position.y;
        humanPosition.z = m_humanPose.position.z;
        m_ownPointCloud.points.push_back(humanPosition);

        //make a 'screen' for the human
        double screenChangeLimit = 1.0;
        double screenResolution = 1.0;

        for (double height = -screenChangeLimit; height <= screenChangeLimit; height+=screenResolution)
        {
            humanPosition.x = m_humanPose.position.x;
            humanPosition.y = m_humanPose.position.y;
            humanPosition.z = m_humanPose.position.z + height;
            m_ownPointCloud.points.push_back(humanPosition);
            for (double horizontal = screenResolution; horizontal <= screenChangeLimit; horizontal+=screenResolution)
            {
                humanPosition.x = horizontal * std::cos(m_humanPose.headingRad + M_PI/2) + m_humanPose.position.x;
                humanPosition.y = horizontal * std::sin(m_humanPose.headingRad + M_PI/2) + m_humanPose.position.y;
                humanPosition.z = m_humanPose.position.z + height;
                m_ownPointCloud.points.push_back(humanPosition);
                humanPosition.x = horizontal * std::cos(m_humanPose.headingRad - M_PI/2) + m_humanPose.position.x;
                humanPosition.y = horizontal * std::sin(m_humanPose.headingRad - M_PI/2) + m_humanPose.position.y;
                humanPosition.z = m_humanPose.position.z + height;
                m_ownPointCloud.points.push_back(humanPosition);
            }
        }

        if (m_dimension == Common::DIMENSION::DIM_2)
        {
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> AFromPoint;
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bFromPoint;

            double distanceToGoal = std::sqrt(std::pow(m_avgOfExtremaPose.position.x - m_goal.position.x, 2) +
                                              std::pow(m_avgOfExtremaPose.position.y - m_goal.position.y, 2) );

            Polyhedron<2> poly2DViz;
            m_generateConvexRegions.Generate2DConvexRegionFromPoint(m_ownPointCloud,
                                                                    m_avgOfExtremaPose.position,
                                                                    0.1,
                                                                    m_localBoundingBoxForPathAlongX + distanceToGoal,
                                                                    m_localBoundingBoxForPathAlongY + distanceToGoal,
                                                                    AFromPoint,
                                                                    bFromPoint,
                                                                    poly2DViz);

            m_polys2DViz.clear();
            m_polys2DViz.push_back(poly2DViz);


            Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> AFromPath;
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bFromPath;
            m_generateConvexRegions.Generate2DConvexRegionFromPath(m_ownPointCloud,
                                                                   m_avgOfExtremaPose.position,
                                                                   m_goal.position,
                                                                   m_localBoundingBoxForPathAlongX,
                                                                   m_localBoundingBoxForPathAlongY,
                                                                   AFromPath,
                                                                   bFromPath,
                                                                   m_polys2DViz);



            std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>> AVec;
            std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>> bVec;
            AVec.push_back(AFromPoint);
            AVec.push_back(AFromPath);
            bVec.push_back(bFromPoint);
            bVec.push_back(bFromPath);

            Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> AReduced;
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bReduced;
            if (!m_generateConvexRegions.Intersect2DConvexRegion(AVec, bVec,
                                                                 AReduced, bReduced))
            {
                std::cout << "Agent" << m_ownAgentID << ": reset cause unable to find intersection region" << std::endl;

                ResetPhase();
                return;
            }

            m_ownAgent2DConvexRegion.A = AReduced;
            m_ownAgent2DConvexRegion.b = bReduced;
        }
        else if (m_dimension == Common::DIMENSION::DIM_3)
        {
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> AFromPoint;
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bFromPoint;

            double distanceToGoal = std::sqrt(std::pow(m_avgOfExtremaPose.position.x - m_goal.position.x, 2) +
                                              std::pow(m_avgOfExtremaPose.position.y - m_goal.position.y, 2) +
                                              std::pow(m_avgOfExtremaPose.position.z - m_goal.position.z, 2) );

            Polyhedron<3> poly3DViz;
            m_generateConvexRegions.Generate3DConvexRegionFromPoint(m_ownPointCloud,
                                                                    m_avgOfExtremaPose.position,
                                                                    0.1,
                                                                    m_localBoundingBoxForPathAlongX+distanceToGoal,
                                                                    m_localBoundingBoxForPathAlongY+distanceToGoal,
                                                                    m_localBoundingBoxForPathAlongZ+distanceToGoal,
                                                                    AFromPoint,
                                                                    bFromPoint,
                                                                    poly3DViz);

            m_polys3DViz.clear();
            m_polys3DViz.push_back(poly3DViz);


            Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> AFromPath;
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bFromPath;
            m_generateConvexRegions.Generate3DConvexRegionFromPath(m_ownPointCloud,
                                                                   m_avgOfExtremaPose.position,
                                                                   m_goal.position,
                                                                   m_localBoundingBoxForPathAlongX,
                                                                   m_localBoundingBoxForPathAlongY,
                                                                   m_localBoundingBoxForPathAlongZ,
                                                                   AFromPath,
                                                                   bFromPath,
                                                                   m_polys3DViz);


            std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>> AVec;
            std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>> bVec;
            AVec.push_back(AFromPoint);
            AVec.push_back(AFromPath);
            bVec.push_back(bFromPoint);
            bVec.push_back(bFromPath);

            Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> AReduced;
            Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bReduced;
            if (!m_generateConvexRegions.Intersect3DConvexRegion(AVec, bVec,
                                                                 AReduced, bReduced))
            {
                std::cout << "Agent" << m_ownAgentID << ": reset cause unable to find intersection region" << std::endl;

                ResetPhase();
                return;
            }

            m_ownAgent3DConvexRegion.A = AReduced;
            m_ownAgent3DConvexRegion.b = bReduced;
        }

    }

    bool PhaseSynchronizer::TransitingFromPhase3()
    {
        if(m_taskAllocationObtained)
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
        m_taskAllocationObtained = false;
    }

    void PhaseSynchronizer::DoPhase3()
    {
        if (CheckIfAnyAgentIsStillInPreviousPhase())
        {
            //publish phase 2 message, agent's direction utility (will send out id and direction utility)
            m_handlerPtr->m_pubOwnDirectionUtility(m_ownAgentID, m_ownAgentAngleIndexUtility);
        }
        else
        {
            if (m_dimension == Common::DIMENSION::DIM_2)
            {
                //pub phase 3 message, agent's convex region (will send out id and convex region)
                m_handlerPtr->m_pubOwnConvex2DRegion(m_ownAgentID, m_ownAgent2DConvexRegion);

                //get other agent's convex region
                m_handlerPtr->m_getAgentsConvex2DRegion(m_agents2DConvexRegion);

                //set own agent's convex region
                m_agents2DConvexRegion[m_ownAgentID] = m_ownAgent2DConvexRegion;

                std::unordered_map<int32_t, Common::ConvexRegion2D> agents2DConvexRegionCopy;
                for(auto&&agent : m_phasesOfAgentsInTeam)
                {
                    int32_t agentID = agent.first;
                    if (m_agents2DConvexRegion.find(agentID) != m_agents2DConvexRegion.end())
                    {
                        agents2DConvexRegionCopy[agentID] = m_agents2DConvexRegion[agentID];
                    }
                }

                if (agents2DConvexRegionCopy.size() == m_phasesOfAgentsInTeam.size())
                {
                    //get intersected obstacle free region
                    //input: agentsConvexRegionCopy
                    //output: obstacleFreeIntersectedRegion
                    std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>> AVec;
                    std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>> bVec;
                    for (auto&& convexRegion : agents2DConvexRegionCopy)
                    {
                        AVec.push_back(convexRegion.second.A);
                        bVec.push_back(convexRegion.second.b);
                    }
                    Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> AReduced;
                    Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bReduced;
                    if (!m_generateConvexRegions.Intersect2DConvexRegion(AVec, bVec,
                                                                         AReduced, bReduced))
                    {
                        std::cout << "Agent" << m_ownAgentID << ": reset cause unable to find intersection region" << std::endl;

                        ResetPhase();
                        return;
                    }

                    if (!m_generateConvexRegions.Normlize2DHalfSpace(AReduced, bReduced))
                    {
                        std::cout << "Agent" << m_ownAgentID << ": reset cause unable to norm half space" << std::endl;

                        ResetPhase();
                        return;
                    }

                    //do optimization with ANormalized and bNormalized
                    std::unordered_map<uint32_t, Common::Position> optVirtualPositions;
                    double optDeltaX, optDeltaY, optDeltaYaw, optDeltaSize;
                    Common::Formation2DType formation2DType;

		            ROS_INFO("Desired goal %f, %f. Yaw, %f",m_goal.position.x , m_goal.position.y,  Common::MinusPiToPi(m_goal.headingRad + m_incrementOffsetToFormationYaw));
                    ROS_INFO("AReduced matrix:");
                    std::cout << AReduced << std::endl;
                    ROS_INFO("BReduced matrix:");
                    std::cout << bReduced << std::endl;

                    bool optSuccess = Formation2D::GetOptimizedPositions2DInFormation(m_workspace,
                                                                                        m_expectedNumberOfAgents,
                                                                                        m_agentRadius,
                                                                                        m_desiredDistanceInLineFormation,
                                                                                        m_desiredDistanceInTriFormation,
                                                                                        m_desiredDistanceInLineFormation,
                                                                                        m_goal.position.x,
                                                                                        m_goal.position.y,
                                                                                        Common::MinusPiToPi(m_goal.headingRad + m_incrementOffsetToFormationYaw),
                                                                                        m_weightForGoal,
                                                                                        m_weightForRotation,
                                                                                        m_weightForSize,
                                                                                        AReduced,
                                                                                        bReduced,
                                                                                        m_desiredHeight,
                                                                                        m_priorityPenalty,
                                                                                        optVirtualPositions,
                                                                                        optDeltaX,
                                                                                        optDeltaY,
                                                                                        optDeltaYaw,
                                                                                        optDeltaSize,
                                                                                        formation2DType);

                    if (!optSuccess)
                    {
                        std::cout << "Agent" << m_ownAgentID << ": reset cause no feasible formation found after optimization" << std::endl;

                        ResetPhase();
                        return;
                    }

                    //do task allocation (calculate cost for other agents since their position is known)
                    std::unordered_map<int, uint32_t> assignedTaskIndexMap;
                    VirtualPositionAssignment virtualPositionAssignment;

                    virtualPositionAssignment.AssignPositionUsingMunkres(optVirtualPositions, m_poseOfAgentsInTeam,
                                                                         assignedTaskIndexMap);

                    m_taskAllocationObtained = true;
                    // pub task assignments
                    m_ownTaskAssignments.clear();
                    for (auto&& assignedTaskIndex : assignedTaskIndexMap)
                    {
                        int agentOfInterest = assignedTaskIndex.first;
                        uint32_t taskOfInterest = assignedTaskIndex.second;
                        if (optVirtualPositions.find(taskOfInterest) != optVirtualPositions.end())
                        {
                            Common::Pose assignedPose;
                            assignedPose.position = optVirtualPositions[taskOfInterest];
                            assignedPose.headingRad = m_goal.headingRad;

                            m_ownTaskAssignments[agentOfInterest] = assignedPose;
                        }
                    }

                    m_handlerPtr->m_pubOwnTaskAssignments(m_ownAgentID, m_ownTaskAssignments);
                }
            }
            else if (m_dimension == Common::DIMENSION::DIM_3)
            {
                //pub phase 3 message, agent's convex region (will send out id and convex region)
                m_handlerPtr->m_pubOwnConvex3DRegion(m_ownAgentID, m_ownAgent3DConvexRegion);

                //get other agent's convex region
                m_handlerPtr->m_getAgentsConvex3DRegion(m_agents3DConvexRegion);

                //set own agent's convex region
                m_agents3DConvexRegion[m_ownAgentID] = m_ownAgent3DConvexRegion;

                std::unordered_map<int32_t, Common::ConvexRegion3D> agents3DConvexRegionCopy;
                for(auto&&agent : m_phasesOfAgentsInTeam)
                {
                    int32_t agentID = agent.first;
                    if (m_agents3DConvexRegion.find(agentID) != m_agents3DConvexRegion.end())
                    {
                        agents3DConvexRegionCopy[agentID] = m_agents3DConvexRegion[agentID];
                    }
                }

                if (agents3DConvexRegionCopy.size() == m_phasesOfAgentsInTeam.size())
                {
                    //get intersected obstacle free region
                    //input: agentsConvexRegionCopy
                    //output: obstacleFreeIntersectedRegion
                    std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>> AVec;
                    std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>> bVec;
                    for (auto&& convexRegion : agents3DConvexRegionCopy)
                    {
                        AVec.push_back(convexRegion.second.A);
                        bVec.push_back(convexRegion.second.b);
                    }
                    Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> AReduced;
                    Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bReduced;
                    if (!m_generateConvexRegions.Intersect3DConvexRegion(AVec, bVec,
                                                                         AReduced, bReduced))
                    {
                        std::cout << "Agent" << m_ownAgentID << ": reset cause unable to find intersection region" << std::endl;

                        ResetPhase();
                        return;
                    }

                    if (!m_generateConvexRegions.Normlize3DHalfSpace(AReduced, bReduced))
                    {
                        std::cout << "Agent" << m_ownAgentID << ": reset cause unable to norm half space" << std::endl;

                        ResetPhase();
                        return;
                    }

                    //convert goal rotation to desried quaternion
                    double roll = 0.0;
                    double pitch = 0.0;
                    double yaw = Common::MinusPiToPi(m_goal.headingRad + m_incrementOffsetToFormationYaw);
                    Eigen::Quaterniond desiredQ = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                                                  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                                  * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

                    //do optimization with ANormalized and bNormalized
                    std::unordered_map<uint32_t, Common::Position> optVirtualPositions;
                    double optDeltaX, optDeltaY, optDeltaZ, optDeltaQw, optDeltaQx, optDeltaQy, optDeltaQz, optDeltaSize;
                    Common::Formation3DType formation3DType;
                    bool optSuccess = Formation3D::GetOptimizedPositions3DInFormation(m_workspace,
                                                                                      m_expectedNumberOfAgents,
                                                                                      m_agentRadius,
                                                                                      m_desiredDistanceInLineFormation,
                                                                                      m_desiredDistanceInTriFormation,
                                                                                      m_desiredDistanceInLineFormation,
                                                                                      m_goal.position.x, m_goal.position.y, m_goal.position.z,
                                                                                      desiredQ.w(), desiredQ.x(), desiredQ.y(), desiredQ.z(),
                                                                                      m_weightForGoal,
                                                                                      m_weightForRotation,
                                                                                      m_weightForSize,
                                                                                      AReduced,
                                                                                      bReduced,
                                                                                      m_priorityPenalty,
                                                                                      optVirtualPositions,
                                                                                      optDeltaX,
                                                                                      optDeltaY,
                                                                                      optDeltaZ,
                                                                                      optDeltaQw,
                                                                                      optDeltaQx,
                                                                                      optDeltaQy,
                                                                                      optDeltaQz,
                                                                                      optDeltaSize,
                                                                                      formation3DType);

                    if(!optSuccess)
                    {
                        std::cout << "Agent" << m_ownAgentID << ": reset cause no feasible formation found after optimization" << std::endl;

                        ResetPhase();
                        return;
                    }

                    //do task allocation (calculate cost for other agents since their position is known)
                    std::unordered_map<int, uint32_t> assignedTaskIndexMap;
                    VirtualPositionAssignment virtualPositionAssignment;

                    virtualPositionAssignment.AssignPositionUsingMunkres(optVirtualPositions, m_poseOfAgentsInTeam,
                                                                         assignedTaskIndexMap);

                    m_taskAllocationObtained = true;
                    // pub task assignments
                    m_ownTaskAssignments.clear();
                    for (auto&& assignedTaskIndex : assignedTaskIndexMap)
                    {
                        int agentOfInterest = assignedTaskIndex.first;
                        uint32_t taskOfInterest = assignedTaskIndex.second;
                        if (optVirtualPositions.find(taskOfInterest) != optVirtualPositions.end())
                        {
                            Common::Pose assignedPose;
                            assignedPose.position = optVirtualPositions[taskOfInterest];
                            assignedPose.headingRad = m_goal.headingRad;

                            m_ownTaskAssignments[agentOfInterest] = assignedPose;
                        }
                    }

                    m_handlerPtr->m_pubOwnTaskAssignments(m_ownAgentID, m_ownTaskAssignments);
                }
            }
        }
    }

    /*for phase 4*/
    void PhaseSynchronizer::OnEnterPhase4()
    {
        //clear other agents optimized formation parameters
        m_agentsTaskAssignments.clear();
    }

    bool PhaseSynchronizer::TransitingFromPhase4()
    {
        if(m_taskAssignmentsVerified)
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
        m_taskAssignmentsVerified = false;
    }

    void PhaseSynchronizer::DoPhase4()
    {
        if (CheckIfAnyAgentIsStillInPreviousPhase())
        {
            //pub phase 3 message, agent's convex region (will send out id and convex region)
            if (m_dimension == Common::DIMENSION::DIM_2)
            {
                m_handlerPtr->m_pubOwnConvex2DRegion(m_ownAgentID, m_ownAgent2DConvexRegion);
            }
            else if (m_dimension == Common::DIMENSION::DIM_3)
            {
                m_handlerPtr->m_pubOwnConvex3DRegion(m_ownAgentID, m_ownAgent3DConvexRegion);
            }
        }
        else
        {
            //pub phase 4 message, agent's task assignments (will send out id and task assignemnt positions)
            m_handlerPtr->m_pubOwnTaskAssignments(m_ownAgentID, m_ownTaskAssignments);

            //get other agent's task assignments
            m_handlerPtr->m_getAgentsTaskAssignments(m_agentsTaskAssignments);

            //set own agent's task assignments
            m_agentsTaskAssignments[m_ownAgentID] = m_ownTaskAssignments;

            std::vector<Common::Pose> ownTaskAssignmentsVec;
            for (auto&&taskAssignment : m_ownTaskAssignments)
            {
                ownTaskAssignmentsVec.push_back(taskAssignment.second);
            }

            std::vector<std::unordered_map<int32_t, Common::Pose>> agentsTaskAssignmentsVec;
            for(auto&&agent : m_phasesOfAgentsInTeam)
            {
                int32_t agentID = agent.first;
                if (m_agentsTaskAssignments.find(agentID) != m_agentsTaskAssignments.end())
                {
                    agentsTaskAssignmentsVec.push_back(m_agentsTaskAssignments[agentID]);
                }
            }

            if (agentsTaskAssignmentsVec.size() == m_phasesOfAgentsInTeam.size())
            {
                //check that agents assignments are the same for all agents

                bool taskAssignmentsVerified = true;

                for (int i = 0; i < static_cast<int>(agentsTaskAssignmentsVec.size())-1 && taskAssignmentsVerified; ++i)
                {
                    std::unordered_map<int32_t, Common::Pose> agentTaskAssignmentsMap1 = agentsTaskAssignmentsVec.at(i);
                    for (int j = i+1; j < static_cast<int>(agentsTaskAssignmentsVec.size()) && taskAssignmentsVerified; ++j)
                    {
                        std::unordered_map<int32_t, Common::Pose> agentTaskAssignmentsMap2 = agentsTaskAssignmentsVec.at(j);

                        if (agentTaskAssignmentsMap1.size() != agentTaskAssignmentsMap2.size())
                        {
                            taskAssignmentsVerified = false;
                            break;
                        }
                        for (auto&& taskAssignment1 : agentTaskAssignmentsMap1)
                        {
                            int32_t agentID = taskAssignment1.first;
                            if (agentTaskAssignmentsMap2.find(agentID) == agentTaskAssignmentsMap2.end())
                            {
                                taskAssignmentsVerified = false;
                                break;
                            }
                            else
                            {
                                if (std::abs(agentTaskAssignmentsMap1[agentID].position.x - agentTaskAssignmentsMap2[agentID].position.x) >= 1.0 ||
                                    std::abs(agentTaskAssignmentsMap1[agentID].position.y - agentTaskAssignmentsMap2[agentID].position.y) >= 1.0 ||
                                    std::abs(agentTaskAssignmentsMap1[agentID].position.z - agentTaskAssignmentsMap2[agentID].position.z) >= 1.0)
                                {
                                    taskAssignmentsVerified = false;
                                    break;
                                }
                            }

                        }
                    }
                }

                if (!taskAssignmentsVerified)
                {
                    std::cout << "Agent" << m_ownAgentID << ": reset cause task assignments do not match" << std::endl;

                    ResetPhase();
                    return;
                }
                else if (m_ownTaskAssignments.find(m_ownAgentID) == m_ownTaskAssignments.end())
                {
                    std::cout << "Agent" << m_ownAgentID << ": reset cause could not find own task assignments" << std::endl;

                    ResetPhase();
                    return;
                }
                else
                {
                    m_handlerPtr->m_pubOwnAgentAssignedPose(m_ownAgentID, m_ownTaskAssignments[m_ownAgentID]);
                    m_taskAssignmentsVerified = true;
                }
            }

        }
    }


    /*for phase 5*/
    void PhaseSynchronizer::OnEnterPhase5()
    {

    }

    bool PhaseSynchronizer::TransitingFromPhase5()
    {
        if(m_allAgentHasReachedLastPhase)
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
        m_allAgentHasReachedLastPhase = false;
    }

    void PhaseSynchronizer::DoPhase5()
    {
        if (CheckIfAnyAgentIsStillInPreviousPhase())
        {
            //pub phase 4 message, agent's task assignments (will send out id and task assignemnt positions)
            m_handlerPtr->m_pubOwnTaskAssignments(m_ownAgentID, m_ownTaskAssignments);
        }
        else
        {
            m_allAgentHasReachedLastPhase = true;
        }
    }

    void PhaseSynchronizer::Vizualize()
    {
        m_handlerPtr->PubProcessedPointCloud(m_ownPointCloud);
        m_handlerPtr->Pub2DPolyUAV(m_polys2DViz);
        m_handlerPtr->Pub3DPolyUAV(m_polys3DViz);
    }
}
