#include "../include/teaming_planner/teaming_planner.h"
#include "../../distributed_multi_robot_formation/src/DistributedMultiRobotFormationHandler.h"
#include "../../distributed_multi_robot_formation/src/DistributedMultiRobotFormation.h"

void TeamingPlanner::teamingPlannerMain()
{
    switch (mModuleState)
    {
        case TeamingPlannerConstants::ModuleState::INITILAISING:
            if (!mModuleStateVerbose)
            {
                ROS_INFO("Module State Initialising\n");
                mModuleStateVerbose = true;
            }
            
            mHistoryOfHumanPoses.reserve(mPlanningHorizon/mIntervalDistance);

            // Bind common funcs 

            // Bind formation funcs
            mFormationHandlerPtr->m_getNumberOfAgentsInTeam = std::bind(&TeamingPlanner::getNumberOfAgentsInTeam, this, std::placeholders::_1);
            
            mFormationHandlerPtr->m_getOwnAgentID = std::bind(&TeamingPlanner::getOwnAgentId, this, std::placeholders::_1);
            mFormationHandlerPtr->m_getPosesForFormationToTrack_mrf = std::bind(&TeamingPlanner::getPosesForFormationToTrack_mrf, this, std::placeholders::_1); // HIstory of human poses
            mFormationHandlerPtr->m_getPhasesAndTimeRecordOfAgents = std::bind(&TeamingPlanner::getPhaseAndTimeMap_mrf, this, std::placeholders::_1);
            mFormationHandlerPtr->m_pubOwnPhaseAndTime = std::bind(&TeamingPlanner::pubPhaseAndTime_mrf, this, std::placeholders::_1, std::placeholders::_2);

            mFormationHandlerPtr->m_clearAgentsPoseBuffer = std::bind(&TeamingPlanner::clearPoseMap, this);
            mFormationHandlerPtr->m_pubOwnPoseFunc = std::bind(&TeamingPlanner::pubPose_mrf, this, std::placeholders::_1, std::placeholders::_2);
            mFormationHandlerPtr->m_getOwnAgentPose = std::bind(&TeamingPlanner::getOwnUAVSystemPose_mrf, this, std::placeholders::_1);
            mFormationHandlerPtr->m_getAgentsPose = std::bind(&TeamingPlanner::getPoseMap_mrf, this, std::placeholders::_1);
            mFormationHandlerPtr->m_getHumanPose = std::bind(&TeamingPlanner::getHumanSystemPose_mrf, this, std::placeholders::_1);

            mFormationHandlerPtr->m_clearAgentsDirectionUtilityBuffer = std::bind(&TeamingPlanner::clearDirectionUtilityMap, this);
            mFormationHandlerPtr->m_pubOwnDirectionUtility = std::bind(&TeamingPlanner::pubDirectionUtility_mrf, this, std::placeholders::_1, std::placeholders::_2);
            mFormationHandlerPtr->m_getAgentsDirectionUtility = std::bind(&TeamingPlanner::getDirectionUtilityMap_mrf, this, std::placeholders::_1);

            mFormationHandlerPtr->m_getOwnAgentLidarPointCloud = std::bind(&TeamingPlanner::getOwnAgentLidarPointCloud, this, std::placeholders::_1);
            
            mFormationHandlerPtr->m_clearAgentsConvexRegion2DBuffer = std::bind(&TeamingPlanner::clearConvexRegion2DMap, this);
            mFormationHandlerPtr->m_pubOwnConvex2DRegion = std::bind(&TeamingPlanner::pubConvexRegion2D_mrf, this, std::placeholders::_1, std::placeholders::_2);
            mFormationHandlerPtr->m_getAgentsConvex2DRegion = std::bind(&TeamingPlanner::getConvexRegion2DMap_mrf, this, std::placeholders::_1);
            mFormationHandlerPtr->m_clearAgentsConvexRegion3DBuffer = std::bind(&TeamingPlanner::clearConvexRegion3DMap, this);
            mFormationHandlerPtr->m_pubOwnConvex3DRegion = std::bind(&TeamingPlanner::pubConvexRegion3D_mrf, this, std::placeholders::_1, std::placeholders::_2);
            mFormationHandlerPtr->m_getAgentsConvex3DRegion = std::bind(&TeamingPlanner::getConvexRegion3DMap_mrf, this, std::placeholders::_1);

            mFormationHandlerPtr->m_pubOwnTaskAssignments = std::bind(&TeamingPlanner::pubAssignedPoseMap_mrf, this, std::placeholders::_1, std::placeholders::_2);

            mFormationHandlerPtr->m_clearAgentsTaskAssignmentsBuffer = std::bind(&TeamingPlanner::clearPoseMap, this);
            mFormationHandlerPtr->m_getAgentsTaskAssignments = std::bind(&TeamingPlanner::getAssignedVirtualPoseMap_mrf, this, std::placeholders::_1);
            mFormationHandlerPtr->m_pubOwnAgentAssignedPose = std::bind(&TeamingPlanner::pubAssignedPose_mrf, this, std::placeholders::_1, std::placeholders::_2);

            // Bind consensuns_path_planner common funcs

            // Attach the handler
            mDistributedFormation.AttachHandler(mFormationHandlerPtr);

            // Set params 

            // quick implementation
            mModuleState = TeamingPlannerConstants::ModuleState::READY;
            mModuleStateVerbose = false;
            mTask.type = Common::Entity::MTTaskEnum::FOLLOW_ME;
            mNumberOfAgentsInFormation = 2;
            break;

        case TeamingPlannerConstants::ModuleState::READY:
            
            if(!mModuleStateVerbose)
            {
                ROS_INFO("Module State Ready, waiting for Task Command\n");
                mModuleStateVerbose = true;
            }

            break;

        case TeamingPlannerConstants::ModuleState::RUNNING:

            if(!mModuleStateVerbose)
            {
                ROS_INFO("Module State Running\n");
                mModuleStateVerbose = true;
            }
            // switch (mTask.type)
            // {
            //     case Common::Entity::MTTaskEnum::FOLLOW_ME:
            //     {
            //         mgunTargetPoseRecieved = false; // Reset the mode to false
            //         if (!mModuleTaskVerbose)
            //         {
            //             //ROS_INFO("Follow Me\n");
            //             mModuleTaskVerbose = true;
            //         }
            //         mDistributedFormation.RunDistributedFormation();
            //         break;
            //     }
            // }

            //     case Common::Entity::MTTaskEnum::GO_THERE:
            //     {
            //         ROS_INFO("Go there!");
            //         switchToGunTargetPose(mSourceSegmentId);
            //         if (!mModuleTaskVerbose)
            //         {
            //             //ROS_INFO("Go There\n");
            //             mModuleTaskVerbose = true;
            //         }
            //         break;

            //     }
            // }
            break;
        case TeamingPlannerConstants::ModuleState::DEACTIVATED:

            if(!mModuleStateVerbose)
            {
                //ROS_INFO("Module State Deactivated\n");
                mModuleStateVerbose = true;
            }
            break;
        default:

            if(!mModuleStateVerbose)
            {
                //ROS_INFO("Module State Not Applicable\n");
                mModuleStateVerbose = true;
            }
            break;
    }
    // Call benson's class and init paramaters and run his module.
}

bool TeamingPlanner::checkAndAddHumanSystemPose(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses, const DistributedFormation::Common::Pose aPose)
{
    bool status(false);
    if (historyOfHumanPoses.empty())
    {
        historyOfHumanPoses.emplace_back(aPose);
        status = true;
    }
    else
    {
        double tDist = euclideanDistance(historyOfHumanPoses.back().position.x, historyOfHumanPoses.back().position.y, aPose.position.x, aPose.position.y);
        if (tDist >= mIntervalDistance)
        {
            historyOfHumanPoses.emplace_back(aPose);
            status = true;
        }
        else
        {
            status = false;
        }
    }
    return status;
}

double TeamingPlanner::euclideanDistance(const double x1, const double y1, const double x2, const double y2)
{
    double x = (x2-x1)*(x2-x1);
    double y = (y2-y1)*(y2-y1);
    return std::sqrt(x+y);
}