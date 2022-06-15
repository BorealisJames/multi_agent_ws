#include "../include/teaming_planner/teaming_planner.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormationHandler.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormation.h"

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

            mHandlerPtr->m_getNumberOfAgentsInTeam = std::bind(&TeamingPlanner::getNumberOfAgentsInTeam, this, std::placeholders::_1);
            mHandlerPtr->m_getOwnAgentID = std::bind(&TeamingPlanner::getOwnAgentId, this, std::placeholders::_1);
            mHandlerPtr->m_getPosesForFormationToTrack = std::bind(&TeamingPlanner::getPosesForFormationToTrackRFH, this, std::placeholders::_1); // HIstory of human poses
            mHandlerPtr->m_getPhasesAndTimeRecordOfAgents = std::bind(&TeamingPlanner::getPhaseAndTimeMapRFH, this, std::placeholders::_1);
            mHandlerPtr->m_pubOwnPhaseAndTime = std::bind(&TeamingPlanner::pubPhaseAndTimeRFH, this, std::placeholders::_1, std::placeholders::_2);

            mHandlerPtr->m_clearAgentsPoseBuffer = std::bind(&TeamingPlanner::clearPoseMapRFH, this);
            mHandlerPtr->m_pubOwnPoseFunc = std::bind(&TeamingPlanner::pubPoseRFH, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getOwnAgentPose = std::bind(&TeamingPlanner::getOwnUAVSystemPoseRFH, this, std::placeholders::_1);
            mHandlerPtr->m_getAgentsPose = std::bind(&TeamingPlanner::getPoseMapRFH, this, std::placeholders::_1);
            mHandlerPtr->m_getHumanPose = std::bind(&TeamingPlanner::getHumanSystemPoseRFH, this, std::placeholders::_1);

            mHandlerPtr->m_clearAgentsDirectionUtilityBuffer = std::bind(&TeamingPlanner::clearDirectionUtilityMapRFH, this);
            mHandlerPtr->m_pubOwnDirectionUtility = std::bind(&TeamingPlanner::pubDirectionUtilityRFH, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getAgentsDirectionUtility = std::bind(&TeamingPlanner::getDirectionUtilityMapRFH, this, std::placeholders::_1);

            mHandlerPtr->m_getOwnAgentLidarPointCloud = std::bind(&TeamingPlanner::getOwnAgentLidarPointCloud, this, std::placeholders::_1);
            
            mHandlerPtr->m_clearAgentsConvexRegion2DBuffer = std::bind(&TeamingPlanner::clearConvexRegion2DMapRFH, this);
            mHandlerPtr->m_pubOwnConvex2DRegion = std::bind(&TeamingPlanner::pubConvexRegion2DRFH, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getAgentsConvex2DRegion = std::bind(&TeamingPlanner::getConvexRegion2DMapRFH, this, std::placeholders::_1);
            mHandlerPtr->m_clearAgentsConvexRegion3DBuffer = std::bind(&TeamingPlanner::clearConvexRegion3DMapRFH, this);
            mHandlerPtr->m_pubOwnConvex3DRegion = std::bind(&TeamingPlanner::pubConvexRegion3DRFH, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getAgentsConvex3DRegion = std::bind(&TeamingPlanner::getConvexRegion3DMapRFH, this, std::placeholders::_1);

            mHandlerPtr->m_pubOwnTaskAssignments = std::bind(&TeamingPlanner::pubAssignedPoseMapRFH, this, std::placeholders::_1, std::placeholders::_2);

            mHandlerPtr->m_clearAgentsTaskAssignmentsBuffer = std::bind(&TeamingPlanner::clearPoseMapRFH, this);
            mHandlerPtr->m_getAgentsTaskAssignments = std::bind(&TeamingPlanner::getAssignedVirtualPoseMapRFH, this, std::placeholders::_1);
            mHandlerPtr->m_pubOwnAgentAssignedPose = std::bind(&TeamingPlanner::pubAssignedPoseRFH, this, std::placeholders::_1, std::placeholders::_2);

            mDistributedFormation.AttachHandler(mHandlerPtr);

            mModuleState = TeamingPlannerConstants::ModuleState::READY;
            mModuleStateVerbose = false;

            // quick implementation
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
            switch (mTask.type)
            {
                case Common::Entity::MTTaskEnum::FOLLOW_ME:
                {
                    mgunTargetPoseRecieved = false; // Reset the mode to false
                    if (!mModuleTaskVerbose)
                    {
                        //ROS_INFO("Follow Me\n");
                        mModuleTaskVerbose = true;
                    }
                    mDistributedFormation.RunDistributedFormation();
                    break;
                }
            }

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