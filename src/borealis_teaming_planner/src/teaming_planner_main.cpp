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

            mHandlerPtr->m_getOwnAgentID = std::bind(&TeamingPlanner::getOwnAgentId, this, std::placeholders::_1);

            mHandlerPtr->m_getHistoryOfHumanPoses = std::bind(&TeamingPlanner::getHistoryOfHumanPoses, this, std::placeholders::_1);
            mHandlerPtr->m_getPhasesAndTimeRecordOfAgents = std::bind(&TeamingPlanner::getPhaseAndTimeMap, this, std::placeholders::_1);
            mHandlerPtr->m_pubOwnPhaseAndTime = std::bind(&TeamingPlanner::pubPhaseAndTime, this, std::placeholders::_1, std::placeholders::_2);

            mHandlerPtr->m_clearAgentsPoseBuffer = std::bind(&TeamingPlanner::clearPoseMap, this);
            mHandlerPtr->m_pubOwnPoseFunc = std::bind(&TeamingPlanner::pubPose, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getOwnAgentPose = std::bind(&TeamingPlanner::getOwnUAVSystemPose, this, std::placeholders::_1);
            mHandlerPtr->m_getAgentsPose = std::bind(&TeamingPlanner::getPoseMap, this, std::placeholders::_1);
            mHandlerPtr->m_getHumanPose = std::bind(&TeamingPlanner::getHumanSystemPose, this, std::placeholders::_1);

            mHandlerPtr->m_clearAgentsDirectionUtilityBuffer = std::bind(&TeamingPlanner::clearDirectionUtilityMap, this);
            mHandlerPtr->m_pubOwnDirectionUtility = std::bind(&TeamingPlanner::pubDirectionUtility, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getAgentsDirectionUtility = std::bind(&TeamingPlanner::getDirectionUtilityMap, this, std::placeholders::_1);

            mHandlerPtr->m_getOwnAgentLidarPointCloud = std::bind(&TeamingPlanner::getOwnAgentLidarPointCloud, this, std::placeholders::_1);
            
            mHandlerPtr->m_clearAgentsConvexRegion2DBuffer = std::bind(&TeamingPlanner::clearConvexRegion2DMap, this);
            mHandlerPtr->m_pubOwnConvex2DRegion = std::bind(&TeamingPlanner::pubConvexRegion2D, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getAgentsConvex2DRegion = std::bind(&TeamingPlanner::getConvexRegion2DMap, this, std::placeholders::_1);
            mHandlerPtr->m_clearAgentsConvexRegion3DBuffer = std::bind(&TeamingPlanner::clearConvexRegion3DMap, this);
            mHandlerPtr->m_pubOwnConvex3DRegion = std::bind(&TeamingPlanner::pubConvexRegion3D, this, std::placeholders::_1, std::placeholders::_2);
            mHandlerPtr->m_getAgentsConvex3DRegion = std::bind(&TeamingPlanner::getConvexRegion3DMap, this, std::placeholders::_1);

            mHandlerPtr->m_pubOwnTaskAssignments = std::bind(&TeamingPlanner::pubAssignedPoseMap, this, std::placeholders::_1, std::placeholders::_2);

            mHandlerPtr->m_clearAgentsTaskAssignmentsBuffer = std::bind(&TeamingPlanner::clearPoseMap, this);
            mHandlerPtr->m_getAgentsTaskAssignments = std::bind(&TeamingPlanner::getAssignedVirtualPoseMap, this, std::placeholders::_1);
            mHandlerPtr->m_pubOwnAgentAssignedPose = std::bind(&TeamingPlanner::pubAssignedPose, this, std::placeholders::_1, std::placeholders::_2);

            mDistributedFormation.AttachHandler(mHandlerPtr);

            mModuleState = TeamingPlannerConstants::ModuleState::READY;
            mModuleStateVerbose = false;
            break;

        case TeamingPlannerConstants::ModuleState::READY:
            
            if(!mModuleStateVerbose)
            {
                //ROS_INFO("Module State Ready, waiting for Task Command\n");
                mModuleStateVerbose = true;
            }

            mControlState = Common::Entity::ControlState::IN_CONTROL;

            break;
        case TeamingPlannerConstants::ModuleState::RUNNING:

            if(!mModuleStateVerbose)
            {
                //ROS_INFO("Module State Running\n");
                mModuleStateVerbose = true;
            }

            pubControlState(mControlState);
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

                case Common::Entity::MTTaskEnum::GO_THERE:
                {
                    // ROS_INFO("Go there!");
                    switchToGunTargetPose(mSourceSegmentId);
                    if (!mModuleTaskVerbose)
                    {
                        //ROS_INFO("Go There\n");
                        mModuleTaskVerbose = true;
                    }
                    break;

                }

                case Common::Entity::MTTaskEnum::DISTRACT_TARGET:
                {
                    if (!mModuleTaskVerbose)
                    {
                        //ROS_INFO("Distract Target\n");
                        mModuleTaskVerbose = true;
                    }
                    break;
                }
                default:
                    //ROS_INFO("Wrong Task Enum\n");
                    break;
            }

            break;
        case TeamingPlannerConstants::ModuleState::CANCELLED:

            if(!mModuleStateVerbose)
            {
                //ROS_INFO("Module State Cancelled\n");
                mModuleStateVerbose = true;
            }
            break;
        case TeamingPlannerConstants::ModuleState::COMPLETED:
            
            if(!mModuleStateVerbose)
            {
                //ROS_INFO("Module State Completed\n");
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