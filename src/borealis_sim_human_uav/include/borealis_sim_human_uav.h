#ifndef BOREALIS_FOLLOW_ME_H
#define BOREALIS_FOLLOW_ME_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <unordered_map>

#include "../../distributed_multi_robot_formation/src/Common/Common.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>

#include "../../distributed_multi_robot_formation/src/Common/Common.h"
#include "../../Common/ConstantsEnum.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mt_msgs/pose.h>
#include <mt_msgs/mtTask.h>
#include <std_msgs/String.h>

/**
 * @brief Essentially starts the human MAVROS stuff and publish the task to the teaming planner.
 */

enum class BorealisMode{UAVSimulationFollow, ROSbagFollow};

class BorealisFollowMe
{
    private:
        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;
        
        // Configurable Variables
        double mModulePeriod = 1;
        double mOffsetDistance = 2.5;
        BorealisMode mMode = BorealisMode::UAVSimulationFollow;

        // Variables
        Common::Entity::Pose mHumanSystemPose; 
        Common::Entity::Pose mHumanLocalPose;
        mavros_msgs::SetMode mOffboardSetMode;
        mavros_msgs::CommandBool mArmCommand;
        mavros_msgs::State mCurrentState;
        mavros_msgs::ParamSet mVelSet;
        
        Common::Entity::MTTaskBundle mTask;
        DistributedFormation::Common::Pose mGoal;

        bool mTaskPublished;
        bool mArmedAndOffboard;
        bool mArmed;
        bool mOffboard;
        bool mWaypointReached;
        bool mLastWaypointReached;

        // Publisher
        ros::Publisher mTaskPublisher;
        ros::Publisher mGoalPublisher;
        ros::Publisher mSetPointPositionPublisher;
        // Subscriber
        ros::Subscriber mMavrosStateSubscriber;
        ros::Subscriber mHumanSystemPoseSubscriber;
        ros::Subscriber mHumanLocalPoseSubscriber;

        // Timer
        ros::Timer mModuleLoopTimer;

        // Subscriber Functions
        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& aState);
        
        // Service Clients
        ros::ServiceClient mArmingClient;
        ros::ServiceClient mSetModeClient;
        ros::ServiceClient mSetMPCVelClient;

        // Timer Functions
        void moduleLoopCallback(const ros::TimerEvent& aEvent);

        bool pubTask(const Common::Entity::MTTaskBundle aTask);
        bool pubGoal(DistributedFormation::Common::Pose aGoal);

    public:
        BorealisFollowMe(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate);
        virtual ~BorealisFollowMe();

}; // BOREALIS_FOLLOW_ME_H


#endif // BOREALIS_FOLLOW_ME_H
