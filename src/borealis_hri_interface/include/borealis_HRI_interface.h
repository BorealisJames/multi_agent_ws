#ifndef BOREALIS_FOLLOW_ME_H
#define BOREALIS_FOLLOW_ME_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <unordered_map>

#include "../../distributed_multi_robot_formation/src/Common/Common.h"
#include "../../Common/ConstantsEnum.h"
#include "../../Common/Config/ConfigFileReader.h"


#include <mt_msgs/pose.h>
#include <mt_msgs/mtTask.h>
#include <std_msgs/String.h>

/**
 * @brief Listens to HRI (Human Robot Interface) for Tasks or Goals and Publish it to the Teaming planner
 */


class BorealisHRIInterface
{
    private:
        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;
        Common::Utils::ConfigFileReader mConfigFileReader;

        Common::Entity::MTTaskBundle mTask;
        DistributedFormation::Common::Pose mGoal;

        double mModulePeriod;
        uint32_t mSourceSegmentId;


        // Publisher
        ros::Publisher mTaskPublisher;
        ros::Publisher mGoalPublisher;
        
        ros::Subscriber mHumanTaskSubscriber;

        // Timer
        ros::Timer mModuleLoopTimer;

        void HRITaskCallback(const std_msgs::String::ConstPtr& msg);
        
        // Timer Functions
        void moduleLoopCallback(const ros::TimerEvent& aEvent);

        bool pubTask(const Common::Entity::MTTaskBundle aTask);
        bool pubGoal(DistributedFormation::Common::Pose aGoal);

    public:
        BorealisHRIInterface(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate);
        virtual ~BorealisHRIInterface();

}; // BOREALIS_FOLLOW_ME_H


#endif // BOREALIS_FOLLOW_ME_H
