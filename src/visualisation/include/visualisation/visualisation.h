#ifndef VISUALISATION_H
#define VISUALISATION_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include "../../../Common/ConstantsEnum.h"
#include "../../../Common/Config/ConfigFileReader.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <mt_msgs/pose.h>

class Visualisation
{
    private:
        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;

        Common::Utils::ConfigFileReader mConfigFileReader;

        // Configurable Parameters
        int mNumberOfUAV;
        double mUAVScaleX;
        double mUAVScaleY;
        double mUAVScaleZ;
        double mGoalScaleX;
        double mGoalScaleY;
        double mGoalScaleZ;
        double mPoseScaleX;
        double mPoseScaleY;
        double mPoseScaleZ;

        tf::TransformListener mTransformListener;
        std::vector<visualization_msgs::Marker> mUAVSetPointPositionMarkerVector;
        std::vector<visualization_msgs::Marker> mUAVSystemPoseMarkerVector;
        visualization_msgs::Marker mGoalPositionMarker;
        visualization_msgs::Marker mHumanSystemPoseMarker;

        // Publisher
        ros::Publisher mSetPointPositionPublisher;
        ros::Publisher mGoalPositionPublisher;
        ros::Publisher mSystemPosePublisher;

        // Subscriber
        std::vector<ros::Subscriber> mUAVSetPointPositionSubscriberVector;
        std::vector<ros::Subscriber> mUAVSystemPoseSubscriberVector;
        ros::Subscriber mGoalPositionSubscriber;
        ros::Subscriber mHumanSystemPoseSubscriber;        

        // Timers
        ros::Timer mModuleLoopTimer;

        // Publisher Functions
        void pubSetPointPosition(const visualization_msgs::Marker aSetPointPosition);
        void pubGoalPosition(const visualization_msgs::Marker aGoal);
        void pubSystemPose(const visualization_msgs::Marker aSystemPose);

        // Subscriber Functions
        void UAVSetPointPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& aUAVSetPointPosition, int i);
        void UAVSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aUAVSystemPose, int i);
        void GoalPositionCallback(const mt_msgs::pose::ConstPtr& aGoal);
        void HumanSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aHumanSystemPose);

        
    public:
        Visualisation(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate);
        virtual ~Visualisation();
}; // Visualisation

#endif // VISUALISATION_H