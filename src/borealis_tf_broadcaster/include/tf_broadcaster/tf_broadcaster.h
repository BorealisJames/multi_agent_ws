#ifndef TF_BROADCASTER_H
#define TF_BROADCASTER_H

// ROS Packages
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// ROS Messages
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

// Common
#include "../../../Common/ConstantsEnum.h"

namespace TF
{
class TfBroadcaster
{
    private:
        ros::NodeHandlePtr mNh;
        ros::NodeHandlePtr mNhPrivate;

        std::string mNamespace;

        // ROS Service Clients

        // Flags
        bool mBroadcast_lidar;

        float mX_offset; // follow px4 convention. +X is foward, +Y is right, +Z is downwards
        float mY_offset;
        float mZ_offset;

        // ROS Subscribers
        ros::Subscriber mSystemOdomSub;
        ros::Subscriber mSystemPoseSub;
        ros::Subscriber mLocalPoseSub;
        ros::Subscriber mLidarSub;  // Only to initialise lidar frame id (to be consistent for transform in other nodes, to consider this node publishing a mapping of frame ids and "Common" names)
        ros::Subscriber mCameraSub;
        ros::Subscriber mLocalOdomSub;

        // ROS Timers 

        // Cache of frame link messages
        geometry_msgs::PoseStamped mSystemOdomPose;
        nav_msgs::Odometry mSystemPose;
        geometry_msgs::PoseStamped mLocalPose;
        
        geometry_msgs::PoseStamped mLocalOdomPose; // quick fix

        // Frame names
        std::string mSystemOdomFrame;
        std::string mBodyFrame;
        std::string mLidarFrame;
        std::string mCameraFrame;
        
        // TF Publishers
        tf::TransformBroadcaster mSystemMap2SystemOdomBroadcaster;
        tf::Pose mSystemOdom_tfPose;

        tf::TransformBroadcaster mSystemMap2BodyBroadcaster;
        tf::Pose mSystemBody_tfPose;

        tf::TransformBroadcaster mSystemFrameToLocalFrameBroadcaster;
        tf::Pose mSystemLocal_tfPose;

        tf::TransformBroadcaster mBody2LocalMapBroadcaster;
        tf::Pose mLocalBody_tfPose;

        tf::TransformBroadcaster mBody2LidarBroadcaster;
        tf::Transform mBody2LidarTransform;
        tf::Quaternion mBody2LidarRotation;

        tf::TransformBroadcaster mBody2CameraBroadcaster;
        tf::Transform mBody2CameraTransform;
        tf::Quaternion mBody2CameraRotation;

        ros::Publisher mSystemPosePub;

        // ROS Publisher Functions
        void pubSystemMap2SystemOdomTransform();
        void pubSystemMap2BodyTransform();
        void pubSystemFrameToLocalFrameTransform();
        void pubBody2LocalMapTransform();
        void pubBody2LidarTransform(const ros::Time& stamp);
        void pubBody2CameraTransform(const ros::Time& stamp);

        void pubSystemPose();

        // ROS Subscriber Functions
        void systemOdomCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        void systemPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void localOdomCallbackBorealis(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void lidarCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
        void cameraCallback(const nav_msgs::Odometry::ConstPtr &msg);

        void lidar2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        

        // ROS Timer Functions

    public:
        TfBroadcaster(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate);
        virtual ~ TfBroadcaster();
};

}   // TF

#endif // TF_BROADCASTER_H

