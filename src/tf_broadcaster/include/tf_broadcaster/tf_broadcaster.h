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

#include "../../../distributed_multi_robot_formation/src/ProcessPointCloud/ProcessPointCloud.h"

// Common
#include "../../../Common/ConstantsEnum.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


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
        bool mBroadcast_camera;
    
        // ROS Subscribers
        ros::Subscriber mSystemOdomSub;
        ros::Subscriber mSystemPoseSub;
        ros::Subscriber mLocalPoseSub;
        ros::Subscriber mLidarSub;  // Only to initialise lidar frame id (to be consistent for transform in other nodes, to consider this node publishing a mapping of frame ids and "Common" names)
        ros::Subscriber mCameraSub;

        // ROS Timers 
        
        // Ros Publishers
        ros::Publisher mPointCloudDebugPublisher;

        // Cache of frame link messages
        geometry_msgs::PoseStamped mSystemOdomPose;
        nav_msgs::Odometry mSystemPose;
        geometry_msgs::PoseStamped mLocalPose;
        sensor_msgs::PointCloud mPointCloudDebug;

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

        tf2_ros::Buffer mtfBuffer;
        tf2_ros::TransformListener* mtfListener; // Some C++ stuff check it out https://answers.ros.org/question/80206/why-cant-initialize-tf2_rostransformlistenter-in-hydro/

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
        void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

        // ROS Timer Functions

    public:
        TfBroadcaster(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate);
        virtual ~ TfBroadcaster();
};

}   // TF

#endif // TF_BROADCASTER_H
