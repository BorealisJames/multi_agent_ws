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

/*
    Replace UWB frames with mavros frames
*/

namespace TF
{
class TfBroadcaster
{
    private:
        ros::NodeHandlePtr mNh;
        ros::NodeHandlePtr mNhPrivate;

        std::string mBodyFrame;

        // ROS Subscribers
        ros::Subscriber mSystemPoseSub;

        // Cache of frame link messages
        geometry_msgs::PoseStamped mSystemPose;
        
        // TF Publishers
        tf::TransformBroadcaster mSystemMap2BodyBroadcaster;
        tf::Pose mSystemBody_tfPose;

        // ROS Publisher Functions
        void pubSystemMap2BodyTransform();
        
        // ROS Subscriber Functions
        void systemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


    public:
        TfBroadcaster(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate);
        virtual ~ TfBroadcaster();
};

}   // TF

#endif // TF_BROADCASTER_H
