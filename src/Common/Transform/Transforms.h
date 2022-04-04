#ifndef TRANSFORMS_H
#define TRANSFORMS_H

// ROS Packages
#include <ros/ros.h>
#include <ros/datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
// ROS Messages
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen_conversions/eigen_msg.h>
// C++ Packages
#include <Eigen/Dense>
// Common
#include "../ConstantsMath.h"

namespace Common
{
    namespace Utils
    {
        static bool tfListenerLookupTransform(const tf::TransformListener& tfListener,
                                              const std::string& fromFrame, const std::string& toFrame,
                                              tf::StampedTransform& tfTransform)
        {
            bool success = true;
            try
            {
                tfListener.lookupTransform(toFrame, fromFrame,
                                           ros::Time(0),   // Time of latest available transform
                                           tfTransform);
            }
            catch (tf::TransformException ex)
            {
                success = false;
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            return success;
        }

        static geometry_msgs::PoseStamped transformPose(const tf::TransformListener& tfListener,
                                                        const geometry_msgs::Pose& fromPose,
                                                        const ros::Time& fromStamp,
                                                        const std::string& fromFrame, const std::string& toFrame)
        {
            // Convert to PoseStamped
            std_msgs::Header fromHeader;
            fromHeader.stamp = fromStamp;
            fromHeader.frame_id = fromFrame;
            geometry_msgs::PoseStamped fromPoseStamped;
            fromPoseStamped.header = fromHeader;
            fromPoseStamped.pose = fromPose;
            // Transform
            geometry_msgs::PoseStamped toPoseStamped;
            tfListener.transformPose(toFrame,fromPoseStamped,toPoseStamped);

            return toPoseStamped;
        }

        static geometry_msgs::PoseStamped transformPose(const tf::TransformListener& tfListener,
                                                        const Eigen::Vector3f& fromPosition, const Eigen::Quaternionf& fromOrientation,
                                                        const ros::Time& fromStamp,
                                                        const std::string& fromFrame, const std::string& toFrame)
        {
            geometry_msgs::Pose fromPose;
            fromPose.position.x = static_cast<double>(fromPosition.x());
            fromPose.position.y = static_cast<double>(fromPosition.y());
            fromPose.position.z = static_cast<double>(fromPosition.z());
            fromPose.orientation.x = static_cast<double>(fromOrientation.x());
            fromPose.orientation.y = static_cast<double>(fromOrientation.y());
            fromPose.orientation.z = static_cast<double>(fromOrientation.z());
            fromPose.orientation.w = static_cast<double>(fromOrientation.w());

            return transformPose(tfListener,fromPose,fromStamp,fromFrame,toFrame);
        }

        static geometry_msgs::PoseStamped transformPose(const tf::TransformListener& tfListener,
                                                        const Eigen::Vector3f& fromPosition, const Eigen::Quaternionf& fromOrientation,
                                                        const std::string& fromFrame, const std::string& toFrame)
        {
            const ros::Time fromStamp = ros::Time(0);   // Lastest available transform
            return transformPose(tfListener,fromPosition,fromOrientation,fromStamp,fromFrame,toFrame);   
        }

        static bool transformPointCloud(const tf::TransformListener& tfListener,
                                        const pcl::PointCloud<pcl::PointXYZI>& inCloud,
                                        const std::string& toFrame,
                                        pcl::PointCloud<pcl::PointXYZI>& outCloud)
        {
            tf::StampedTransform tfTransform;
            const bool success = tfListenerLookupTransform(tfListener,inCloud.header.frame_id,toFrame,tfTransform);

            if (success)
            {
                pcl_ros::transformPointCloud(inCloud,outCloud,tfTransform);
            }

            return success;
        }

        static bool transformLaserScan(const tf::TransformListener& tfListener,
                                       const sensor_msgs::LaserScan& inLaserScan,
                                       const std::string& toFrame,
                                       sensor_msgs::LaserScan& outLaserScan)
        {
            tf::StampedTransform tfTransform;
            const bool success = tfListenerLookupTransform(tfListener,inLaserScan.header.frame_id,toFrame,tfTransform);

            if (success)
            {
                //pcl_ros::transformPointCloud(inCloud,outCloud,tfTransform);
            }

            return success; 
        }
    }   // Utils
}   // Common

#endif // TRANSFORMS_H