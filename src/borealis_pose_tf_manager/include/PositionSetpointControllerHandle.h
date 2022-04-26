#pragma once

#include <math.h>
#include <ros/node_handle.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

class PositionSetpointControllerHandle
{
public:
    PositionSetpointControllerHandle(ros::NodeHandle &nodeHandle);

    void PublishPose();

private:
    template <class Type>
    Type getParam(const std::string &name);

    void LoadParameters();

    void DesiredPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose);

    void AssignedVirtualPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose);

    ros::NodeHandle node_handle_;

    tf::TransformListener mPoseTransformListener;

    std::string sub_assigned_virtual_pose_topic;
    std::string pub_to_sense_avoid_topic;
    std::string source_frame;
    std::string target_frame;

    geometry_msgs::PoseStamped sense_avoid_pose;
        
    double sense_avoid_z;

    bool use_current_pose_;

    ros::Publisher pub_to_sense_avoid;
    ros::Subscriber sub_assigned_virtual_pose;
};

template <class Type>
Type PositionSetpointControllerHandle::getParam(const std::string &name) {
    Type val;
    const bool success = node_handle_.getParam(name, val);

    if (!success) {
        ROS_ERROR_STREAM("PARAMETER NOT FOUND: " << name.c_str());
        ros::shutdown();
    }
    return val;
}
