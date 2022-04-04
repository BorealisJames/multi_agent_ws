#pragma once

#include <math.h>
#include <ros/node_handle.h>
#include <geometry_msgs/PoseStamped.h>

class PositionSetpointControllerHandle
{
public:
    PositionSetpointControllerHandle(ros::NodeHandle &nodeHandle);

    double getRate();

    void PublishPose();

private:
    template <class Type>
    Type getParam(const std::string &name);

    void LoadParameters();

    void DesiredPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose);

    void CurrentPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose);

    ros::NodeHandle node_handle_;

    std::string sub_desired_setpoint_topic_;
    std::string sub_current_position_topic_;
    std::string pub_setpoint_position_topic_;
    double rate_;
    double setpoint_x_;
    double setpoint_y_;
    double setpoint_z_;
    double setquat_x_;
    double setquat_y_;
    double setquat_z_;
    double setquat_w_;
    bool use_current_pose_;

    ros::Publisher pub_setpoint_position_;
    ros::Subscriber sub_desired_setpoint_;
    ros::Subscriber sub_current_position_;
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
