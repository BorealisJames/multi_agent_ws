#include "PositionSetpointControllerHandle.h"

PositionSetpointControllerHandle::PositionSetpointControllerHandle(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
    , use_current_pose_(true)
    , setpoint_x_(0)
    , setpoint_y_(0)
    , setpoint_z_(0)
    , setquat_x_(0)
    , setquat_y_(0)
    , setquat_z_(0)
    , setquat_w_(1)
{
    LoadParameters();

    // Create subscriber
    sub_desired_setpoint_ = node_handle_.subscribe(sub_desired_setpoint_topic_, 1,
                                                   &PositionSetpointControllerHandle::DesiredPoseCallback, this);
    sub_current_position_ = node_handle_.subscribe(sub_current_position_topic_, 1,
                                                   &PositionSetpointControllerHandle::CurrentPoseCallback, this);

    // Create publisher
    pub_setpoint_position_ = node_handle_.advertise<geometry_msgs::PoseStamped> (pub_setpoint_position_topic_, 1, false);
}

double
PositionSetpointControllerHandle::getRate()
{
    return rate_;
}

void
PositionSetpointControllerHandle::LoadParameters()
{
    ROS_INFO("Loading Topic Names and Flags");

    sub_desired_setpoint_topic_ = getParam<std::string>("sub_desired_setpoint_topic");
    sub_current_position_topic_ = getParam<std::string>("sub_current_position_topic");
    pub_setpoint_position_topic_ = getParam<std::string>("pub_setpoint_position_topic");
    rate_ = getParam<double>("rate");
    setpoint_z_ = getParam<double>("setpoint_z");
}

void
PositionSetpointControllerHandle::PublishPose()
{

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();

    pose.pose.orientation.x = setquat_x_;
    pose.pose.orientation.y = setquat_y_;
    pose.pose.orientation.z = setquat_z_;
    pose.pose.orientation.w = setquat_w_;

    pose.pose.position.x = setpoint_x_;
    pose.pose.position.y = setpoint_y_;
    pose.pose.position.z = setpoint_z_;

    pub_setpoint_position_.publish(pose);
}

void
PositionSetpointControllerHandle::DesiredPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose)
{

    setpoint_x_ = pose->pose.position.x;
    setpoint_y_ = pose->pose.position.y;
    // setpoint_z_ = pose->pose.position.z; Quick fix for gun command pose as gun command pose does not give Z value

    setquat_x_ = pose->pose.orientation.x;
    setquat_y_ = pose->pose.orientation.y;
    setquat_z_ = pose->pose.orientation.z;
    setquat_w_ = pose->pose.orientation.w;

    use_current_pose_ = false;
}

void
PositionSetpointControllerHandle::CurrentPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose)
{
    if (use_current_pose_)
    {
        setpoint_x_ = pose->pose.position.x;
        setpoint_y_ = pose->pose.position.y;
//        setpoint_z_ = pose->pose.position.z;

        setquat_x_ = pose->pose.orientation.x;
        setquat_y_ = pose->pose.orientation.y;
        setquat_z_ = pose->pose.orientation.z;
        setquat_w_ = pose->pose.orientation.w;
    }

    use_current_pose_ = false;
}
