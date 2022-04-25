#include "PositionSetpointControllerHandle.h"

PositionSetpointControllerHandle::PositionSetpointControllerHandle(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
    , use_current_pose_(true)
    , sense_avoid_z(0)
{
    LoadParameters();

    // Create subscriber
    sub_assigned_virtual_pose = node_handle_.subscribe(sub_assigned_virtual_pose_topic, 1,
                                                   &PositionSetpointControllerHandle::AssignedVirtualPoseCallback, this);

    // Create publisher
    pub_to_sense_avoid = node_handle_.advertise<geometry_msgs::PoseStamped> (pub_to_sense_avoid_topic, 1, false);
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

    sub_assigned_virtual_pose_topic = getParam<std::string>("sub_assigned_virtual_position_topic");
    pub_to_sense_avoid_topic = getParam<std::string>("pub_to_sense_avoid_topic");
    source_frame = getParam<std::string>("source_frame");
    target_frame = getParam<std::string>("target_frame");
    rate_ = getParam<double>("rate");
    sense_avoid_z = getParam<double>("setpoint_z");
}

void
PositionSetpointControllerHandle::PublishPose()
{
    sense_avoid_pose.header.stamp = ros::Time::now();
    sense_avoid_pose.pose.position.z = sense_avoid_z;
    pub_to_sense_avoid.publish(sense_avoid_pose);
}

void
PositionSetpointControllerHandle::AssignedVirtualPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose)
{
    if (use_current_pose_)
    {
    geometry_msgs::PoseStamped tmp;
    tmp.pose = pose->pose;
    tmp.header = pose->header;

    if(!mPoseTransformListener.waitForTransform(target_frame, source_frame, tmp.header.stamp ,ros::Duration(0.3)))
    {
        std::string str_tmp;
        ROS_WARN("Wait for transform timed out, using last available transform instead.");
    }
    
    mPoseTransformListener.transformPose(target_frame, tmp, sense_avoid_pose);
    }

    use_current_pose_ = false;
}
