#include "../include/tf_broadcaster/tf_broadcaster.h"

namespace TF
{

TfBroadcaster::TfBroadcaster(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate):
    mNh(nh),
    mNhPrivate(nhPrivate)
{
    mSystemPoseSub = mNh->subscribe<geometry_msgs::PoseStamped>("system_pose_topic", 10, &TfBroadcaster::systemPoseCallback, this);
    mSystemPoseSub2 = mNh->subscribe<geometry_msgs::PoseStamped>("system_pose_topic2", 10, &TfBroadcaster::systemPoseCallback2, this);
    mSystemPoseSub3 = mNh->subscribe<geometry_msgs::PoseStamped>("system_pose_topic3", 10, &TfBroadcaster::systemPoseCallback3, this);

}

TfBroadcaster::~TfBroadcaster()
{
    // Destructor
}

}   // TF