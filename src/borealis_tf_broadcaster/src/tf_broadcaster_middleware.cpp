#include "../include/tf_broadcaster/tf_broadcaster.h"

namespace TF
{

// ROS Publisher Functions

void TfBroadcaster::pubSystemMap2BodyTransform()
{
    tf::poseMsgToTF(mSystemPose.pose, mSystemBody_tfPose);
    mSystemMap2BodyBroadcaster.sendTransform(tf::StampedTransform(mSystemBody_tfPose,
                                                                  mSystemPose.header.stamp,
                                                                  "odom", mBodyFrame));
}

void TfBroadcaster::systemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mSystemPose = *msg;
    mSystemPose.header.stamp = mSystemPose.header.stamp;
    pubSystemMap2BodyTransform();
}

}   // TF