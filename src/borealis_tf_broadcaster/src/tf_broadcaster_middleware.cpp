#include "../include/tf_broadcaster/tf_broadcaster.h"

namespace TF
{

// ROS Publisher Functions

void TfBroadcaster::pubSystemMap2BodyTransform()
{
    tf::poseMsgToTF(mSystemPose.pose, mSystemBody_tfPose);
    mSystemMap2BodyBroadcaster.sendTransform(tf::StampedTransform(mSystemBody_tfPose,
                                                                  mSystemPose.header.stamp,
                                                                  "odom", "uav1UWB"));
}

void TfBroadcaster::pubSystemMap2BodyTransform2()
{
    tf::poseMsgToTF(mSystemPose2.pose, mSystemBody_tfPose2);
    mSystemMap2BodyBroadcaster.sendTransform(tf::StampedTransform(mSystemBody_tfPose2,
                                                                  mSystemPose2.header.stamp,
                                                                  "odom", "uav2UWB"));
}

void TfBroadcaster::pubSystemMap2BodyTransform3()
{
    tf::poseMsgToTF(mSystemPose2.pose, mSystemBody_tfPose2);
    mSystemMap2BodyBroadcaster.sendTransform(tf::StampedTransform(mSystemBody_tfPose3,
                                                                  mSystemPose3.header.stamp,
                                                                  "odom", "uav3UWB"));
}


void TfBroadcaster::systemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mSystemPose = *msg;
    mSystemPose.header.stamp = mSystemPose.header.stamp;
    pubSystemMap2BodyTransform();
}

void TfBroadcaster::systemPoseCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mSystemPose2 = *msg;
    mSystemPose2.header.stamp = mSystemPose.header.stamp;
    pubSystemMap2BodyTransform2();
}

void TfBroadcaster::systemPoseCallback3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mSystemPose3 = *msg;
    mSystemPose3.header.stamp = mSystemPose.header.stamp;
    pubSystemMap2BodyTransform3();
}


}   // TF