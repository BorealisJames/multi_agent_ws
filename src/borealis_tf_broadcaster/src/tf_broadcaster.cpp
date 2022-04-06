#include "../include/tf_broadcaster/tf_broadcaster.h"

namespace TF
{

TfBroadcaster::TfBroadcaster(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate):
    mNh(nh),
    mNhPrivate(nhPrivate),
    mNamespace(Common::Entity::AgentNamespace(mNh)),
    mBroadcast_lidar(true), // default true
    mLidarFrame(""),
    mCameraFrame("")
{
    mSystemOdomFrame = Common::Entity::AgentLocalOdomFrame(mNh);
    mBodyFrame = Common::Entity::AgentBodyFrame(mNh);
    mSystemOdomPose.header.frame_id = Common::Entity::SYSTEM_FRAME;

    mNhPrivate->param<bool>("broadcast_lidar",mBroadcast_lidar,true);

    // mSystemOdomSub = mNh->subscribe<gazebo_msgs::ModelStates>("system_map2local_odom_topic", 10, &TfBroadcaster::systemOdomCallback, this);
    // mSystemPoseSub = mNh->subscribe<nav_msgs::Odometry>("system_map2body_topic", 10, &TfBroadcaster::systemPoseCallback, this);
    // mLocalPoseSub = mNh->subscribe<nav_msgs::Odometry>("local_map2body_topic", 10, &TfBroadcaster::localOdomCallbackBorealis, this); // Get local odom position

    mLocalOdomSub = mNh->subscribe<geometry_msgs::PoseStamped>("local_odom_topic", 10, &TfBroadcaster::localOdomCallbackBorealis, this); // Get local odom position 

    // Broadcast local to lidar frame
    if (mBroadcast_lidar)
    {
        mLidarSub = mNh->subscribe<sensor_msgs::PointCloud2>("body2lidar_topic", 10, &TfBroadcaster::lidar2Callback, this); 
    }
    mSystemPosePub = mNh->advertise<geometry_msgs::PoseStamped>("systempose_topic", 10);
}

TfBroadcaster::~TfBroadcaster()
{
    // Destructor
}

}   // TF
