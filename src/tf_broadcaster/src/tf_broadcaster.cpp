#include "../include/tf_broadcaster/tf_broadcaster.h"

namespace TF
{

TfBroadcaster::TfBroadcaster(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate):
    mNh(nh),
    mNhPrivate(nhPrivate),
    mNamespace(Common::Entity::AgentNamespace(mNh)),
    mBroadcast_lidar(true), // default true
    mBroadcast_camera(true),
    mLidarFrame(""),
    mCameraFrame("")
{

    mtfListener = new tf2_ros::TransformListener(mtfBuffer); // guess and checked until code compiled xd


    mSystemOdomFrame = Common::Entity::AgentLocalOdomFrame(mNh);
    mBodyFrame = Common::Entity::AgentBodyFrame(mNh);
    mSystemOdomPose.header.frame_id = Common::Entity::SYSTEM_FRAME;

    mNhPrivate->param<bool>("broadcast_lidar",mBroadcast_lidar,true);
    mNhPrivate->param<bool>("broadcast_camera",mBroadcast_camera,true);

    mSystemOdomSub = mNh->subscribe<gazebo_msgs::ModelStates>("system_map2local_odom_topic", 10, &TfBroadcaster::systemOdomCallback, this);
    mSystemPoseSub = mNh->subscribe<nav_msgs::Odometry>("system_map2body_topic", 10, &TfBroadcaster::systemPoseCallback, this);
    mLocalPoseSub = mNh->subscribe<geometry_msgs::PoseStamped>("local_map2body_topic", 10, &TfBroadcaster::localPoseCallback, this);

    mPointCloudDebugPublisher = mNh->advertise<sensor_msgs::PointCloud>("pointcloud_debug", 10);
    if (mBroadcast_lidar)
    {
        mLidarSub = mNh->subscribe<sensor_msgs::PointCloud2>("body2lidar_topic", 10, &TfBroadcaster::lidarCallback, this);
    }
    if (mBroadcast_camera)
    {
        mCameraSub = mNh->subscribe<sensor_msgs::PointCloud2>("body2camera_topic", 10, &TfBroadcaster::cameraCallback, this);
    }
    mSystemPosePub = mNh->advertise<geometry_msgs::PoseStamped>("systempose_topic", 10);
}

TfBroadcaster::~TfBroadcaster()
{
    // Destructor
}

}   // TF