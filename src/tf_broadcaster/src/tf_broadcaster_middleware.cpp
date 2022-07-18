#include "../include/tf_broadcaster/tf_broadcaster.h"

namespace TF
{

// ROS Publisher Functions
void TfBroadcaster::pubSystemMap2SystemOdomTransform()
{
    tf::poseMsgToTF(mSystemOdomPose.pose,mSystemOdom_tfPose);
    mSystemMap2SystemOdomBroadcaster.sendTransform(tf::StampedTransform(mSystemOdom_tfPose,
                                                                        mSystemOdomPose.header.stamp,
                                                                        mSystemOdomPose.header.frame_id, mSystemOdomFrame));
}

void TfBroadcaster::pubSystemMap2BodyTransform()
{
    tf::poseMsgToTF(mSystemPose.pose.pose,mSystemBody_tfPose);
    mSystemMap2BodyBroadcaster.sendTransform(tf::StampedTransform(mSystemBody_tfPose,
                                                                  mSystemPose.header.stamp,
                                                                  mSystemPose.header.frame_id, mBodyFrame));
}

void TfBroadcaster::pubSystemFrameToLocalFrameTransform()
{
    tf::poseMsgToTF(mSystemPose.pose.pose, mSystemLocal_tfPose);
    mSystemFrameToLocalFrameBroadcaster.sendTransform(tf::StampedTransform(mSystemLocal_tfPose, mSystemPose.header.stamp, mSystemPose.header.frame_id, Common::Entity::AgentLocalFrame(mNh)));
}

void TfBroadcaster::pubBody2LocalMapTransform()
{
    tf::poseMsgToTF(mLocalPose.pose,mLocalBody_tfPose);
    // Inverse transform (body -> LOCAL/map)
    mBody2LocalMapBroadcaster.sendTransform(tf::StampedTransform(mLocalBody_tfPose.inverse(),
                                                                 mLocalPose.header.stamp,
                                                                 mBodyFrame, Common::Entity::AgentLocalFrame(mNh)));
}

void TfBroadcaster::pubBody2LidarTransform(const ros::Time& stamp)
{
    mBody2LidarTransform.setOrigin(tf::Vector3(0.0,0.0,0.1));
    mBody2LidarRotation.setRPY(0.0,0.0,0.0);
    mBody2LidarTransform.setRotation(mBody2LidarRotation);
    if (mLidarFrame.empty())
    {
        ROS_INFO("Waiting for Lidar message...");
    }
    else
    {
        mBody2LidarBroadcaster.sendTransform(tf::StampedTransform(mBody2LidarTransform, stamp, mBodyFrame, mLidarFrame));
    }
}

void TfBroadcaster::pubBody2CameraTransform(const ros::Time& stamp)
{
    mBody2CameraTransform.setOrigin(tf::Vector3(0.1,0.0,0.0));
    mBody2CameraRotation.setRPY(-1.5707,0,-1.5707);
    mBody2CameraTransform.setRotation(mBody2CameraRotation);
    if (mCameraFrame.empty())
    {
        ROS_INFO("Waiting for Camera message...");
    }
    else
    {
        mBody2CameraBroadcaster.sendTransform(tf::StampedTransform(mBody2CameraTransform, stamp, mBodyFrame, mCameraFrame));
    }
}

void TfBroadcaster::pubSystemPose()
{
    mSystemPosePub.publish(mSystemOdomPose);
}

// ROS Subscriber Functions
void TfBroadcaster::systemOdomCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    const size_t numNames = msg->name.size();
    size_t idx = 0;
    for (; idx < numNames; idx++)
    {
        if (msg->name[idx] == mNamespace)
        {
            break;
        }
    }
    if (idx < numNames)
    {
        mSystemOdomPose.header.stamp = ros::Time::now();
        mSystemOdomPose.pose = msg->pose[idx];
        // pubSystemMap2SystemOdomTransform();

        mSystemPose.header.stamp = ros::Time::now();
        mSystemPose.header.frame_id = "/map";
        mSystemPose.child_frame_id = "/map";
        mSystemPose.pose.pose = mSystemOdomPose.pose;
        pubSystemMap2BodyTransform();
        // pubSystemFrameToLocalFrameTransform();
        pubSystemPose();
    }
}

void TfBroadcaster::systemPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    mSystemPose = *msg;
    mSystemPose.header.stamp = mSystemPose.header.stamp;
    pubSystemMap2BodyTransform();
}

void TfBroadcaster::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mLocalPose = *msg;
    // Pub inverse transform (body -> LOCAL/map) as there already exists a map -> body transform (tf does not allow multiple parents) 
    pubBody2LocalMapTransform();
    if (mBroadcast_lidar)
    {
        pubBody2LidarTransform(msg->header.stamp);
    }
    if (mBroadcast_camera)
    {
        pubBody2CameraTransform(msg->header.stamp);
    }
}

void TfBroadcaster::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (mLidarFrame.empty())
    {
        mLidarFrame = msg->header.frame_id;
        ROS_INFO("%s",("Lidar frame set: " + mLidarFrame).c_str());
    }
    else
    {
        std::string sourceFrame = mLidarFrame;
        DistributedFormation::ProcessPointCloud tmpProcessPointCloud;

        sensor_msgs::PointCloud2 tmp;
        sensor_msgs::PointCloud2 tmpTransformed;
        sensor_msgs::PointCloud tmpToPublish;

        tmpProcessPointCloud.ApplyVoxelFilterToPCL2(*msg, tmp);
        geometry_msgs::TransformStamped transformStamped;

        try
        {
            transformStamped = mtfBuffer.lookupTransform("odom", sourceFrame,ros::Time(0));
            ROS_INFO("Transform stamped: ");
            std::cout << transformStamped << std::endl;
            tf2::doTransform(tmp, tmpTransformed, transformStamped);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.5).sleep();
        }
        catch (std::length_error e )
        {
            ROS_WARN("End of tf buffer reached! There is more than 10s time desync somewhere in the tree from lidar frame to odom");
            ROS_WARN("%s", e.what());
            ros::Duration(0.5).sleep(); 
            // handle custom exception
        }

        sensor_msgs::convertPointCloud2ToPointCloud(tmpTransformed, tmpToPublish);

        mPointCloudDebugPublisher.publish(tmpToPublish);
    }

}

void TfBroadcaster::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (mCameraFrame.empty())
    {
        mCameraFrame = msg->header.frame_id;
        ROS_INFO("%s",("Camera frame set: " + mCameraFrame).c_str());
        mCameraSub.shutdown();
    }
}

}   // TF