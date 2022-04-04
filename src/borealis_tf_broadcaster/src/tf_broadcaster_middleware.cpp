#include "../include/tf_broadcaster/tf_broadcaster.h"

namespace TF
{

    // ROS Publisher Functions

    void TfBroadcaster::localOdomCallbackBorealis(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        mLocalOdomPose = *msg; // the pose msg is considered odom (since its a measurement of how much it has moved) in this case...

        geometry_msgs::Pose globalPose;
        geometry_msgs::Pose localPose;

        ros::Time now = ros::Time::now();

        // Find global position
        globalPose.position.x = mLocalOdomPose.pose.position.x + mX_offset - 0.1; // 10 cm offset due to location of t265
        globalPose.position.y = mLocalOdomPose.pose.position.y + mY_offset;
        globalPose.position.z = mLocalOdomPose.pose.position.z + mZ_offset;
        globalPose.orientation = mLocalOdomPose.pose.orientation;

        // Publish global to body
        tf::poseMsgToTF(globalPose, mLocalBody_tfPose);
        // Inverse transform (body -> globalmap)
        mSystemMap2BodyBroadcaster.sendTransform(tf::StampedTransform(mLocalBody_tfPose.inverse(),
                                                                     msg->header.stamp,
                                                                     "/odom", mBodyFrame)); // global map



        // Find local position
        localPose.position.x = mLocalOdomPose.pose.position.x - 0.1; // 10 cm offset due to location of t265
        localPose.position.y = mLocalOdomPose.pose.position.y;
        localPose.position.z = mLocalOdomPose.pose.position.z;
        localPose.orientation = mLocalOdomPose.pose.orientation;

        // Publish body to local
        tf::poseMsgToTF(localPose, mLocalBody_tfPose);
        // Inverse transform (body -> LOCAL/map)
        mBody2LocalMapBroadcaster.sendTransform(tf::StampedTransform(mLocalBody_tfPose,
                                                                     msg->header.stamp,
                                                                     mBodyFrame, Common::Entity::AgentLocalFrame(mNh))); 

        // Pubish body to sensors
        if (mBroadcast_lidar)
        {
            pubBody2LidarTransform(msg->header.stamp);
            pubBody2CameraTransform(msg->header.stamp);
        }

	// Publish system pose

        mSystemOdomPose.header.stamp = ros::Time::now();
        mSystemOdomPose.pose = globalPose;
        // pubSystemMap2SystemOdomTransform();

        mSystemPose.header.stamp = ros::Time::now();
        mSystemPose.header.frame_id = "/odom";
        mSystemPose.child_frame_id = "/odom";
        mSystemPose.pose.pose = mSystemOdomPose.pose;
//	ROS_INFO("Publishing system pose %i", mSystemPose.pose.pose.position.x);
        pubSystemMap2BodyTransform();
        // pubSystemFrameToLocalFrameTransform();
        pubSystemPose();


    }

    void TfBroadcaster::pubBody2LidarTransform(const ros::Time &stamp)
    {
        mBody2LidarTransform.setOrigin(tf::Vector3(0.0, 0.0, 0.1));
        mBody2LidarRotation.setRPY(0.0, 0.0, 0.0);
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

    void TfBroadcaster::pubBody2CameraTransform(const ros::Time &stamp)
    {
        mBody2CameraTransform.setOrigin(tf::Vector3(0.1, 0.0, 0.0));
        mBody2CameraRotation.setRPY(-1.5707, 0, -1.5707);
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

    void TfBroadcaster::pubSystemMap2SystemOdomTransform() // not used
    {
        tf::poseMsgToTF(mSystemOdomPose.pose, mSystemOdom_tfPose);
        mSystemMap2SystemOdomBroadcaster.sendTransform(tf::StampedTransform(mSystemOdom_tfPose,
                                                                            mSystemOdomPose.header.stamp,
                                                                            mSystemOdomPose.header.frame_id, mSystemOdomFrame));
    }

    void TfBroadcaster::pubSystemFrameToLocalFrameTransform() // not being used 2!
    {
        tf::poseMsgToTF(mSystemPose.pose.pose, mSystemLocal_tfPose);
        mSystemFrameToLocalFrameBroadcaster.sendTransform(tf::StampedTransform(mSystemLocal_tfPose, mSystemPose.header.stamp, mSystemPose.header.frame_id, Common::Entity::AgentLocalFrame(mNh)));
    }

    void TfBroadcaster::pubSystemMap2BodyTransform() // Publish Global to Body Frame // not used in borealis
    {
        tf::poseMsgToTF(mSystemPose.pose.pose, mSystemBody_tfPose);
        mSystemMap2BodyBroadcaster.sendTransform(tf::StampedTransform(mSystemBody_tfPose,
                                                                      mSystemPose.header.stamp,
                                                                      mSystemPose.header.frame_id, mBodyFrame));
    }

    void TfBroadcaster::pubBody2LocalMapTransform() // Unused in borealisPublish Body to Local Odom
    {
        tf::poseMsgToTF(mLocalPose.pose, mLocalBody_tfPose);
        // Inverse transform (body -> LOCAL/map)
        mBody2LocalMapBroadcaster.sendTransform(tf::StampedTransform(mLocalBody_tfPose.inverse(),
                                                                     mLocalPose.header.stamp,
                                                                     mBodyFrame, Common::Entity::AgentLocalFrame(mNh)));
    }

    // Unused
    void TfBroadcaster::pubSystemPose()
    {
        mSystemPosePub.publish(mSystemOdomPose);
    }

    // Unused
    void TfBroadcaster::systemOdomCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        // Grab all Models from Gazebo
        const size_t numNames = msg->name.size();
        size_t idx = 0;

        // Hard fix for now as dynamic renaming of gazebo models is not straight forward
        // if (mNamespace == "uav0")
        // {
        //     mNamespace = "iris0"; // gazebo model name
        // }
        // if (mNamespace == "uav1")
        // {
        //     mNamespace = "iris1"; // gazebo model name
        // }

        // From the msg, find the id of the model that has the same namespace
        for (; idx < numNames; idx++)
        {
            if (msg->name[idx] == mNamespace)
            {
                break;
            }
        }

        // ROS_INFO("numNames = %i, idx = %i  ",numNames, idx);
        if (idx <= numNames)
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
        // ROS_INFO("%s",("Publishing map2body transform " + mNamespace).c_str());
    }

    // unused
    void TfBroadcaster::systemPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        mSystemPose = *msg;
        pubSystemMap2BodyTransform();
    }

    // unused
    void TfBroadcaster::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        mLocalPose = *msg;
        // Pub inverse transform (body -> LOCAL/map) as there already exists a map -> body transform (tf does not allow multiple parents)
        pubBody2LocalMapTransform(); // Publish Body to global frame (odom)
                                     // Publish
        if (mBroadcast_lidar)
        {
            pubBody2LidarTransform(msg->header.stamp);
            pubBody2CameraTransform(msg->header.stamp);
        }
    }

    // unused
    void TfBroadcaster::lidarCallback(const sensor_msgs::PointCloud::ConstPtr &msg) // not used
    {

        if (mLidarFrame.empty())
        {
            mLidarFrame = msg->header.frame_id;
            ROS_INFO("%s", ("Lidar frame set: " + mLidarFrame).c_str());
            mLidarSub.shutdown();
        }
    }

    void TfBroadcaster::lidar2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (mLidarFrame.empty())
        {
            mLidarFrame = "ousterlidar";
            ROS_INFO("%s", ("Lidar frame set: " + mLidarFrame).c_str());
            mLidarSub.shutdown();
        }
    }

    void TfBroadcaster::cameraCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (mCameraFrame.empty())
        {
            mCameraFrame = "cameralink";
            ROS_INFO("%s", ("Camera frame set: " + mCameraFrame).c_str());
            mCameraSub.shutdown();
        }
    }

} // TF

