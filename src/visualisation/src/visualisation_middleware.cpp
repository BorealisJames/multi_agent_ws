#include "../include/visualisation/visualisation.h"

// Publisher Functions
void Visualisation::pubSetPointPosition(const visualization_msgs::Marker aSetPointPosition)
{
    mSetPointPositionPublisher.publish(aSetPointPosition);
}

void Visualisation::pubGoalPosition(const visualization_msgs::Marker aGoalPosition)
{
    mGoalPositionPublisher.publish(aGoalPosition);
}

void Visualisation::pubSystemPose(const visualization_msgs::Marker aSystemPose)
{
    mSystemPosePublisher.publish(aSystemPose);
}

// Subscriber Functions
void Visualisation::UAVSetPointPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& aUAVSetPointPosition, int i)
{
    geometry_msgs::PoseStamped tmp_local;
    geometry_msgs::PoseStamped tmp_system;
    tmp_local = *aUAVSetPointPosition;

    const ros::Time refTime = ros::Time::now();

    std::string systemFrame = Common::Entity::SYSTEM_FRAME;
    std::string localFrame = "iris" + std::to_string(i) + "/LOCAL/map";
    tmp_local.header.frame_id = localFrame;
    tmp_local.header.stamp = refTime;
    tmp_system.header.frame_id = systemFrame;
    tmp_system.header.stamp = refTime;

    if (!mTransformListener.waitForTransform(systemFrame, localFrame, refTime, ros::Duration(0.1)))
    {
        ROS_WARN("Wait for transform timed out, using last available transform instead.");
    }
    mTransformListener.transformPose(systemFrame, tmp_local, tmp_system);

    mUAVSetPointPositionMarkerVector.at(i).pose = tmp_system.pose;
    pubSetPointPosition(mUAVSetPointPositionMarkerVector.at(i));
}

void Visualisation::UAVSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aUAVSystemPose, int i)
{
    geometry_msgs::PoseStamped tmp;
    tmp = *aUAVSystemPose;
    mUAVSystemPoseMarkerVector.at(i).pose = tmp.pose;

    pubSystemPose(mUAVSystemPoseMarkerVector.at(i));
}

void Visualisation::GoalPositionCallback(const mt_msgs::pose::ConstPtr& aGoal)
{
    mt_msgs::pose tmp;
    tmp = *aGoal;
    mGoalPositionMarker.pose.position = tmp.position;
    tf::Quaternion tQuat = tf::createQuaternionFromYaw(tmp.headingRad);
    mGoalPositionMarker.pose.orientation.w = tQuat.getW();
    mGoalPositionMarker.pose.orientation.x = tQuat.getX();
    mGoalPositionMarker.pose.orientation.y = tQuat.getY();
    mGoalPositionMarker.pose.orientation.z = tQuat.getZ();

    // pubGoalPosition(mGoalPositionMarker);
}

void Visualisation::HumanSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aHumanSystemPose)
{
    geometry_msgs::PoseStamped tmp;
    tmp = *aHumanSystemPose;
    mHumanSystemPoseMarker.pose = tmp.pose;

    pubSystemPose(mHumanSystemPoseMarker);
}