#include "../include/borealis_HRI_interface.h"
#include <tf/transform_listener.h>

// geometry_msgs::PoseStamped subtractPoseStamped(geometry_msgs::PoseStamped previous, geometry_msgs::PoseStamped current)
// {
//     geometry_msgs::PoseStamped vector_diff;
//     tf2::Quaternion q1_inv;
//     tf2::Quaternion q2;
//     tf2::Quaternion qr;


//     vector_diff.pose.position.x = current.pose.position.x - previous.pose.position.x;
//     vector_diff.pose.position.y = current.pose.position.y - previous.pose.position.y;
//     vector_diff.pose.position.z = current.pose.position.z - previous.pose.position.z;

//     q1_inv.setX(previous.pose.orientation.x);
//     q1_inv.setY(previous.pose.orientation.y);
//     q1_inv.setZ(previous.pose.orientation.z);
//     q1_inv.setW(-previous.pose.orientation.w); // Negative to invert it

//     q2.setX(current.pose.orientation.x);
//     q2.setY(current.pose.orientation.y);
//     q2.setZ(current.pose.orientation.z);
//     q2.setW(current.pose.orientation.w); 

//     qr = q2 * q1_inv;

//     vector_diff.pose.orientation.x = qr.getX();
//     vector_diff.pose.orientation.y = qr.getY();
//     vector_diff.pose.orientation.z = qr.getZ();
//     vector_diff.pose.orientation.w = qr.getW();

//     return vector_diff;

// }

// geometry_msgs::PoseStamped addPoseStamped(geometry_msgs::PoseStamped vector_pose, geometry_msgs::PoseStamped current)
// {

//     geometry_msgs::PoseStamped new_pose_stamped;
//     tf2::Quaternion q1_rot;
//     tf2::Quaternion q2_origin;
//     tf2::Quaternion q_new;

//     new_pose_stamped.pose.position.x = vector_pose.pose.position.x + current.pose.position.x;
//     new_pose_stamped.pose.position.y = vector_pose.pose.position.y + current.pose.position.y;
//     new_pose_stamped.pose.position.z = vector_pose.pose.position.z + current.pose.position.z;

//     q2_origin.setX(current.pose.orientation.x);
//     q2_origin.setY(current.pose.orientation.y);
//     q2_origin.setZ(current.pose.orientation.z);
//     q2_origin.setW(current.pose.orientation.w);

//     q1_rot.setX(vector_pose.pose.orientation.x);
//     q1_rot.setY(vector_pose.pose.orientation.y);
//     q1_rot.setZ(vector_pose.pose.orientation.z);
//     q1_rot.setW(vector_pose.pose.orientation.w);

//     q_new = q1_rot * q2_origin;

//     new_pose_stamped.pose.orientation.x = q_new.getX();
//     new_pose_stamped.pose.orientation.y = q_new.getY();
//     new_pose_stamped.pose.orientation.z = q_new.getZ();
//     new_pose_stamped.pose.orientation.w = q_new.getW();

//     return new_pose_stamped;
// }



int main (int argc, char** argv)
{
    ros::init(argc, argv, "borealis_follow_me");
    ros::NodeHandle nh("");
    ros::NodeHandle nhPrivate("~");

    ROS_INFO("borealis_HRI_interface node initialized");

    // geometry_msgs::PoseStamped t265Pose;
    // geometry_msgs::PoseStamped goal_t265Pose;
    // geometry_msgs::PoseStamped goal_uwbPose;
    // geometry_msgs::PoseStamped uwb_pose;
    // geometry_msgs::PoseStamped vector_diff;

    // tf2::Quaternion quat1;

    // // t265
    // t265Pose.pose.position.x = 0.1;
    // t265Pose.pose.position.y = 0.1;
    // t265Pose.pose.position.z = 0.1;
    
    // // 90 degrees
    // quat1.setEuler(0, 1.57079, 0);
    // t265Pose.pose.orientation.x = quat1.getX();
    // t265Pose.pose.orientation.y = quat1.getY();
    // t265Pose.pose.orientation.z = quat1.getZ();
    // t265Pose.pose.orientation.w = quat1.getW();

    // std::cout << "t265 pose w is " << t265Pose.pose.orientation.w << std::endl;
    // std::cout << "t265 pose x is " << t265Pose.pose.orientation.x << std::endl;
    // std::cout << "t265 pose y is " << t265Pose.pose.orientation.y << std::endl;
    // std::cout << "t265 pose z is " << t265Pose.pose.orientation.z << std::endl;

    // // uwb
    // uwb_pose.pose.position.x = 0.2;
    // uwb_pose.pose.position.y = 0.2;
    // uwb_pose.pose.position.z = 0.2;
    
    // // 90 degrees
    // quat1.setEuler(0, 0, 0);
    // uwb_pose.pose.orientation.x = quat1.getX();
    // uwb_pose.pose.orientation.y = quat1.getY();
    // uwb_pose.pose.orientation.z = quat1.getZ();
    // uwb_pose.pose.orientation.w = quat1.getW();

    // // uwb_goal
    // goal_uwbPose.pose.position.x = 10;
    // goal_uwbPose.pose.position.y = 10;
    // goal_uwbPose.pose.position.z = 10;
    
    // // 90 degrees
    // quat1.setEuler(0, -1.57079, 0);
    // goal_uwbPose.pose.orientation.x = quat1.getX();
    // goal_uwbPose.pose.orientation.y = quat1.getY();
    // goal_uwbPose.pose.orientation.z = quat1.getZ();
    // goal_uwbPose.pose.orientation.w = quat1.getW();

    // vector_diff = subtractPoseStamped(uwb_pose, goal_uwbPose);
    // goal_t265Pose = addPoseStamped(vector_diff, t265Pose);


    // std::cout << "The goal x is " << goal_t265Pose.pose.position.x << std::endl;
    // std::cout << "The goal y is " << goal_t265Pose.pose.position.y << std::endl;
    // std::cout << "The goal x is " << goal_t265Pose.pose.position.z << std::endl;

    // std::cout << "The vector_diff orientation x is " << vector_diff.pose.orientation.x << std::endl;
    // std::cout << "The vector_diff  orientation y is " << vector_diff.pose.orientation.y << std::endl;
    // std::cout << "The vector_diff orientation z is " << vector_diff.pose.orientation.z << std::endl;
    // std::cout << "The vector_diff  orientation w is " << vector_diff.pose.orientation.w << std::endl;

    // std::cout << "The goal orientation x is " << goal_t265Pose.pose.orientation.x << std::endl;
    // std::cout << "The goal  orientation y is " << goal_t265Pose.pose.orientation.y << std::endl;
    // std::cout << "The goal orientation z is " << goal_t265Pose.pose.orientation.z << std::endl;
    // std::cout << "The goal  orientation w is " << goal_t265Pose.pose.orientation.w << std::endl;

    BorealisHRIInterface human_follow_me = BorealisHRIInterface(nh, nhPrivate);

    ros::spin();
    
    return 0;
}

