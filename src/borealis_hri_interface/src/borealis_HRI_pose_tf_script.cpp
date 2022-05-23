#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "../../Common/ConstantsEnum.h"
#include "../../Common/Config/ConfigFileReader.h"

// quick fix 

int main(int argc, char** argv){

    ros::init(argc, argv, "pseudo_uwb_t265_frame_broadcaster");
    ros::NodeHandle node("");
    ros::NodeHandle nhPrivate("~");
    geometry_msgs::TransformStamped pseudo_tf_to_broadcast;

    // prepare drone info 
    std::string drone_number = std::getenv( "DRONE_NUMBER" );
    std::cout << "drone number is " <<  drone_number << std::endl;
    std::string header_frameid = "uav" + drone_number;
    std::string frameid_to_compare = "uav" + drone_number + "/t265_pose_frame";
    std::string pseudo_child_frameId = "uav" + drone_number + "/pseudo_uwb_to_t265_transform";

    std::cout << "header frame is " << header_frameid << std::endl;
    std::cout << "frame id to compare is " << header_frameid << std::endl;
    std::cout << "pseudo child frame is " << pseudo_child_frameId << std::endl;

    tf2_ros::TransformBroadcaster pseudo_tf_broadcaster;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    pseudo_tf_to_broadcast.header.frame_id = "odom";
    pseudo_tf_to_broadcast.child_frame_id = pseudo_child_frameId;
    ros::Rate rate(20.0);

    /// Publish transform data in a new frame
    while (node.ok()){
        // Get transform data between these 2 frames
        geometry_msgs::TransformStamped tmp_listen_tf;
        try
        {
            tmp_listen_tf = tf_buffer.lookupTransform(frameid_to_compare, header_frameid, ros::Time(0));
            pseudo_tf_to_broadcast.header.stamp = tmp_listen_tf.header.stamp;
            pseudo_tf_to_broadcast.transform = tmp_listen_tf.transform;
            // Send this to the tf tree
            pseudo_tf_broadcaster.sendTransform(pseudo_tf_to_broadcast);

        }
        catch(tf2::TransformException& ex)
        {
        ROS_WARN("%s",ex.what()); 
        ros::Duration(1.0).sleep();
        continue;
        }
        rate.sleep();
    }

};