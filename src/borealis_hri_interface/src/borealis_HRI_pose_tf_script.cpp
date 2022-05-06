#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "pseudo_lidar_frame_broadcaster");
    ros::NodeHandle node("~");

    std::string mHeaderFrameId;
    std::string mPseudoChildFrameId;
    std::string mFrameIdToCompare;
    int mSourceSegmentId;
    
    node.getParam("headerFrameId", mHeaderFrameId);
    node.getParam("pseudoChildFrameId", mPseudoChildFrameId);
    node.getParam("frameIdToCompare", mFrameIdToCompare);
    node.getParam("sourceSegmentId", mSourceSegmentId);

    std::cout << "mHeaderFrameId is: " << mHeaderFrameId << std::endl;
    std::cout << "mFrameIdToCompare is: " << mFrameIdToCompare << std::endl;
    std::cout << "mPseudoChildFrameId is: " << mPseudoChildFrameId << std::endl;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped broadcast_tf;

    broadcast_tf.header.frame_id = mHeaderFrameId;
    broadcast_tf.child_frame_id = mPseudoChildFrameId;

    ros::Rate rate(50.0);
    while (node.ok()){

        geometry_msgs::TransformStamped listen_tf;
    
        try
        {
        listen_tf = tf_buffer.lookupTransform(mFrameIdToCompare, mHeaderFrameId, ros::Time(0));
        }
        catch(tf2::TransformException& ex)
        {
        ROS_WARN("%s",ex.what()); 
        ros::Duration(1.0).sleep();
        continue;
        }
        
        broadcast_tf.header.stamp = listen_tf.header.stamp;
        broadcast_tf.transform = listen_tf.transform;

        tf_broadcaster.sendTransform(broadcast_tf);

        rate.sleep();
        
    }

};