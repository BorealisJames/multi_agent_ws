#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// quick fix lmao

void UWBtfCallback(geometry_msgs::PoseWithCovarianceStamped const &msg) {
    tf2_ros::TransformBroadcaster muwbframe;
    geometry_msgs::TransformStamped uwbtransform;
    
    uwbtransform.header.stamp = ros::Time::now();
    uwbtransform.header.frame_id = "odom"; 
    uwbtransform.child_frame_id = "UWB_uav1_body";
    uwbtransform.transform.rotation = msg.pose.pose.orientation; 
    uwbtransform.transform.translation = msg.pose.pose.position; 

    muwbframe.sendTransform(uwbtransform);

}

void UWBtfCallback2(geometry_msgs::PoseWithCovarianceStamped const &msg) {
    tf2_ros::TransformBroadcaster muwbframe;
    geometry_msgs::TransformStamped uwbtransform;
    
    uwbtransform.header.stamp = ros::Time::now();
    uwbtransform.header.frame_id = "odom"; 
    uwbtransform.child_frame_id = "UWB_uav2_body";
    uwbtransform.transform.rotation = msg.pose.pose.orientation; 
    uwbtransform.transform.translation = msg.pose.pose.position; 

    muwbframe.sendTransform(uwbtransform);

}

int main(int argc, char** argv){

    ros::init(argc, argv, "pseudo_lidar_frame_broadcaster");
    ros::NodeHandle node("~");

    std::string mHeaderFrameId;
    std::string mPseudoChildFrameId;
    std::string mFrameIdToCompare;

    ros::Subscriber mUwBsubscriber1;
    ros::Subscriber mUwBsubscriber2;

    int mSourceSegmentId;
    
    node.getParam("headerFrameId", mHeaderFrameId);
    node.getParam("pseudoChildFrameId", mPseudoChildFrameId);
    node.getParam("frameIdToCompare", mFrameIdToCompare);
    node.getParam("sourceSegmentId", mSourceSegmentId);

    std::cout << "mHeaderFrameId is: " << mHeaderFrameId << std::endl;
    std::cout << "mFrameIdToCompare is: " << mFrameIdToCompare << std::endl;
    std::cout << "mPseudoChildFrameId is: " << mPseudoChildFrameId << std::endl;

    mUwBsubscriber = node.subscribe("uwb_uav_frame1", geometry_msgs::PoseWithCovarianceStamped, UWBtfCallback);
    mUwBsubscriber2 = node.subscribe("uwb_uav_frame2", geometry_msgs::PoseWithCovarianceStamped, UWBtfCallback2);

    tf2_ros::TransformBroadcaster mPsuedotf_broadcaster;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped pseudo_broadcast_tf;
    geometry_msgs::TransformStamped uwb_broadcast_tf;

    pseudo_broadcast_tf.header.frame_id = mHeaderFrameId;
    pseudo_broadcast_tf.child_frame_id = mPseudoChildFrameId;

    ros::Rate rate(50.0);
    while (node.ok()){

        geometry_msgs::TransformStamped listen_tf;
    
        try
        {
        listen_tf = tf_buffer.lookupTransform(mFrameIdToCompare, mHeaderFrameId, ros::Time(0.1));
        }
        catch(tf2::TransformException& ex)
        {
        ROS_WARN("%s",ex.what()); 
        ros::Duration(1.0).sleep();
        continue;
        }
        
        pseudo_broadcast_tf.header.stamp = listen_tf.header.stamp;
        pseudo_broadcast_tf.transform = listen_tf.transform;

        mPsuedotf_broadcaster.sendTransform(pseudo_broadcast_tf);

        rate.sleep();
        
    }

};