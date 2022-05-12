#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// quick fix 

void UWBtfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    tf2_ros::TransformBroadcaster uwbframe;
    geometry_msgs::TransformStamped uwbtransform;
    
    uwbtransform.header.stamp = ros::Time::now();
    uwbtransform.header.frame_id = "odom"; 
    uwbtransform.child_frame_id = "UWB_uav1_body";

    uwbtransform.transform.translation.x = msg->pose.pose.position.x;
    uwbtransform.transform.translation.y = msg->pose.pose.position.y;
    uwbtransform.transform.translation.z = msg->pose.pose.position.z;

    uwbtransform.transform.rotation.w = msg->pose.pose.orientation.w;
    uwbtransform.transform.rotation.x = msg->pose.pose.orientation.x;
    uwbtransform.transform.rotation.y = msg->pose.pose.orientation.y;
    uwbtransform.transform.rotation.z = msg->pose.pose.orientation.z;

    uwbframe.sendTransform(uwbtransform);

}

void UWBtfCallback2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    tf2_ros::TransformBroadcaster uwbframe;
    geometry_msgs::TransformStamped uwbtransform;
    
    uwbtransform.header.stamp = ros::Time::now();
    uwbtransform.header.frame_id = "odom"; 
    uwbtransform.child_frame_id = "UWB_uav2_body";

    uwbtransform.transform.translation.x = msg->pose.pose.position.x;
    uwbtransform.transform.translation.y = msg->pose.pose.position.y;
    uwbtransform.transform.translation.z = msg->pose.pose.position.z;

    uwbtransform.transform.rotation.w = msg->pose.pose.orientation.w;
    uwbtransform.transform.rotation.x = msg->pose.pose.orientation.x;
    uwbtransform.transform.rotation.y = msg->pose.pose.orientation.y;
    uwbtransform.transform.rotation.z = msg->pose.pose.orientation.z;

    uwbframe.sendTransform(uwbtransform);

}

int main(int argc, char** argv){

    ros::init(argc, argv, "pseudo_uwb_t265_frame_broadcaster");
    ros::NodeHandle node("~");

    std::string mHeaderFrameId;
    std::string mPseudoChildFrameId;
    std::string mFrameIdToCompare;

    std::string mHeaderFrameId2;
    std::string mPseudoChildFrameId2;
    std::string mFrameIdToCompare2;

    tf2_ros::TransformBroadcaster mPsuedotf_broadcaster;
    tf2_ros::TransformBroadcaster mPsuedotf_broadcaster_2;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped pseudo_broadcast_tf;
    geometry_msgs::TransformStamped pseudo_broadcast_tf_2;
    
    node.getParam("headerFrameIdUAV1", mHeaderFrameId);
    node.getParam("pseudoChildFrameIdUAV1", mPseudoChildFrameId);
    node.getParam("frameIdToCompareUAV1", mFrameIdToCompare);

    std::cout << "mHeaderFrameId is: " << mHeaderFrameId << std::endl;
    std::cout << "mFrameIdToCompare is: " << mFrameIdToCompare << std::endl;
    std::cout << "mPseudoChildFrameId is: " << mPseudoChildFrameId << std::endl;

    node.getParam("headerFrameIdUAV2", mHeaderFrameId2);
    node.getParam("pseudoChildFrameIdUAV2", mPseudoChildFrameId2);
    node.getParam("frameIdToCompareUAV2", mFrameIdToCompare2);

    std::cout << "mHeaderFrameId2 is: " << mHeaderFrameId2 << std::endl;
    std::cout << "mPseudoChildFrameId2 is: " << mPseudoChildFrameId2 << std::endl;
    std::cout << "mFrameIdToCompare2 is: " << mFrameIdToCompare2 << std::endl;

    ros::Subscriber mUwBsubscriber1 = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("uwb_uav_frame1", 100, UWBtfCallback);
    ros::Subscriber mUwBsubscriber2 = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("uwb_uav_frame2", 100, UWBtfCallback2);

    pseudo_broadcast_tf.header.frame_id = mHeaderFrameId;
    pseudo_broadcast_tf.child_frame_id = mPseudoChildFrameId;

    pseudo_broadcast_tf_2.header.frame_id = mHeaderFrameId2;
    pseudo_broadcast_tf_2.child_frame_id = mPseudoChildFrameId2;

    ros::Rate rate(50.0);
    while (node.ok()){
        
        // UAV1
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
        
        pseudo_broadcast_tf.header.stamp = listen_tf.header.stamp;
        pseudo_broadcast_tf.transform = listen_tf.transform;

        mPsuedotf_broadcaster.sendTransform(pseudo_broadcast_tf);

        // UAV2

        try
        {
        listen_tf = tf_buffer.lookupTransform(mFrameIdToCompare2, mHeaderFrameId2, ros::Time(0));
        }
        catch(tf2::TransformException& ex)
        {
        ROS_WARN("%s",ex.what()); 
        ros::Duration(1.0).sleep();
        continue;
        }
        
        pseudo_broadcast_tf_2.header.stamp = listen_tf.header.stamp;
        pseudo_broadcast_tf_2.transform = listen_tf.transform;

        mPsuedotf_broadcaster_2.sendTransform(pseudo_broadcast_tf_2);

        rate.sleep();
        
    }

};