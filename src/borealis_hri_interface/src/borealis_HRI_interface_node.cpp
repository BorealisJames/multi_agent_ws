#include "../include/borealis_HRI_interface.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "borealis_follow_me");
    ros::NodeHandle nh("");
    ros::NodeHandle nhPrivate("~");

    ROS_INFO("borealis_HRI_interface node initialized");
    BorealisHRIInterface human_follow_me = BorealisHRIInterface(nh, nhPrivate);

    ros::spin();
    
    return 0;
}