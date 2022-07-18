#include "../include/borealis_sim_human_uav.h"
#include "../include/test.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "borealis_follow_me");
    ros::NodeHandle nh("");
    ros::NodeHandle nhPrivate("~");

    ROS_INFO("Borealis Sim node initialized");
    BorealisFollowMe human_follow_me = BorealisFollowMe(nh, nhPrivate);
    
    return 0;
}