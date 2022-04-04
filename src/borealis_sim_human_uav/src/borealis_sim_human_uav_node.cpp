#include "../include/borealis_sim_human_uav.h"
#include "../include/test.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "borealis_follow_me");
    ros::NodeHandle nh("human");
    ros::NodeHandle nhPrivate("~");

    ROS_INFO("borealis_follow_me node initialized");
    int x  = add(1,2);
    BorealisFollowMe human_follow_me = BorealisFollowMe(nh, nhPrivate);
    
    return 0;
}