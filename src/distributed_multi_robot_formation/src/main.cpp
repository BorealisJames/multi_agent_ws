#include <ros/ros.h>
#include <iostream>

#include "DistributedMultiRobotFormation.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distributed_multi_robot_formation");

    ros::NodeHandle nh("~");

    //init DistributedMultiRobotFormation

    ros::spin();
    return 0;
}
