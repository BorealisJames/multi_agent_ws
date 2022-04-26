#include <ros/ros.h>
#include <iostream>

#include "PositionSetpointControllerHandle.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_setpoint_controller");

    ros::NodeHandle nh("~");

    PositionSetpointControllerHandle PSC_handle(nh);

    ros::spin();
    return 0;
}
