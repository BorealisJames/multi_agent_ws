#include <ros/ros.h>
#include <iostream>

#include "PositionSetpointControllerHandle.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_setpoint_controller");

    ros::NodeHandle nh("~");

    PositionSetpointControllerHandle PSC_handle(nh);

    ros::Timer timerPublishPositionSetpoint =
            nh.createTimer(ros::Duration(1.0 / PSC_handle.getRate()),
                           std::bind(&PositionSetpointControllerHandle::PublishPose, &PSC_handle));

    ros::spin();
    return 0;
}
