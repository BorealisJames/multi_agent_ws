#include <ros/ros.h>
#include <iostream>

#include "PhaseSynchronizer/PhaseSynchronizer.h"
#include "DistributedGlobalPathPlannerHandler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distributed_global_path_planner");

    ros::NodeHandle nh("~");

    DistributedGlobalPathPlanner::PhaseSynchronizer ps;

    while(true)
    {
        ps.SyncPhasesOfAgents();
    }

    ros::spin();
    return 0;
}
