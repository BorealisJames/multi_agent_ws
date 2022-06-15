//
// Created by benson on 20/8/21.
//

#pragma once

#include <sensor_msgs/PointCloud.h>

#include "../Common/Common.h"

namespace DistributedGlobalPathPlanner
{

class ProcessPointCloud
{
public:

    void AppendPointClouds(const sensor_msgs::PointCloud& cloudInput1, const sensor_msgs::PointCloud& cloudInput2,
                           sensor_msgs::PointCloud& cloudOutput);

    void RemovePointsWithinARadiusAndFromGroundFromPointCloud2D(const sensor_msgs::PointCloud& cloudInput,
                                                                const Eigen::Vector3d& point,
                                                                const double removalRadius,
                                                                sensor_msgs::PointCloud& cloudOutput);

    void RemovePointsWithinARadiusPointCloud3D(const sensor_msgs::PointCloud& cloudInput,
                                               const Eigen::Vector3d& point,
                                               const double removalRadius,
                                               sensor_msgs::PointCloud& cloudOutput);

private:

};

}   // namespace DistributedGlobalPathPlanner
