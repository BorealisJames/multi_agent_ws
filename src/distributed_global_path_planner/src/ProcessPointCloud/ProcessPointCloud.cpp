//
// Created by benson on 20/8/21.
//

#include "ProcessPointCloud.h"

namespace DistributedGlobalPathPlanner
{
    void
    ProcessPointCloud::AppendPointClouds(const sensor_msgs::PointCloud& cloudInput1, const sensor_msgs::PointCloud& cloudInput2,
                                        sensor_msgs::PointCloud& cloudOutput)
    {
        cloudOutput = cloudInput1;
        cloudOutput.header.frame_id = "map";
        cloudOutput.points.insert(cloudOutput.points.end(), cloudInput2.points.begin(), cloudInput2.points.end());
        cloudOutput.channels.insert(cloudOutput.channels.end(), cloudInput2.channels.begin(), cloudInput2.channels.end());
    }

    void
    ProcessPointCloud::RemovePointsWithinARadiusAndFromGroundFromPointCloud2D(const sensor_msgs::PointCloud& cloudInput,
                                                                                const Eigen::Vector3d& point,
                                                                                const double removalRadius,
                                                                                sensor_msgs::PointCloud& cloudOutput)
    {
        cloudOutput = cloudInput;

        double removalRadiusSquared = removalRadius*removalRadius;

        int i=0;
        while (i!=cloudOutput.points.size())
        {
            double distSquared = (cloudOutput.points.at(i).x-point.x())*(cloudOutput.points.at(i).x-point.x()) +
                                 (cloudOutput.points.at(i).y-point.y())*(cloudOutput.points.at(i).y-point.y());

            if (distSquared<=removalRadiusSquared ||
                cloudOutput.points.at(i).z<=0.75)
            {
                cloudOutput.points.erase(cloudOutput.points.begin() + i);

                if (i<cloudOutput.channels.size())
                {
                    cloudOutput.channels.erase(cloudOutput.channels.begin() + i);
                }
            }
            else
            {
                i++;
            }
        }
    }

    void
    ProcessPointCloud::RemovePointsWithinARadiusPointCloud3D(const sensor_msgs::PointCloud& cloudInput,
                                                                 const Eigen::Vector3d& point,
                                                                 const double removalRadius,
                                                                 sensor_msgs::PointCloud& cloudOutput)
    {
        cloudOutput = cloudInput;

        double removalRadiusSquared = removalRadius*removalRadius;

        int i=0;
        while (i!=cloudOutput.points.size())
        {
            double distSquared = (cloudOutput.points.at(i).x-point.x())*(cloudOutput.points.at(i).x-point.x()) +
                                 (cloudOutput.points.at(i).y-point.y())*(cloudOutput.points.at(i).y-point.y()) +
                                 (cloudOutput.points.at(i).z-point.z())*(cloudOutput.points.at(i).z-point.z());

            if (distSquared<=removalRadiusSquared)
            {
                cloudOutput.points.erase(cloudOutput.points.begin() + i);

                if (i<cloudOutput.channels.size())
                {
                    cloudOutput.channels.erase(cloudOutput.channels.begin() + i);
                }
            }
            else
            {
                i++;
            }
        }
    }

}   // namespace DistributedGlobalPathPlanner