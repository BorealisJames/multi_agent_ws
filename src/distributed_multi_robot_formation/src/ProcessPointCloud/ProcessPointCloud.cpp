//
// Created by benson on 20/8/21.
//

#include "ProcessPointCloud.h"

namespace DistributedFormation
{

    bool 
    ProcessPointCloud::ApplyVoxelFilterAndConvertToPointCloud(const sensor_msgs::PointCloud2& pointCloud2input, sensor_msgs::PointCloud& pointCloud1output)
    {
        bool success = false;

        // Adapted from https://dabit-industries.github.io/turtlebot2-tutorials/13-ROSPCL.html for the voxel filter

        // Container for original & filtered data
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2 cloud_filtered;

        // Convert to PCL data type from ROS sensor_msgs::PointCloud2
        pcl_conversions::toPCL(pointCloud2input, *cloud);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (m_leaf_size_x, m_leaf_size_y, m_leaf_size_z);
        sor.filter (cloud_filtered);

        // Convert back to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl_conversions::moveFromPCL(cloud_filtered, output);

        if (sensor_msgs::convertPointCloud2ToPointCloud(pointCloud2input, pointCloud1output))
        {
            success = true;
        }
        else
        {
            success = false;
        }
        return success;
    }

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
                                                                                  const Common::Position& point,
                                                                                  const double removalRadius,
                                                                                  sensor_msgs::PointCloud& cloudOutput)
    {
        cloudOutput = cloudInput;

        double removalRadiusSquared = removalRadius*removalRadius;

        int i=0;
        while (i!=cloudOutput.points.size())
        {
            double distSquared = (cloudOutput.points.at(i).x-point.x)*(cloudOutput.points.at(i).x-point.x) +
                                    (cloudOutput.points.at(i).y-point.y)*(cloudOutput.points.at(i).y-point.y);

            if (distSquared<=removalRadiusSquared ||
                cloudOutput.points.at(i).z<=0.25)
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
                                                                 const Common::Position& point,
                                                                 const double removalRadius,
                                                                 sensor_msgs::PointCloud& cloudOutput)
    {
        cloudOutput = cloudInput;

        double removalRadiusSquared = removalRadius*removalRadius;

        int i=0;
        while (i!=cloudOutput.points.size())
        {
            double distSquared = (cloudOutput.points.at(i).x-point.x)*(cloudOutput.points.at(i).x-point.x) +
                                 (cloudOutput.points.at(i).y-point.y)*(cloudOutput.points.at(i).y-point.y) +
                                 (cloudOutput.points.at(i).z-point.z)*(cloudOutput.points.at(i).z-point.z);

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

    void
    ProcessPointCloud::RemovePointsFromPointCloudWithinABoundingBox2D(const sensor_msgs::PointCloud& cloudInput,
                                                                          const Common::Position& point,
                                                                          const double boundingBoxX,
                                                                          const double boundingBoxY,
                                                                          sensor_msgs::PointCloud& cloudOutput)
    {
        cloudOutput = cloudInput;

        int i=0;
        while (i!=cloudOutput.points.size())
        {
            bool removePoint = false;

            double deltaXAbs = std::abs(cloudOutput.points.at(i).x-point.x);
            double deltaYAbs = std::abs(cloudOutput.points.at(i).y-point.y);

            if (deltaXAbs<=boundingBoxX && deltaYAbs<=boundingBoxY)
            {
                removePoint = true;
            }

            if (removePoint)
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
    ProcessPointCloud::RemovePointsFromPointCloudWithinABoundingBox3D(const sensor_msgs::PointCloud& cloudInput,
                                                                        const Common::Position& point,
                                                                        const double boundingBoxX,
                                                                        const double boundingBoxY,
                                                                        const double boundingBoxZ,
                                                                        sensor_msgs::PointCloud& cloudOutput)
    {
        cloudOutput = cloudInput;

        int i=0;
        while (i!=cloudOutput.points.size())
        {
            bool removePoint = false;

            double deltaXAbs = std::abs(cloudOutput.points.at(i).x-point.x);
            double deltaYAbs = std::abs(cloudOutput.points.at(i).y-point.y);
            double deltaZAbs = std::abs(cloudOutput.points.at(i).z-point.z);

            if (deltaXAbs<=boundingBoxX && deltaYAbs<=boundingBoxY && deltaZAbs<=boundingBoxZ)
            {
                removePoint = true;
            }

            if (removePoint)
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

}   // namespace DistributedFormation