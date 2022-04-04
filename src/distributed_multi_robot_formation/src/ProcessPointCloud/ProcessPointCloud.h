//
// Created by benson on 20/8/21.
//

#pragma once

#include <sensor_msgs/PointCloud.h>
#include "../Common/Common.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


namespace DistributedFormation
{

class ProcessPointCloud
{
public:

    void AppendPointClouds(const sensor_msgs::PointCloud& cloudInput1, const sensor_msgs::PointCloud& cloudInput2,
                           sensor_msgs::PointCloud& cloudOutput);

    void RemovePointsWithinARadiusAndFromGroundFromPointCloud2D(const sensor_msgs::PointCloud& cloudInput,
                                                                const Common::Position& point,
                                                                const double removalRadius,
                                                                sensor_msgs::PointCloud& cloudOutput);

    void RemovePointsWithinARadiusPointCloud3D(const sensor_msgs::PointCloud& cloudInput,
                                               const Common::Position& point,
                                               const double removalRadius,
                                               sensor_msgs::PointCloud& cloudOutput);

    void RemovePointsFromPointCloudWithinABoundingBox2D(const sensor_msgs::PointCloud& cloudInput,
                                                        const Common::Position& point,
                                                        const double boundingBoxX,
                                                        const double boundingBoxY,
                                                        sensor_msgs::PointCloud& cloudOutput);

    void RemovePointsFromPointCloudWithinABoundingBox3D(const sensor_msgs::PointCloud& cloudInput,
                                                        const Common::Position& point,
                                                        const double boundingBoxX,
                                                        const double boundingBoxY,
                                                        const double boundingBoxZ,
                                                        sensor_msgs::PointCloud& cloudOutput);

    // Quick Fix
    bool ApplyVoxelFilterAndConvertToPointCloud(const sensor_msgs::PointCloud2& pointCloud2input, sensor_msgs::PointCloud& pointCloud1output);

private:
    float m_leaf_size_x = 0.8; // 0.7 to 0.8 seems good
    float m_leaf_size_y = 0.8; 
    float m_leaf_size_z = 0.8;
};

}   // namespace DistributedFormation
