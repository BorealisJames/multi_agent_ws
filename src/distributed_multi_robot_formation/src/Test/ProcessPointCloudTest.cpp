//
// Created by benson on 28/1/21.
//

#include <gtest/gtest.h>
#include <set>

#include "../Common/Common.h"
#include "../ProcessPointCloud/ProcessPointCloud.h"

namespace DistributedFormation
{
    TEST (ProcessPointCloud, AppendPointClouds)
    {
        ProcessPointCloud processPointCloud;

        sensor_msgs::PointCloud cloudInput1;
        geometry_msgs::Point32 pt1, pt2;
        pt1.x = 0;
        pt1.y = 0;
        pt1.z = 4;
        cloudInput1.points.push_back(pt1);
        pt2.x = 1;
        pt2.y = 1;
        pt2.z = 100;
        cloudInput1.points.push_back(pt2);

        sensor_msgs::PointCloud cloudInput2;
        geometry_msgs::Point32 pt3, pt4;
        pt3.x = 5;
        pt3.y = 4;
        pt3.z = 4;
        cloudInput2.points.push_back(pt3);
        pt4.x = -4;
        pt4.y = -7;
        pt4.z = 0;
        cloudInput2.points.push_back(pt4);

        sensor_msgs::PointCloud cloudOutput;

        processPointCloud.AppendPointClouds(cloudInput1, cloudInput2, cloudOutput);

        EXPECT_EQ(4, cloudOutput.points.size());
    }

    TEST (ProcessPointCloud, RemovePointsFromPointCloudWithinARadius2D)
    {
        ProcessPointCloud processPointCloud;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1, pt2, pt3, pt4;
        pt1.x = 0;
        pt1.y = 0;
        pt1.z = 4;
        cloudInput.points.push_back(pt1); // removed as its within removal radius
        pt2.x = 1;
        pt2.y = 1;
        pt2.z = 100;    //to show that in 2D all the points are flattened (i.e. all points exist on the same z level)
        cloudInput.points.push_back(pt2); // removed as its within removal radius
        pt3.x = 5;
        pt3.y = 4;
        pt3.z = 4;
        cloudInput.points.push_back(pt3); // not removed as its not within removal radius
        pt4.x = -4;
        pt4.y = -7;
        pt4.z = 0;
        cloudInput.points.push_back(pt4); // removed as its too near ground

        Common::Position point;
        point.x = 0.5;
        point.y = 0.5;

        double removalRadius = 4;

        sensor_msgs::PointCloud cloudOutput;

        processPointCloud.RemovePointsWithinARadiusAndFromGroundFromPointCloud2D(cloudInput, point, removalRadius,
                                                                                     cloudOutput);

        EXPECT_EQ(1, cloudOutput.points.size());
    }

    TEST (ProcessPointCloud, RemovePointsFromPointCloudWithinARadius3D)
    {
        ProcessPointCloud processPointCloud;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1, pt2, pt3;
        pt1.x = 0;
        pt1.y = 0;
        pt1.z = 4;
        cloudInput.points.push_back(pt1); // removed as its within removal radius
        pt2.x = 1;
        pt2.y = 1;
        pt2.z = 100;    //to show that this point is treated differently in 3D
        cloudInput.points.push_back(pt2); // not removed as its not within removal radius
        pt3.x = 5;
        pt3.y = 4;
        pt3.z = 4;
        cloudInput.points.push_back(pt3); // not removed as its not within removal radius

        Common::Position point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        double removalRadius = 4;

        sensor_msgs::PointCloud cloudOutput;

        processPointCloud.RemovePointsWithinARadiusPointCloud3D(cloudInput, point, removalRadius,
                                                                    cloudOutput);

        EXPECT_EQ(2, cloudOutput.points.size());
    }

    TEST (ProcessPointCloud, RemovePointsFromPointCloudWithinABoundingBox2D)
    {
        ProcessPointCloud processPointCloud;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1, pt2, pt3, pt4;
        pt1.x = 0;
        pt1.y = 0;
        cloudInput.points.push_back(pt1);
        pt2.x = 1;
        pt2.y = 1;
        cloudInput.points.push_back(pt2);
        pt3.x = 5;
        pt3.y = 4;
        cloudInput.points.push_back(pt3);
        pt4.x = -4;
        pt4.y = -7;
        cloudInput.points.push_back(pt4);

        Common::Position point;
        point.x = 0.5;
        point.y = 0.5;

        double boundingBoxX = 2;
        double boundingBoxY = 2;

        sensor_msgs::PointCloud cloudOutput;

        processPointCloud.RemovePointsFromPointCloudWithinABoundingBox2D(cloudInput, point, boundingBoxX,
                                                                             boundingBoxY, cloudOutput);

        EXPECT_EQ(2, cloudOutput.points.size());
    }

    TEST (ProcessPointCloud, RemovePointsFromPointCloudWithinABoundingBox3D)
    {
        ProcessPointCloud processPointCloud;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1, pt2, pt3, pt4;
        pt1.x = 0;
        pt1.y = 0;
        pt1.z = 0;
        cloudInput.points.push_back(pt1);
        pt2.x = 1;
        pt2.y = 1;
        pt2.z = 1;
        cloudInput.points.push_back(pt2);
        pt3.x = 1;
        pt3.y = 1;
        pt3.z = 3;
        cloudInput.points.push_back(pt3);
        pt4.x = 2.1;
        pt4.y = 2.1;
        pt4.z = 2.1;
        cloudInput.points.push_back(pt4);

        Common::Position point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        double boundingBoxX = 2;
        double boundingBoxY = 2;
        double boundingBoxZ = 2;

        sensor_msgs::PointCloud cloudOutput;

        processPointCloud.RemovePointsFromPointCloudWithinABoundingBox3D(cloudInput, point,
                                                                             boundingBoxX, boundingBoxY, boundingBoxZ,
                                                                             cloudOutput);

        EXPECT_EQ(2, cloudOutput.points.size());
    }

} // namespace DistributedFormation