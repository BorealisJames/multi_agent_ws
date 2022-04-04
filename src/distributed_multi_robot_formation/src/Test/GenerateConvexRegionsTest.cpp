//
// Created by benson on 28/1/21.
//

#include <gtest/gtest.h>
#include <set>

#include "../Common/Common.h"
#include "../GenerateConvexRegions/GenerateConvexRegions.h"
#include "../ProcessPointCloud/ProcessPointCloud.h"

namespace DistributedFormation
{
    TEST (GenerateConvexRegions, RemovePointsFromPointCloudWithinARadius2D)
    {
        GenerateConvexRegions generateConvexRegions;
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

    TEST (GenerateConvexRegions, RemovePointsFromPointCloudWithinARadius3D)
    {
        GenerateConvexRegions generateConvexRegions;
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

    TEST (GenerateConvexRegions, RemovePointsFromPointCloudWithinABoundingBox2D)
    {
        GenerateConvexRegions generateConvexRegions;
        ProcessPointCloud processPointCloud;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1, pt2, pt3, pt4;
        pt1.x = 0;
        pt1.y = 0;
        cloudInput.points.push_back(pt1);
        pt2.x = 1;
        pt2.y = 1;
        cloudInput.points.push_back(pt2);
        pt3.x = 2.2;
        pt3.y = 2.2;
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

        EXPECT_EQ(1, cloudOutput.points.size());
    }

    TEST (GenerateConvexRegions, RemovePointsFromPointCloudWithinABoundingBox3D)
    {
        GenerateConvexRegions generateConvexRegions;
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

    TEST (GenerateConvexRegions, Generate2DConvexRegionFromPath)
    {
        GenerateConvexRegions generateConvexRegions;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1;
        pt1.x = 0;
        pt1.y = 0.5;
        cloudInput.points.push_back(pt1);

        Common::Position pathStart;
        pathStart.x = -1;
        pathStart.y = 0;
        pathStart.z = 0;
        Common::Position pathEnd;
        pathEnd.x = 1;
        pathEnd.y = 0;
        pathEnd.z = 0;

        double localBoundaryX = 1;
        double localBoundaryY = 1;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> output_A;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> output_b;

        vec_E<Polyhedron<2>> polys2DViz;
        generateConvexRegions.Generate2DConvexRegionFromPath(cloudInput, pathStart, pathEnd, localBoundaryX, localBoundaryY, output_A, output_b, polys2DViz);

        bool success = generateConvexRegions.Normlize2DHalfSpace(output_A, output_b);

        EXPECT_TRUE(success);
        ASSERT_EQ(4, output_A.rows());
        ASSERT_EQ(4, output_b.rows());

        std::vector<Eigen::Matrix<decimal_t, 1, 3>> outputHalfSpace;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Matrix<decimal_t, 1, 3> halfSpace;
            halfSpace(0) = output_A(i,0);
            halfSpace(1) = output_A(i,1);
            halfSpace(2) = output_b(i);
            outputHalfSpace.push_back(halfSpace);
        }

        Eigen::Matrix<decimal_t, 4, 2> expected_A;
        Eigen::Matrix<decimal_t, 4, 1> expected_b;
        expected_A(0,0) = 0;
        expected_A(0,1) = 1;
        expected_b(0) = 0.5;
        expected_A(1,0) = 0;
        expected_A(1,1) = -1;
        expected_b(1) = 1.0;
        expected_A(2,0) = 1;
        expected_A(2,1) = 0;
        expected_b(2) = 2.0;
        expected_A(3,0) = -1;
        expected_A(3,1) = 0;
        expected_b(3) = 2.0;
        std::vector<Eigen::Matrix<decimal_t, 1, 3>> expectedHalfSpace;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Matrix<decimal_t, 1, 3> halfSpace;
            halfSpace(0) = expected_A(i,0);
            halfSpace(1) = expected_A(i,1);
            halfSpace(2) = expected_b(i);
            expectedHalfSpace.push_back(halfSpace);
        }

        bool equal = true;
        for (int i = 0; i < 4 && equal; i++)
        {
            for (int j = 0; j < 5 && equal; j++)
            {
                if (j == 4)
                {
                    equal = false;
                    break;
                }

                if (std::abs(expectedHalfSpace.at(i).x() - outputHalfSpace.at(j).x()) <= std::numeric_limits<double>::epsilon() &&
                        std::abs(expectedHalfSpace.at(i).y() - outputHalfSpace.at(j).y()) <= std::numeric_limits<double>::epsilon() &&
                        std::abs(expectedHalfSpace.at(i).z() - outputHalfSpace.at(j).z()) <= std::numeric_limits<double>::epsilon())
                {
                    break;
                }
            }
        }

        EXPECT_TRUE(equal);
    }

    TEST (GenerateConvexRegions, Generate3DConvexRegionFromPath)
    {
        GenerateConvexRegions generateConvexRegions;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1;
        pt1.x = 1.5;
        pt1.y = 0;
        pt1.z = 0;
        cloudInput.points.push_back(pt1);

        Common::Position pathStart;
        pathStart.x = -1;
        pathStart.y = 0;
        pathStart.z = 0;
        Common::Position pathEnd;
        pathEnd.x = 1;
        pathEnd.y = 0;
        pathEnd.z = 0;

        double localBoundaryX = 1;
        double localBoundaryY = 1;
        double localBoundaryZ = 1;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> output_A;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> output_b;

        vec_E<Polyhedron<3>> polys3DViz;
        generateConvexRegions.Generate3DConvexRegionFromPath(cloudInput, pathStart, pathEnd,
                                                             localBoundaryX, localBoundaryY, localBoundaryZ,
                                                             output_A, output_b, polys3DViz);

        bool success = generateConvexRegions.Normlize3DHalfSpace(output_A, output_b);

        EXPECT_TRUE(success);
        ASSERT_EQ(6, output_A.rows());
        ASSERT_EQ(6, output_b.rows());

        std::vector<Eigen::Matrix<decimal_t, 1, 4>> outputHalfSpace;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Matrix<decimal_t, 1, 4> halfSpace;
            halfSpace(0) = output_A(i,0);
            halfSpace(1) = output_A(i,1);
            halfSpace(2) = output_A(i,2);
            halfSpace(3) = output_b(i);
            outputHalfSpace.push_back(halfSpace);
        }

        Eigen::Matrix<decimal_t, 6, 3> expected_A;
        Eigen::Matrix<decimal_t, 6, 1> expected_b;
        expected_A(0,0) = 1;
        expected_A(0,1) = 0;
        expected_A(0,2) = 0;
        expected_b(0) = 1.5;
        expected_A(1,0) = -1;
        expected_A(1,1) = 0;
        expected_A(1,2) = 0;
        expected_b(1) = 2.0;
        expected_A(2,0) = 0;
        expected_A(2,1) = 1;
        expected_A(2,2) = 0;
        expected_b(2) = 1.0;
        expected_A(3,0) = 0;
        expected_A(3,1) = -1;
        expected_A(3,2) = 0;
        expected_b(3) = 1.0;
        expected_A(4,0) = 0;
        expected_A(4,1) = 0;
        expected_A(4,2) = 1;
        expected_b(4) = 1.0;
        expected_A(5,0) = 0;
        expected_A(5,1) = 0;
        expected_A(5,2) = -1;
        expected_b(5) = 1.0;
        std::vector<Eigen::Matrix<decimal_t, 1, 4>> expectedHalfSpace;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Matrix<decimal_t, 1, 4> halfSpace;
            halfSpace(0) = expected_A(i,0);
            halfSpace(1) = expected_A(i,1);
            halfSpace(2) = expected_A(i,2);
            halfSpace(3) = expected_b(i);
            expectedHalfSpace.push_back(halfSpace);
        }

        bool equalOverall = true;
        for (int i = 0; i < 6; i++)
        {
            bool equalFound = false;
            for (int j = 0; j < 6; j++)
            {
                if (std::abs(expectedHalfSpace.at(i)(0) - outputHalfSpace.at(j)(0)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(1) - outputHalfSpace.at(j)(1)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(2) - outputHalfSpace.at(j)(2)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(3) - outputHalfSpace.at(j)(3)) <= std::numeric_limits<double>::epsilon())
                {
                    equalFound = true;
                    break;
                }
            }

            equalOverall &= equalFound;
        }

        EXPECT_TRUE(equalOverall);
    }

    TEST (GenerateConvexRegions, Generate2DConvexRegionFromPoint)
    {
        GenerateConvexRegions generateConvexRegions;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1;
        pt1.x = 0;
        pt1.y = 0.5;
        cloudInput.points.push_back(pt1);

        Common::Position point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        double localBoundaryX = 1.0;
        double localBoundaryY = 1.0;

        double dilateRadius = 0.1;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> output_A;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> output_b;

        Polyhedron<2> poly2DViz;
        generateConvexRegions.Generate2DConvexRegionFromPoint(cloudInput, point,dilateRadius ,localBoundaryX, localBoundaryY, output_A, output_b, poly2DViz);

        bool success = generateConvexRegions.Normlize2DHalfSpace(output_A, output_b);
        EXPECT_TRUE(success);
        ASSERT_EQ(4, output_A.rows());
        ASSERT_EQ(4, output_b.rows());

        std::vector<Eigen::Matrix<decimal_t, 1, 3>> outputHalfSpace;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Matrix<decimal_t, 1, 3> halfSpace;
            halfSpace(0) = output_A(i,0);
            halfSpace(1) = output_A(i,1);
            halfSpace(2) = output_b(i);
            outputHalfSpace.push_back(halfSpace);
        }

        Eigen::Matrix<decimal_t, 4, 2> expected_A;
        Eigen::Matrix<decimal_t, 4, 1> expected_b;
        expected_A(0,0) = 0;
        expected_A(0,1) = 1;
        expected_b(0) = 0.5;
        expected_A(1,0) = 0;
        expected_A(1,1) = -1;
        expected_b(1) = 1.0;
        expected_A(2,0) = 1;
        expected_A(2,1) = 0;
        expected_b(2) = 1.0;
        expected_A(3,0) = -1;
        expected_A(3,1) = 0;
        expected_b(3) = 1.0;
        std::vector<Eigen::Matrix<decimal_t, 1, 3>> expectedHalfSpace;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Matrix<decimal_t, 1, 3> halfSpace;
            halfSpace(0) = expected_A(i,0);
            halfSpace(1) = expected_A(i,1);
            halfSpace(2) = expected_b(i);
            expectedHalfSpace.push_back(halfSpace);
        }

        bool equal = true;
        for (int i = 0; i < 4 && equal; i++)
        {
            for (int j = 0; j < 5 && equal; j++)
            {
                if (j == 4)
                {
                    equal = false;
                    break;
                }

                if (std::abs(expectedHalfSpace.at(i).x() - outputHalfSpace.at(j).x()) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i).y() - outputHalfSpace.at(j).y()) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i).z() - outputHalfSpace.at(j).z()) <= std::numeric_limits<double>::epsilon())
                {
                    break;
                }
            }
        }

        EXPECT_TRUE(equal);
    }

    TEST (GenerateConvexRegions, Generate3DConvexRegionFromPoint)
    {
        GenerateConvexRegions generateConvexRegions;

        sensor_msgs::PointCloud cloudInput;
        geometry_msgs::Point32 pt1;
        pt1.x = 0.5;
        pt1.y = 0;
        pt1.z = 0;
        cloudInput.points.push_back(pt1);

        Common::Position point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        double localBoundaryX = 1;
        double localBoundaryY = 1;
        double localBoundaryZ = 1;

        double dilateRadius = 0.1;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> output_A;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> output_b;

        Polyhedron<3> poly3DViz;

        generateConvexRegions.Generate3DConvexRegionFromPoint(cloudInput, point,
                                                             dilateRadius, localBoundaryX, localBoundaryY, localBoundaryZ,
                                                             output_A, output_b, poly3DViz);

        bool success = generateConvexRegions.Normlize3DHalfSpace(output_A, output_b);

        EXPECT_TRUE(success);
        ASSERT_EQ(6, output_A.rows());
        ASSERT_EQ(6, output_b.rows());

        std::vector<Eigen::Matrix<decimal_t, 1, 4>> outputHalfSpace;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Matrix<decimal_t, 1, 4> halfSpace;
            halfSpace(0) = output_A(i,0);
            halfSpace(1) = output_A(i,1);
            halfSpace(2) = output_A(i,2);
            halfSpace(3) = output_b(i);
            outputHalfSpace.push_back(halfSpace);
        }

        Eigen::Matrix<decimal_t, 6, 3> expected_A;
        Eigen::Matrix<decimal_t, 6, 1> expected_b;
        expected_A(0,0) = 1;
        expected_A(0,1) = 0;
        expected_A(0,2) = 0;
        expected_b(0) = 0.5;
        expected_A(1,0) = -1;
        expected_A(1,1) = 0;
        expected_A(1,2) = 0;
        expected_b(1) = 1.0;
        expected_A(2,0) = 0;
        expected_A(2,1) = 1;
        expected_A(2,2) = 0;
        expected_b(2) = 1.0;
        expected_A(3,0) = 0;
        expected_A(3,1) = -1;
        expected_A(3,2) = 0;
        expected_b(3) = 1.0;
        expected_A(4,0) = 0;
        expected_A(4,1) = 0;
        expected_A(4,2) = 1;
        expected_b(4) = 1.0;
        expected_A(5,0) = 0;
        expected_A(5,1) = 0;
        expected_A(5,2) = -1;
        expected_b(5) = 1.0;
        std::vector<Eigen::Matrix<decimal_t, 1, 4>> expectedHalfSpace;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Matrix<decimal_t, 1, 4> halfSpace;
            halfSpace(0) = expected_A(i,0);
            halfSpace(1) = expected_A(i,1);
            halfSpace(2) = expected_A(i,2);
            halfSpace(3) = expected_b(i);
            expectedHalfSpace.push_back(halfSpace);
        }

        bool equalOverall = true;
        for (int i = 0; i < 6; i++)
        {
            bool equalFound = false;
            for (int j = 0; j < 6; j++)
            {
                if (std::abs(expectedHalfSpace.at(i)(0) - outputHalfSpace.at(j)(0)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(1) - outputHalfSpace.at(j)(1)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(2) - outputHalfSpace.at(j)(2)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(3) - outputHalfSpace.at(j)(3)) <= std::numeric_limits<double>::epsilon())
                {
                    equalFound = true;
                    break;
                }
            }

            equalOverall &= equalFound;
        }

        EXPECT_TRUE(equalOverall);
    }

    TEST (GenerateConvexRegions, Intersect2DConvexRegion)
    {
        GenerateConvexRegions generateConvexRegions;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> A1;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> b1;
        A1.resize(4,2);
        b1.resize(4,1);
        A1(0,0) = 0;
        A1(0,1) = 1;
        b1(0) = 2.0;
        A1(1,0) = 0;
        A1(1,1) = -1;
        b1(1) = 2.0;
        A1(2,0) = 1;
        A1(2,1) = 0;
        b1(2) = 1.0;
        A1(3,0) = -1;
        A1(3,1) = 0;
        b1(3) = 1.0;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> A2;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> b2;
        A2.resize(4,2);
        b2.resize(4,1);
        A2(0,0) = 0;
        A2(0,1) = 1;
        b2(0) = 1.0;
        A2(1,0) = 0;
        A2(1,1) = -1;
        b2(1) = 1.0;
        A2(2,0) = 1;
        A2(2,1) = 0;
        b2(2) = 2.0;
        A2(3,0) = -1;
        A2(3,1) = 0;
        b2(3) = 2.0;

        std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>> AVec;
        std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>> bVec;

        AVec.push_back(A1);
        AVec.push_back(A2);
        bVec.push_back(b1);
        bVec.push_back(b2);

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> output_A;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> output_b;

        generateConvexRegions.Intersect2DConvexRegion(AVec, bVec, output_A, output_b);
        bool success = generateConvexRegions.Normlize2DHalfSpace(output_A, output_b);
        EXPECT_TRUE(success);
        ASSERT_EQ(4, output_A.rows());
        ASSERT_EQ(4, output_b.rows());

        std::vector<Eigen::Matrix<decimal_t, 1, 3>> outputHalfSpace;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Matrix<decimal_t, 1, 3> halfSpace;
            halfSpace(0) = output_A(i,0);
            halfSpace(1) = output_A(i,1);
            halfSpace(2) = output_b(i);
            outputHalfSpace.push_back(halfSpace);
        }

        Eigen::Matrix<decimal_t, 4, 2> expected_A;
        Eigen::Matrix<decimal_t, 4, 1> expected_b;
        expected_A(0,0) = 0;
        expected_A(0,1) = 1;
        expected_b(0) = 1;
        expected_A(1,0) = 0;
        expected_A(1,1) = -1;
        expected_b(1) = 1.0;
        expected_A(2,0) = 1;
        expected_A(2,1) = 0;
        expected_b(2) = 1.0;
        expected_A(3,0) = -1;
        expected_A(3,1) = 0;
        expected_b(3) = 1.0;
        std::vector<Eigen::Matrix<decimal_t, 1, 3>> expectedHalfSpace;
        for (int i = 0; i < 4; i++)
        {
            Eigen::Matrix<decimal_t, 1, 3> halfSpace;
            halfSpace(0) = expected_A(i,0);
            halfSpace(1) = expected_A(i,1);
            halfSpace(2) = expected_b(i);
            expectedHalfSpace.push_back(halfSpace);
        }

        bool equal = true;
        for (int i = 0; i < 4 && equal; i++)
        {
            for (int j = 0; j < 5 && equal; j++)
            {
                if (j == 4)
                {
                    equal = false;
                    break;
                }

                if (std::abs(expectedHalfSpace.at(i).x() - outputHalfSpace.at(j).x()) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i).y() - outputHalfSpace.at(j).y()) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i).z() - outputHalfSpace.at(j).z()) <= std::numeric_limits<double>::epsilon())
                {
                    break;
                }
            }
        }

        EXPECT_TRUE(equal);
    }

    TEST (GenerateConvexRegions, Intersect3DConvexRegion)
    {
        GenerateConvexRegions generateConvexRegions;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> A1;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> b1;
        A1.resize(6,3);
        b1.resize(6,1);
        A1(0,0) = 1;
        A1(0,1) = 0;
        A1(0,2) = 0;
        b1(0) = 2.0;
        A1(1,0) = -1;
        A1(1,1) = 0;
        A1(1,2) = 0;
        b1(1) = 2.0;
        A1(2,0) = 0;
        A1(2,1) = 1;
        A1(2,2) = 0;
        b1(2) = 1.0;
        A1(3,0) = 0;
        A1(3,1) = -1;
        A1(3,2) = 0;
        b1(3) = 1.0;
        A1(4,0) = 0;
        A1(4,1) = 0;
        A1(4,2) = 1;
        b1(4) = 1.0;
        A1(5,0) = 0;
        A1(5,1) = 0;
        A1(5,2) = -1;
        b1(5) = 1.0;

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> A2;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> b2;
        A2.resize(6,3);
        b2.resize(6,1);
        A2(0,0) = 1;
        A2(0,1) = 0;
        A2(0,2) = 0;
        b2(0) = 1.0;
        A2(1,0) = -1;
        A2(1,1) = 0;
        A2(1,2) = 0;
        b2(1) = 1.0;
        A2(2,0) = 0;
        A2(2,1) = 1;
        A2(2,2) = 0;
        b2(2) = 2.0;
        A2(3,0) = 0;
        A2(3,1) = -1;
        A2(3,2) = 0;
        b2(3) = 2.0;
        A2(4,0) = 0;
        A2(4,1) = 0;
        A2(4,2) = 1;
        b2(4) = 1.0;
        A2(5,0) = 0;
        A2(5,1) = 0;
        A2(5,2) = -1;
        b2(5) = 1.0;

        std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>> AVec;
        std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>> bVec;

        AVec.push_back(A1);
        AVec.push_back(A2);
        bVec.push_back(b1);
        bVec.push_back(b2);

        Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> output_A;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> output_b;

        generateConvexRegions.Intersect3DConvexRegion(AVec, bVec, output_A, output_b);
        bool success = generateConvexRegions.Normlize3DHalfSpace(output_A, output_b);
        EXPECT_TRUE(success);
        ASSERT_EQ(6, output_A.rows());
        ASSERT_EQ(6, output_b.rows());

        std::vector<Eigen::Matrix<decimal_t, 1, 4>> outputHalfSpace;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Matrix<decimal_t, 1, 4> halfSpace;
            halfSpace(0) = output_A(i,0);
            halfSpace(1) = output_A(i,1);
            halfSpace(2) = output_A(i,2);
            halfSpace(3) = output_b(i);
            outputHalfSpace.push_back(halfSpace);
        }

        Eigen::Matrix<decimal_t, 6, 3> expected_A;
        Eigen::Matrix<decimal_t, 6, 1> expected_b;
        expected_A(0,0) = 1;
        expected_A(0,1) = 0;
        expected_A(0,2) = 0;
        expected_b(0) = 1.0;
        expected_A(1,0) = -1;
        expected_A(1,1) = 0;
        expected_A(1,2) = 0;
        expected_b(1) = 1.0;
        expected_A(2,0) = 0;
        expected_A(2,1) = 1;
        expected_A(2,2) = 0;
        expected_b(2) = 1.0;
        expected_A(3,0) = 0;
        expected_A(3,1) = -1;
        expected_A(3,2) = 0;
        expected_b(3) = 1.0;
        expected_A(4,0) = 0;
        expected_A(4,1) = 0;
        expected_A(4,2) = 1;
        expected_b(4) = 1.0;
        expected_A(5,0) = 0;
        expected_A(5,1) = 0;
        expected_A(5,2) = -1;
        expected_b(5) = 1.0;
        std::vector<Eigen::Matrix<decimal_t, 1, 4>> expectedHalfSpace;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Matrix<decimal_t, 1, 4> halfSpace;
            halfSpace(0) = expected_A(i,0);
            halfSpace(1) = expected_A(i,1);
            halfSpace(2) = expected_A(i,2);
            halfSpace(3) = expected_b(i);
            expectedHalfSpace.push_back(halfSpace);
        }

        bool equalOverall = true;
        for (int i = 0; i < 6; i++)
        {
            bool equalFound = false;
            for (int j = 0; j < 6; j++)
            {
                if (std::abs(expectedHalfSpace.at(i)(0) - outputHalfSpace.at(j)(0)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(1) - outputHalfSpace.at(j)(1)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(2) - outputHalfSpace.at(j)(2)) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedHalfSpace.at(i)(3) - outputHalfSpace.at(j)(3)) <= std::numeric_limits<double>::epsilon())
                {
                    equalFound = true;
                    break;
                }
            }

            equalOverall &= equalFound;
        }

        EXPECT_TRUE(equalOverall);

    }

    TEST (GenerateConvexRegions, Normlize2DHalfSpace)
    {
        GenerateConvexRegions generateConvexRegions;

        Eigen::Matrix<decimal_t, -1, 2> A;
        Eigen::Matrix<decimal_t, -1, 1> b;

        A.resize(2,2);
        b.resize(2,1);

        A(0,0) = 1;
        A(0,1) = 1;
        A(1,0) = 2;
        A(1,1) = 3;
        b(0) = 1;
        b(1) = 2;

        bool success = generateConvexRegions.Normlize2DHalfSpace(A, b);

        EXPECT_TRUE(success);
        EXPECT_DOUBLE_EQ(1.0/sqrt(2), A(0,0));
        EXPECT_DOUBLE_EQ(1.0/sqrt(2), A(0,1));
        EXPECT_DOUBLE_EQ(1.0/sqrt(2), b(0));
        EXPECT_DOUBLE_EQ(2.0/sqrt(13), A(1,0));
        EXPECT_DOUBLE_EQ(3.0/sqrt(13), A(1,1));
        EXPECT_DOUBLE_EQ(2.0/sqrt(13), b(1));
    }

    TEST (GenerateConvexRegions, Normlize3DHalfSpace)
    {
        GenerateConvexRegions generateConvexRegions;

        Eigen::Matrix<decimal_t, -1, 3> A;
        Eigen::Matrix<decimal_t, -1, 1> b;

        A.resize(2,3);
        b.resize(2,1);

        A(0,0) = 1;
        A(0,1) = 1;
        A(0,2) = 1;
        A(1,0) = 2;
        A(1,1) = 3;
        A(1,2) = 4;
        b(0) = 1;
        b(1) = 2;

        bool success = generateConvexRegions.Normlize3DHalfSpace(A, b);

        EXPECT_TRUE(success);
        EXPECT_DOUBLE_EQ(1.0/sqrt(3), A(0,0));
        EXPECT_DOUBLE_EQ(1.0/sqrt(3), A(0,1));
        EXPECT_DOUBLE_EQ(1.0/sqrt(3), A(0,2));
        EXPECT_DOUBLE_EQ(1.0/sqrt(3), b(0));
        EXPECT_DOUBLE_EQ(2.0/sqrt(29), A(1,0));
        EXPECT_DOUBLE_EQ(3.0/sqrt(29), A(1,1));
        EXPECT_DOUBLE_EQ(4.0/sqrt(29), A(1,2));
        EXPECT_DOUBLE_EQ(2.0/sqrt(29), b(1));
    }
} // namespace DistributedFormation