//
// Created by benson on 28/1/21.
//

#include <gtest/gtest.h>
#include <set>

#include "../Common/Common.h"
#include "../ConvexHullOfRobotPosition/ConvexHullOfRobotPosition.h"

namespace DistributedFormation
{
    TEST (ConvexHullOfRobotPosition, Convexhull2D)
    {
        double pt1_x = 0;
        double pt1_y = 1;
        double pt2_x = -1;
        double pt2_y = -1;
        double pt3_x = 1;
        double pt3_y = -1;
        double pt4_x = 0;
        double pt4_y = 0;
        double pt5_x = 0;
        double pt5_y = -1;


        ConvexHullOfRobotPosition convexHullOfRobotPosition;

        Eigen::Matrix<double, Eigen::Dynamic, 2> agentsInputPosition2D (5, 2);
        Eigen::Matrix<double, Eigen::Dynamic, 2> agentsOutputPosition2D;

        agentsInputPosition2D(0,0) = pt1_x;
        agentsInputPosition2D(0,1) = pt1_y;
        agentsInputPosition2D(1,0) = pt2_x;
        agentsInputPosition2D(1,1) = pt2_y;
        agentsInputPosition2D(2,0) = pt3_x;
        agentsInputPosition2D(2,1) = pt3_y;
        agentsInputPosition2D(3,0) = pt4_x;
        agentsInputPosition2D(3,1) = pt4_y;
        agentsInputPosition2D(4,0) = pt5_x;
        agentsInputPosition2D(4,1) = pt5_y;

        convexHullOfRobotPosition.Convexhull2D(agentsInputPosition2D, agentsOutputPosition2D);

        EXPECT_EQ(3, agentsOutputPosition2D.rows());

        std::vector<Common::Position> expectedPositions;
        Common::Position ep1, ep2, ep3;
        ep1.x = pt1_x;
        ep1.y = pt1_y;
        ep1.z = 0;
        expectedPositions.push_back(ep1);
        ep2.x = pt2_x;
        ep2.y = pt2_y;
        ep2.z = 0;
        expectedPositions.push_back(ep2);
        ep3.x = pt3_x;
        ep3.y = pt3_y;
        ep3.z = 0;
        expectedPositions.push_back(ep3);

        std::vector<Common::Position> outputPositions;
        for (int i=0; i<agentsOutputPosition2D.rows(); i++)
        {
            Common::Position op;
            op.x = agentsOutputPosition2D(i, 0);
            op.y = agentsOutputPosition2D(i, 1);
            op.z = 0;
            outputPositions.push_back(op);
        }

        bool equal = true;
        for (int i = 0; i < 3 && equal; i++)
        {
            for (int j = 0; j < 4 && equal; j++)
            {
                if (j == 3)
                {
                    equal = false;
                    break;
                }

                if (std::abs(expectedPositions.at(i).x - outputPositions.at(j).x) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedPositions.at(i).y - outputPositions.at(j).y) <= std::numeric_limits<double>::epsilon())
                {
                    break;
                }
            }
        }

        EXPECT_TRUE(equal);
    }

    TEST (ConvexHullOfRobotPosition, Convexhull3D)
    {
        double pt1_x = 1;
        double pt1_y = 1;
        double pt1_z = 1;
        double pt2_x = -1;
        double pt2_y = 1;
        double pt2_z = 1;
        double pt3_x = 1;
        double pt3_y = -1;
        double pt3_z = 1;
        double pt4_x = 1;
        double pt4_y = 1;
        double pt4_z = -1;
        double pt5_x = -1;
        double pt5_y = -1;
        double pt5_z = 1;
        double pt6_x = -1;
        double pt6_y = 1;
        double pt6_z = -1;
        double pt7_x = 1;
        double pt7_y = -1;
        double pt7_z = -1;
        double pt8_x = -1;
        double pt8_y = -1;
        double pt8_z = -1;
        double pt9_x = 0;
        double pt9_y = 0;
        double pt9_z = 0;



        ConvexHullOfRobotPosition convexHullOfRobotPosition;

        Eigen::Matrix<double, Eigen::Dynamic, 3> agentsInputPosition3D (9, 3);
        Eigen::Matrix<double, Eigen::Dynamic, 3> agentsOutputPosition3D;

        agentsInputPosition3D(0, 0) = pt1_x;
        agentsInputPosition3D(0, 1) = pt1_y;
        agentsInputPosition3D(0, 2) = pt1_z;
        agentsInputPosition3D(1, 0) = pt2_x;
        agentsInputPosition3D(1, 1) = pt2_y;
        agentsInputPosition3D(1, 2) = pt2_z;
        agentsInputPosition3D(2, 0) = pt3_x;
        agentsInputPosition3D(2, 1) = pt3_y;
        agentsInputPosition3D(2, 2) = pt3_z;
        agentsInputPosition3D(3, 0) = pt4_x;
        agentsInputPosition3D(3, 1) = pt4_y;
        agentsInputPosition3D(3, 2) = pt4_z;
        agentsInputPosition3D(4, 0) = pt5_x;
        agentsInputPosition3D(4, 1) = pt5_y;
        agentsInputPosition3D(4, 2) = pt5_z;
        agentsInputPosition3D(5, 0) = pt6_x;
        agentsInputPosition3D(5, 1) = pt6_y;
        agentsInputPosition3D(5, 2) = pt6_z;
        agentsInputPosition3D(6, 0) = pt7_x;
        agentsInputPosition3D(6, 1) = pt7_y;
        agentsInputPosition3D(6, 2) = pt7_z;
        agentsInputPosition3D(7, 0) = pt8_x;
        agentsInputPosition3D(7, 1) = pt8_y;
        agentsInputPosition3D(7, 2) = pt8_z;
        agentsInputPosition3D(8, 0) = pt9_x;
        agentsInputPosition3D(8, 1) = pt9_y;
        agentsInputPosition3D(8, 2) = pt9_z;
        //expect pt9 (0,0,0) to be removed within the cube

        convexHullOfRobotPosition.Convexhull3D(agentsInputPosition3D, agentsOutputPosition3D);

        EXPECT_EQ(8 , agentsOutputPosition3D.rows());

        std::vector<Common::Position> expectedPositions;
        for (int i=0; i < 8; i++)
        {
            Common::Position ep;
            ep.x = agentsInputPosition3D(i, 0);
            ep.y = agentsInputPosition3D(i, 1);
            ep.z = agentsInputPosition3D(i, 2);

            expectedPositions.push_back(ep);
        }

        std::vector<Common::Position> outputPositions;
        for (int i=0; i < agentsOutputPosition3D.rows(); i++)
        {
            Common::Position op;
            op.x = agentsOutputPosition3D(i, 0);
            op.y = agentsOutputPosition3D(i, 1);
            op.z = agentsOutputPosition3D(i, 2);
            outputPositions.push_back(op);
        }

        bool equalOverall = true;
        for (int i = 0; i < 8; i++)
        {
            bool equalFound = false;
            for (int j = 0; j < 8; j++)
            {
                if (std::abs(expectedPositions.at(i).x - outputPositions.at(j).x) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedPositions.at(i).y - outputPositions.at(j).y) <= std::numeric_limits<double>::epsilon() &&
                    std::abs(expectedPositions.at(i).z - outputPositions.at(j).z) <= std::numeric_limits<double>::epsilon())
                {
                    equalFound = true;
                    break;
                }
            }
            equalOverall &= equalFound;
        }

        EXPECT_TRUE(equalOverall);
    }

} // namespace DistributedFormation