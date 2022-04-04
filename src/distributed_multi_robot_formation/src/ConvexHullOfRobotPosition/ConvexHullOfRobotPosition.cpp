//
// Created by benson on 18/1/21.
//

#include "ConvexHullOfRobotPosition.h"

namespace DistributedFormation
{

    bool
    ConvexHullOfRobotPosition::Convexhull2D (const Eigen::Matrix<double, Eigen::Dynamic, 2>& input_points,
                                             Eigen::Matrix<double, Eigen::Dynamic, 2>& output_points)
    {
        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setVertices(input_points);
        if (!success)
        {
            return false;
        }
        auto hrep = poly1.hrep();

        Eigen::Polyhedron poly2;
        success = poly2.setHrep(hrep.first, hrep.second);
        if (!success)
        {
            return false;
        }
        auto vrep = poly2.vrep();

        output_points.conservativeResize(0, Eigen::NoChange);
        for (int i=0; i<vrep.second.rows(); ++i)
        {
            if (vrep.second(i,0) != 0.0)
            {
                output_points.conservativeResize(output_points.rows()+1, Eigen::NoChange);
                output_points.bottomRows(1) = vrep.first.block<1,2>(i,0);
            }
        }

        return true;
    }

    bool
    ConvexHullOfRobotPosition::Convexhull3D (const Eigen::Matrix<double, Eigen::Dynamic, 3>& input_points,
                                             Eigen::Matrix<double, Eigen::Dynamic, 3>& output_points)
    {
        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setVertices(input_points);
        if (!success)
        {
            return false;
        }
        auto hrep = poly1.hrep();

        Eigen::Polyhedron poly2;
        success = poly2.setHrep(hrep.first, hrep.second);
        if (!success)
        {
            return false;
        }
        auto vrep = poly2.vrep();

        output_points.conservativeResize(0, Eigen::NoChange);
        for (int i=0; i<vrep.second.rows(); ++i)
        {
            if (vrep.second(i,0) != 0.0)
            {
                output_points.conservativeResize(output_points.rows()+1, Eigen::NoChange);
                output_points.bottomRows(1) = vrep.first.block<1,3>(i,0);
            }
        }

        return true;
    }

}  // namespace DistributedFormation