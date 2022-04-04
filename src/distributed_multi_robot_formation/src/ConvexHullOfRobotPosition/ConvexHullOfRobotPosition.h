//
// Created by benson on 18/1/21.
//

#pragma once

#include <Eigen/Core>
#include <eigen-cdd/Polyhedron.h>
#include <vector>

namespace DistributedFormation
{

class ConvexHullOfRobotPosition
{
public:

    bool Convexhull2D (const Eigen::Matrix<double, Eigen::Dynamic, 2>& input_points,
                       Eigen::Matrix<double, Eigen::Dynamic, 2>& output_points);

    bool Convexhull3D (const Eigen::Matrix<double, Eigen::Dynamic, 3>& input_points,
                       Eigen::Matrix<double, Eigen::Dynamic, 3>& output_points);
};

}  // namespace DistributedFormation
