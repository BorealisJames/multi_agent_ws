//
// Created by benson on 21/1/21.
//

#pragma once

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <eigen-cdd/Polyhedron.h>
#include <sensor_msgs/PointCloud.h>

#include "../Common/Common.h"

namespace DistributedFormation
{

class GenerateConvexRegions
{
public:

    void Generate2DConvexRegionFromPath(const sensor_msgs::PointCloud& cloud,
                                        const Common::Position& pathStart,
                                        const Common::Position& pathEnd,
                                        const double localBoundaryX,
                                        const double localBoundaryY,
                                        Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                        vec_E<Polyhedron<2>>& polys);

    void Generate3DConvexRegionFromPath(const sensor_msgs::PointCloud& cloud,
                                        const Common::Position& pathStart,
                                        const Common::Position& pathEnd,
                                        const double localBoundaryX,
                                        const double localBoundaryY,
                                        const double localBoundaryZ,
                                        Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                        vec_E<Polyhedron<3>>& polys);

    void Generate2DConvexRegionFromPoint(const sensor_msgs::PointCloud& cloud,
                                         const Common::Position& point,
                                         const double dilateRadius,
                                         const double localBoundaryX,
                                         const double localBoundaryY,
                                         Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                         Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                         Polyhedron<2>& poly);

    void Generate3DConvexRegionFromPoint(const sensor_msgs::PointCloud& cloud,
                                         const Common::Position& point,
                                         const double dilateRadius,
                                         const double localBoundaryX,
                                         const double localBoundaryY,
                                         const double localBoundaryZ,
                                         Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                         Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                         Polyhedron<3>& poly);

    bool Intersect2DConvexRegion(const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>>& AVec,
                                 const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>>& bVec,
                                 Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                 Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b);

    bool Intersect3DConvexRegion(const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>>& AVec,
                                 const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>>& bVec,
                                 Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                 Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b);

    bool Normlize2DHalfSpace (Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                              Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b);

    bool Normlize3DHalfSpace (Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                              Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b);

    bool IsPointWithin2DConvexRegion(const Common::Position& point,
                                     const Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                     const Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b);

    bool IsPointWithin3DConvexRegion(const Common::Position& point,
                                     const Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                     const Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b);
private:

};

}   // namespace DistributedFormation
