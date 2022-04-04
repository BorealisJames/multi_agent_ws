//
// Created by benson on 21/1/21.
//

#include "GenerateConvexRegions.h"

namespace DistributedFormation
{
    void
    GenerateConvexRegions::Generate2DConvexRegionFromPath(const sensor_msgs::PointCloud& cloud,
                                                          const Common::Position& pathStart,
                                                          const Common::Position& pathEnd,
                                                          const double localBoundaryX,
                                                          const double localBoundaryY,
                                                          Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                                          Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                                          vec_E<Polyhedron<2>>& polys)
    {
        vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);
        vec_Vec2f obs2d;
        for(const auto& it: obs)
        {
            obs2d.push_back(it.topRows<2>());
        }

        vec_Vec2f path;
        path.push_back(Vec2f(pathStart.x, pathStart.y));
        path.push_back(Vec2f(pathEnd.x, pathEnd.y));

        //Using ellipsoid decomposition
        EllipsoidDecomp2D decomp_util;
        decomp_util.set_obs(obs2d);
        decomp_util.set_local_bbox(Vec2f(localBoundaryX, localBoundaryY));
        decomp_util.dilate(path);

        polys = decomp_util.get_polyhedrons();

        //Convert to inequality constraints Ax < b
        A.conservativeResize(0, Eigen::NoChange);
        b.conservativeResize(0, Eigen::NoChange);
        auto constraints = decomp_util.get_constraints();
        for (auto constraint : constraints)
        {
            A.conservativeResize(A.rows()+constraint.A().rows(), Eigen::NoChange);
            A.bottomRows(constraint.A().rows()) = constraint.A();

            b.conservativeResize(b.rows()+constraint.b().rows(), Eigen::NoChange);
            b.bottomRows(constraint.b().rows()) = constraint.b();
        }

        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setHrep(A, b);
        if (!success)
        {
            return;
        }
        auto vrep = poly1.vrep();

        Eigen::Polyhedron poly2;
        success = poly2.setVrep(vrep.first, vrep.second);
        if (!success)
        {
            return;
        }
        auto hrep = poly2.hrep();
        A = hrep.first;
        b = hrep.second;
    }

    void GenerateConvexRegions::Generate3DConvexRegionFromPath(const sensor_msgs::PointCloud& cloud,
                                                                const Common::Position& pathStart,
                                                                const Common::Position& pathEnd,
                                                                const double localBoundaryX,
                                                                const double localBoundaryY,
                                                                const double localBoundaryZ,
                                                                Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                                                Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                                                vec_E<Polyhedron<3>>& polys)
    {
        vec_Vec3f obs3d = DecompROS::cloud_to_vec(cloud);

        vec_Vec3f path;
        path.push_back(Vec3f(pathStart.x, pathStart.y, pathStart.z));
        path.push_back(Vec3f(pathEnd.x, pathEnd.y, pathEnd.z));

        //Using ellipsoid decomposition
        EllipsoidDecomp3D decomp_util;
        decomp_util.set_obs(obs3d);
        decomp_util.set_local_bbox(Vec3f(localBoundaryX, localBoundaryY, localBoundaryZ));
        decomp_util.dilate(path);

        polys = decomp_util.get_polyhedrons();

        //Convert to inequality constraints Ax < b
        A.conservativeResize(0, Eigen::NoChange);
        b.conservativeResize(0, Eigen::NoChange);
        auto constraints = decomp_util.get_constraints();
        for (auto constraint : constraints)
        {
            A.conservativeResize(A.rows()+constraint.A().rows(), Eigen::NoChange);
            A.bottomRows(constraint.A().rows()) = constraint.A();

            b.conservativeResize(b.rows()+constraint.b().rows(), Eigen::NoChange);
            b.bottomRows(constraint.b().rows()) = constraint.b();
        }

        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setHrep(A, b);
        if (!success)
        {
            return;
        }
        auto vrep = poly1.vrep();

        Eigen::Polyhedron poly2;
        success = poly2.setVrep(vrep.first, vrep.second);
        if (!success)
        {
            return;
        }
        auto hrep = poly2.hrep();
        A = hrep.first;
        b = hrep.second;
    }

    void
    GenerateConvexRegions::Generate2DConvexRegionFromPoint(const sensor_msgs::PointCloud& cloud,
                                         const Common::Position& point,
                                         const double dilateRadius,
                                         const double localBoundaryX,
                                         const double localBoundaryY,
                                         Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                         Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                         Polyhedron<2>& poly)
    {
        vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);
        vec_Vec2f obs2d;
        for(const auto& it: obs)
        {
            obs2d.push_back(it.topRows<2>());
        }

        Vec2f seed(Vec2f(point.x, point.y));

        //Using seed decomposition
        SeedDecomp2D decomp_util(seed);
        decomp_util.set_obs(obs2d);
        decomp_util.set_local_bbox(Vec2f(localBoundaryX, localBoundaryY));
        decomp_util.dilate(dilateRadius);

        poly = decomp_util.get_polyhedron();

        //Get inequality constraints Ax < b
        LinearConstraint<2> constraint (seed, decomp_util.get_polyhedron().hyperplanes());
        A = constraint.A();
        b = constraint.b();

        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setHrep(A, b);
        if (!success)
        {
            return;
        }
        auto vrep = poly1.vrep();

        Eigen::Polyhedron poly2;
        success = poly2.setVrep(vrep.first, vrep.second);
        if (!success)
        {
            return;
        }
        auto hrep = poly2.hrep();
        A = hrep.first;
        b = hrep.second;
    }

    void
    GenerateConvexRegions::Generate3DConvexRegionFromPoint(const sensor_msgs::PointCloud& cloud,
                                                             const Common::Position& point,
                                                             const double dilateRadius,
                                                             const double localBoundaryX,
                                                             const double localBoundaryY,
                                                             const double localBoundaryZ,
                                                             Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                                             Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b,
                                                             Polyhedron<3>& poly)
    {
        vec_Vec3f obs3d = DecompROS::cloud_to_vec(cloud);

        Vec3f seed(Vec3f(point.x, point.y, point.z));

        //Using seed decomposition
        SeedDecomp3D decomp_util(seed);
        decomp_util.set_obs(obs3d);
        decomp_util.set_local_bbox(Vec3f(localBoundaryX, localBoundaryY, localBoundaryZ));
        decomp_util.dilate(dilateRadius);

        poly = decomp_util.get_polyhedron();

        //Get inequality constraints Ax < b
        LinearConstraint<3> constraint (seed, decomp_util.get_polyhedron().hyperplanes());
        A = constraint.A();
        b = constraint.b();

        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setHrep(A, b);
        if (!success)
        {
            return;
        }
        auto vrep = poly1.vrep();

        Eigen::Polyhedron poly2;
        success = poly2.setVrep(vrep.first, vrep.second);
        if (!success)
        {
            return;
        }
        auto hrep = poly2.hrep();
        A = hrep.first;
        b = hrep.second;
    }

    bool
    GenerateConvexRegions::Intersect2DConvexRegion(const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>>& AVec,
                                 const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>>& bVec,
                                 Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                 Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b)
    {
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 2> AConcatenate;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bConcatenate;

        AConcatenate.conservativeResize(0, Eigen::NoChange);
        bConcatenate.conservativeResize(0, Eigen::NoChange);

        size_t sizeOfConstraints = 0;
        if (AVec.size() != bVec.size())
        {
            return false;
        }
        else
        {
            sizeOfConstraints = AVec.size();
        }

        for (int i=0; i<sizeOfConstraints; ++i)
        {
            AConcatenate.conservativeResize(AConcatenate.rows()+AVec.at(i).rows(), Eigen::NoChange);
            AConcatenate.bottomRows(AVec.at(i).rows()) = AVec.at(i);

            bConcatenate.conservativeResize(bConcatenate.rows()+bVec.at(i).rows(), Eigen::NoChange);
            bConcatenate.bottomRows(bVec.at(i).rows()) = bVec.at(i);
        }

        //converting between v and h representation of convex hull to remove redundant constraints
        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setHrep(AConcatenate, bConcatenate);
        if (!success)
        {
            return false;
        }
        auto vrep = poly1.vrep();

        Eigen::Polyhedron poly2;
        success = poly2.setVrep(vrep.first, vrep.second);
        if (!success)
        {
            return false;
        }
        auto hrep = poly2.hrep();

        A = hrep.first;
        b = hrep.second;

        return true;
    }

    bool
    GenerateConvexRegions::Intersect3DConvexRegion(const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>>& AVec,
                                                     const std::vector<Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>>& bVec,
                                                     Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                                     Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b)
    {
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> AConcatenate;
        Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> bConcatenate;

        AConcatenate.conservativeResize(0, Eigen::NoChange);
        bConcatenate.conservativeResize(0, Eigen::NoChange);

        size_t sizeOfConstraints = 0;
        if (AVec.size() != bVec.size())
        {
            return false;
        }
        else
        {
            sizeOfConstraints = AVec.size();
        }

        for (int i=0; i<sizeOfConstraints; ++i)
        {
            AConcatenate.conservativeResize(AConcatenate.rows()+AVec.at(i).rows(), Eigen::NoChange);
            AConcatenate.bottomRows(AVec.at(i).rows()) = AVec.at(i);

            bConcatenate.conservativeResize(bConcatenate.rows()+bVec.at(i).rows(), Eigen::NoChange);
            bConcatenate.bottomRows(bVec.at(i).rows()) = bVec.at(i);
        }

        //converting between v and h representation of convex hull to remove redundant constraints
        bool success = true;
        Eigen::Polyhedron poly1;
        success = poly1.setHrep(AConcatenate, bConcatenate);
        if (!success)
        {
            return false;
        }
        auto vrep = poly1.vrep();

        Eigen::Polyhedron poly2;
        success = poly2.setVrep(vrep.first, vrep.second);
        if (!success)
        {
            return false;
        }
        auto hrep = poly2.hrep();

        A = hrep.first;
        b = hrep.second;

        return true;
    }

    bool
    GenerateConvexRegions::Normlize2DHalfSpace (Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                                Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b)
    {
        size_t sizeOfConstraints = 0;
        if (A.rows() != b.rows())
        {
            return false;
        }
        else
        {
            sizeOfConstraints = A.rows();
        }

        for (int i=0; i<sizeOfConstraints; ++i)
        {
            double norm = std::hypot(A(i,0), A(i,1));

            if (std::abs(norm) <= std::numeric_limits<double>::epsilon())
            {
                continue;
            }

            A(i,0) = A(i,0) / norm;
            A(i,1) = A(i,1) / norm;
            b(i) = b(i) / norm;
        }

        return true;
    }

    bool
    GenerateConvexRegions::Normlize3DHalfSpace (Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                                Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b)
    {
        size_t sizeOfConstraints = 0;
        if (A.rows() != b.rows())
        {
            return false;
        }
        else
        {
            sizeOfConstraints = A.rows();
        }

        for (int i=0; i<sizeOfConstraints; ++i)
        {
            double norm = std::sqrt(std::pow(A(i,0), 2) +
                                    std::pow(A(i,1), 2) +
                                    std::pow(A(i,2), 2));

            if (std::abs(norm) <= std::numeric_limits<double>::epsilon())
            {
                continue;
            }

            A(i,0) = A(i,0) / norm;
            A(i,1) = A(i,1) / norm;
            A(i,2) = A(i,2) / norm;
            b(i) = b(i) / norm;
        }

        return true;
    }

    bool
    GenerateConvexRegions::IsPointWithin2DConvexRegion(const Common::Position& point,
                                                         const Eigen::Matrix<decimal_t, Eigen::Dynamic, 2>& A,
                                                         const Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b)
    {
        size_t sizeOfConstraints = 0;
        if (A.rows() != b.rows())
        {
            return false;
        }
        else
        {
            sizeOfConstraints = A.rows();
        }

        bool pointIsWithin2DConvexRegion = true;

        for (int i=0; i<sizeOfConstraints; ++i)
        {
            if (A(i,0)*point.x + A(i,1)*point.y > b(i))
            {
                pointIsWithin2DConvexRegion = false;
                break;
            }
        }

        return pointIsWithin2DConvexRegion;
    }

    bool
    GenerateConvexRegions::IsPointWithin3DConvexRegion(const Common::Position& point,
                                                     const Eigen::Matrix<decimal_t, Eigen::Dynamic, 3>& A,
                                                     const Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>& b)
    {
        size_t sizeOfConstraints = 0;
        if (A.rows() != b.rows())
        {
            return false;
        }
        else
        {
            sizeOfConstraints = A.rows();
        }

        bool pointIsWithin3DConvexRegion = true;

        for (int i=0; i<sizeOfConstraints; ++i)
        {
            if (A(i,0)*point.x + A(i,1)*point.y + A(i,2)*point.z > b(i))
            {
                pointIsWithin3DConvexRegion = false;
                break;
            }
        }

        return pointIsWithin3DConvexRegion;
    }

}   // namespace DistributedFormation