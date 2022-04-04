//
// Created by benson on 13/1/21.
//


#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <math.h>
#include <vector>

namespace DistributedFormation
{

class Common
{

# define k_oneOverSqrtOfThree		0.577350269

public:

    enum class PHASE
    {
        PHASE_1 = 1,   //PHASE_1 is convex hull of robot position
        PHASE_2 = 2,   //PHASE_2 is direction of motionOptimizerVarsConstrCost
        PHASE_3 = 3,   //PHASE_3 is intersection of convex regions
        PHASE_4 = 4,   //PHASE_4 is check that formation position is feasible between agents
        PHASE_5 = 5    //PHASE_5 is assignment of formation position
    };

    enum class WORKSPACE
    {
        DIM_2_WITH_YAW = 0,
        DIM_2_WITHOUT_YAW = 1,
        DIM_3_WITH_ROT = 2,
        DIM_3_WITHOUT_ROT = 3,
        DIM_3_WITH_ONLY_YAW = 4
    };

    enum class DIMENSION
    {
        DIM_2 = 2,
        DIM_3 = 3
    };

    enum class Formation2DType
    {
        NO_FORMATION = 0,
        TRIANGLE_FORMATION = 1,
        LINE_FORMATION = 2,
        ABREAST_FORMATION = 3
    };

    enum class Formation3DType
    {
        NO_FORMATION = 0,
        TRIANGLE_FORMATION = 1,
        LINE_FORMATION = 2,
        ABREAST_FORMATION = 3
    };

    struct PhaseAndTime
    {
        PHASE phase;
        int64_t timeMicroSecs;
    };

    struct Position
    {
        double x;
        double y;
        double z;
    };

    struct DirectionUtility
    {
        std::vector<double> angleIndexAndUtility;
    };

    struct ConvexRegion2D
    {
        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
    };

    struct ConvexRegion3D
    {
        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
    };

    struct Pose
    {
        Position position;
        double headingRad;
    };

    static double MinusPiToPi (const double angleRad)
    {
        double retAngleRad = angleRad;

        while (retAngleRad < -M_PI)
        {
            retAngleRad += 2*M_PI;
        }
        while (retAngleRad > M_PI)
        {
            retAngleRad -= 2*M_PI;
        }

        return retAngleRad;
    }

    // calculate optimized rotation in euler angle (where we yaw the point first, then pitch, then roll)
    static double GetYawPitchRollFromQuaternion(const Eigen::Quaterniond& quat, double& yaw, double& pitch, double& roll)
    {
        Eigen::Quaterniond finalQ = quat;
        finalQ.normalize();
        auto euler = finalQ.toRotationMatrix().eulerAngles(0, 1, 2);

        yaw = finalQ.z();
        pitch = finalQ.y();
        roll = finalQ.x();
    }

};

}  // namespace DistributedFormation
