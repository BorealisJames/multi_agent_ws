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
        ABREAST_FORMATION = 3,
        POINT_FORMATION = 4
    };

    enum class Formation3DType
    {
        NO_FORMATION = 0,
        TRIANGLE_FORMATION = 1,
        LINE_FORMATION = 2,
        ABREAST_FORMATION = 3,
        POINT_FORMATION = 4
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

    struct DistributedFormationParameters
    {
        Common::WORKSPACE workspace;
        int64_t expiryDurationMicroSec;
        unsigned int numberOfAzimuthDiscreteAnglesOnASide;
        double resolutionAzimuthAngleRad;
        unsigned int numberOfElevationDiscreteAnglesOnASide;
        double resolutionElevationAngleRad;
        double distanceToFollowBehind;
        double localBoundingBoxForPathAlongX;
        double localBoundingBoxForPathAlongY;
        double localBoundingBoxForPathAlongZ;
        double pointRemovalRadius;
        double desiredDistanceInTriFormation;
        double desiredDistanceInLineFormation;
        double incrementOffsetToFormationYaw;
        double agentRadius;
        double waypointReachedBoundary;
        double weightForGoal;
        double weightForRotation;
        double weightForSize;
        double desiredHeight;
        double priorityPenalty;

        DistributedFormationParameters()
        : workspace(Common::WORKSPACE::DIM_3_WITH_ROT)
        , expiryDurationMicroSec(15*1000000)
        , numberOfAzimuthDiscreteAnglesOnASide(0)
        , resolutionAzimuthAngleRad(0.0)
        , numberOfElevationDiscreteAnglesOnASide(0)
        , resolutionElevationAngleRad(0.0)
        , distanceToFollowBehind(-1.0)
        , localBoundingBoxForPathAlongX(3.0)
        , localBoundingBoxForPathAlongY(3.0)
        , localBoundingBoxForPathAlongZ(2.0)
        , pointRemovalRadius(0.45)
        , desiredDistanceInTriFormation(2.5)
        , desiredDistanceInLineFormation(1.25)
        , incrementOffsetToFormationYaw(M_PI / 2.0)
        , agentRadius(0.3)
        , waypointReachedBoundary(1.5)
        , weightForGoal(0.3)
        , weightForRotation(0.3)
        , weightForSize(0.4)
        , desiredHeight(2.0)
        , priorityPenalty(1.0)
        {}
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
