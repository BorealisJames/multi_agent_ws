//
// Created by benson on 13/1/21.
//


#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <math.h>
#include <vector>

namespace DistributedGlobalPathPlanner
{

class Common
{


public:

    enum class DIMENSION
    {
        DIM_2 = 2,
        DIM_3 = 3
    };

    enum class PHASE
    {
        PHASE_1 = 1,   //PHASE_1 is convex hull of robot position
        PHASE_2 = 2,
        PHASE_3 = 3,
        PHASE_4 = 4,
        PHASE_5 = 5
    };

    struct PhaseAndTime
    {
        PHASE phase;
        int64_t timeMicroSecs;
    };

    struct Pose
    {
        Eigen::Vector3d position;
        double headingRad;
    };

    struct PathAndCost
    {
        std::vector<Eigen::Vector3d> positions;
        double cost;
    };

    struct PathAndWaypointProgress
    {
        std::vector<Pose> poses;
        int waypointProgress;
    };

    struct DistributedGlobalPathParams
    {
        Common::DIMENSION dimension;
        int64_t expiryDurationMicroSec;
        double pointRemovalRadius;
        double agentRadius;
        double waypointReachedBoundary;
        double desiredHeight;

        DistributedGlobalPathParams()
        : dimension(Common::DIMENSION::DIM_2)
        , expiryDurationMicroSec(15*1000000)
        , pointRemovalRadius(0.45)
        , agentRadius(0.3)
        , waypointReachedBoundary(1.5)
        , desiredHeight(2.0)
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
};

}  // namespace DistributedFormation
