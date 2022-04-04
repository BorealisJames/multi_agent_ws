//
// Created by benson on 28/1/21.
//

#include <gtest/gtest.h>

#include "../Common/Common.h"
#include "../DirectionOfMotion/DirectionOfMotion.h"

namespace DistributedFormation
{
    TEST (DirectionOfMotion, NumberOfDiscretizedAngles)
    {
        const double startingAzimuthAngleRad = 0.5;
        const double startingElevationAngleRad = 0.5;
        const unsigned int numberOfAzimuthDiscreteAnglesOnASide = 2;
        const double resolutionAzimuthAngleRad = 0.1;
        const unsigned int numberOfElevationDiscreteAnglesOnASide = 2;
        const double resolutionElevationAngleRad = 0.1;

        DirectionOfMotion directionOfMotion;
        directionOfMotion.SetStartingAngleAndDiscretizationParams(startingAzimuthAngleRad, startingElevationAngleRad,
                                                                  numberOfAzimuthDiscreteAnglesOnASide,
                                                                  resolutionAzimuthAngleRad,
                                                                  numberOfElevationDiscreteAnglesOnASide,
                                                                  resolutionElevationAngleRad);

        EXPECT_EQ(25, directionOfMotion.NumberOfDiscretizedAngles());
    }

    TEST (DirectionOfMotion, IndexToAngle)
    {
        const double startingAzimuthAngleRad = 0.5;
        const double startingElevationAngleRad = 0.5;
        const unsigned int numberOfAzimuthDiscreteAnglesOnASide = 2;
        const double resolutionAzimuthAngleRad = 0.1;
        const unsigned int numberOfElevationDiscreteAnglesOnASide = 2;
        const double resolutionElevationAngleRad = 0.1;

        DirectionOfMotion directionOfMotion;
        directionOfMotion.SetStartingAngleAndDiscretizationParams(startingAzimuthAngleRad, startingElevationAngleRad,
                                                                  numberOfAzimuthDiscreteAnglesOnASide,
                                                                  resolutionAzimuthAngleRad,
                                                                  numberOfElevationDiscreteAnglesOnASide,
                                                                  resolutionElevationAngleRad);

        double azimuthAngleRad;
        double elevationAngleRad;
        directionOfMotion.IndexToAngle(0, azimuthAngleRad, elevationAngleRad);

        EXPECT_EQ(0.5-(2*0.1), azimuthAngleRad);
        EXPECT_EQ(0.5-(2*0.1), elevationAngleRad);
    }

    TEST (DirectionOfMotion, ConsensusOnDirection)
    {
        const double startingAzimuthAngleRad = 0.5;
        const double startingElevationAngleRad = 0.5;
        const unsigned int numberOfAzimuthDiscreteAnglesOnASide = 1;
        const double resolutionAzimuthAngleRad = 0.1;
        const unsigned int numberOfElevationDiscreteAnglesOnASide = 0;
        const double resolutionElevationAngleRad = 0.1;

        DirectionOfMotion directionOfMotion;
        directionOfMotion.SetStartingAngleAndDiscretizationParams(startingAzimuthAngleRad, startingElevationAngleRad,
                                                                  numberOfAzimuthDiscreteAnglesOnASide,
                                                                  resolutionAzimuthAngleRad,
                                                                  numberOfElevationDiscreteAnglesOnASide,
                                                                  resolutionElevationAngleRad);


        std::unordered_map<int32_t, Common::DirectionUtility> inputDirectionUtility;
        Common::DirectionUtility utilityForAgent1;
        utilityForAgent1.angleIndexAndUtility.push_back(2.0);
        utilityForAgent1.angleIndexAndUtility.push_back(1.0);
        utilityForAgent1.angleIndexAndUtility.push_back(3.0);
        inputDirectionUtility[1] = utilityForAgent1;
        Common::DirectionUtility utilityForAgent2;
        utilityForAgent2.angleIndexAndUtility.push_back(0.8);
        utilityForAgent2.angleIndexAndUtility.push_back(1.5);
        utilityForAgent2.angleIndexAndUtility.push_back(2.0);
        inputDirectionUtility[2] = utilityForAgent2;
        double outputConsensusDirection;

        double outputAzimuthAngleRad;
        double outputElevationAngleRad;
        double success = directionOfMotion.ConsensusOnDirection(inputDirectionUtility,
                                                                outputAzimuthAngleRad, outputElevationAngleRad);

        EXPECT_TRUE(success);

        EXPECT_DOUBLE_EQ(Common::MinusPiToPi(0.5+0.1), outputAzimuthAngleRad);
        EXPECT_DOUBLE_EQ(Common::MinusPiToPi(0.5), outputElevationAngleRad);
    }

    TEST (DirectionOfMotion, GetEndPointFromConsensusDirection)
    {
        double desiredExpansionAzimuthAngleRad = -45*M_PI/180;
        double desiredExpansionElevationAngleRad = -45*M_PI/180;
        Common::Position startPoint;
        startPoint.x = 1;
        startPoint.y = 1;
        startPoint.z = 1;
        double distanceBetweenStartAndEnd = 2.0;
        Common::Position endPoint;

        DirectionOfMotion directionOfMotion;
        directionOfMotion.GetEndPointFromConsensusDirection(desiredExpansionAzimuthAngleRad, desiredExpansionElevationAngleRad,
                                                            startPoint, distanceBetweenStartAndEnd,
                                                            endPoint);

        double deltaX = endPoint.x - startPoint.x;
        double deltaY = endPoint.y - startPoint.y;
        double deltaZ = endPoint.z - startPoint.z;
        double magnitude = std::sqrt (deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);

        double outputAzimuthAngleRadForEndPt = std::atan2(deltaY, deltaX);
        double outputElevationAngleRadForEndPt = std::asin(deltaZ / magnitude);

        EXPECT_DOUBLE_EQ(Common::MinusPiToPi(desiredExpansionAzimuthAngleRad), outputAzimuthAngleRadForEndPt);
        EXPECT_DOUBLE_EQ(Common::MinusPiToPi(desiredExpansionElevationAngleRad), outputElevationAngleRadForEndPt);

    }
} // namespace DistributedFormation