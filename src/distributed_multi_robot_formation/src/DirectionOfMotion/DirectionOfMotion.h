//
// Created by benson on 19/1/21.
//

#pragma once

#include <cfloat>
#include <unordered_map>
#include <vector>

#include "../Common/Common.h"

namespace DistributedFormation
{

class DirectionOfMotion
{
public:
    DirectionOfMotion();

    void SetStartingAngleAndDiscretizationParams(const double startingAzimuthAngleRad, const double startingElevationAngleRad,
                                                 const unsigned int numberOfAzimuthDiscreteAnglesOnASide, const double resolutionAzimuthAngleRad,
                                                 const unsigned int numberOfElevationDiscreteAnglesOnASide, const double resolutionElevationAngleRad);

    bool ConsensusOnDirection (const std::unordered_map<int32_t, Common::DirectionUtility>& inputDirectionUtility,
                               double& outputAzimuthAngleRad, double& outputElevationAngleRad);

    size_t NumberOfDiscretizedAngles();

    //index starts from the negative and the azimuth at the lowest elevation is filled first before going to the azimuth angles at the next elevation
    void IndexToAngle(const unsigned int index, double& azimuthAngleRad, double& elevationAngleRad);

    double UtilityForAngleIndex(const unsigned int index);

    void GetEndPointFromConsensusDirection(const double desiredExpansionAzimuthAngleRad,
                                           const double desiredExpansionElevationAngleRad,
                                           const Common::Position& startPoint,
                                           const double distanceBetweenStartAndEnd,
                                           Common::Position& endPoint);

private:
    double m_startingAzimuthAngleRad;
    double m_startingElevationAngleRad;
    unsigned int m_numberOfAzimuthDiscreteAnglesOnASide;
    double m_resolutionAzimuthAngleRad;
    unsigned int m_numberOfElevationDiscreteAnglesOnASide;
    double m_resolutionElevationAngleRad;

    unsigned int m_numberOfDiscretizedAngles;
};

}  // namespace DistributedFormation