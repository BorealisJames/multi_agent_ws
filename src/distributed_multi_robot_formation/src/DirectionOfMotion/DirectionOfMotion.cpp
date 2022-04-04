//
// Created by benson on 19/1/21.
//

#include "DirectionOfMotion.h"

namespace DistributedFormation
{
    DirectionOfMotion::DirectionOfMotion()
    : m_startingAzimuthAngleRad(0.0)
    , m_startingElevationAngleRad(0.0)
    , m_numberOfAzimuthDiscreteAnglesOnASide(0)
    , m_resolutionAzimuthAngleRad(0.0)
    , m_numberOfElevationDiscreteAnglesOnASide(0)
    , m_resolutionElevationAngleRad(0.0)
    , m_numberOfDiscretizedAngles(1)
    {

    }

    void
    DirectionOfMotion::SetStartingAngleAndDiscretizationParams(const double startingAzimuthAngleRad, const double startingElevationAngleRad,
                                                               const unsigned int numberOfAzimuthDiscreteAnglesOnASide, const double resolutionAzimuthAngleRad,
                                                               const unsigned int numberOfElevationDiscreteAnglesOnASide, const double resolutionElevationAngleRad)
    {
        m_startingAzimuthAngleRad = Common::MinusPiToPi(startingAzimuthAngleRad);
        m_startingElevationAngleRad = Common::MinusPiToPi(startingElevationAngleRad);
        m_numberOfAzimuthDiscreteAnglesOnASide = numberOfAzimuthDiscreteAnglesOnASide;
        m_resolutionAzimuthAngleRad = std::abs(Common::MinusPiToPi(resolutionAzimuthAngleRad));
        m_numberOfElevationDiscreteAnglesOnASide = numberOfElevationDiscreteAnglesOnASide;
        m_resolutionElevationAngleRad = std::abs(Common::MinusPiToPi(resolutionElevationAngleRad));

        m_numberOfDiscretizedAngles = numberOfAzimuthDiscreteAnglesOnASide*2 + 1;
        m_numberOfDiscretizedAngles = (numberOfElevationDiscreteAnglesOnASide*2 + 1) * m_numberOfDiscretizedAngles;
    }

    bool
    DirectionOfMotion::ConsensusOnDirection (const std::unordered_map<int32_t, Common::DirectionUtility>& inputDirectionUtility,
                                             double& outputAzimuthAngleRad, double& outputElevationAngleRad)
    {
        for (auto&& directionUtilityInstance : inputDirectionUtility)
        {
            if (directionUtilityInstance.second.angleIndexAndUtility.size() != m_numberOfDiscretizedAngles)
            {
                return false;
            }
        }

        std::vector<double> minDirectionUtility;
        for (int i = 0; i < m_numberOfDiscretizedAngles; i++)
        {
            double minUtility = DBL_MAX;
            for (auto&& directionUtilityInstance : inputDirectionUtility)
            {
                if (directionUtilityInstance.second.angleIndexAndUtility.at(i) < minUtility)
                {
                    minUtility = directionUtilityInstance.second.angleIndexAndUtility.at(i);
                }
            }

            minDirectionUtility.push_back(minUtility);
        }

        double maxUtility = -DBL_MAX;
        size_t index = 0;
        for (int i = 0; i < minDirectionUtility.size(); i++)
        {
            if (minDirectionUtility.at(i) > maxUtility)
            {
                maxUtility = minDirectionUtility.at(i);
                index = i;
            }
        }

        IndexToAngle(index, outputAzimuthAngleRad, outputElevationAngleRad);

        return true;
    }

    size_t
    DirectionOfMotion::NumberOfDiscretizedAngles()
    {
        return m_numberOfDiscretizedAngles;
    }

    void
    DirectionOfMotion::IndexToAngle(const unsigned int index, double& azimuthAngleRad, double& elevationAngleRad)
    {
        unsigned int indexValidated = std::min(m_numberOfDiscretizedAngles, index);

        unsigned int numberOfAzimuthLevels = (m_numberOfAzimuthDiscreteAnglesOnASide*2 + 1);

        unsigned int elevationLevelIndex = indexValidated/numberOfAzimuthLevels;
        unsigned int azimuthLevelIndex = indexValidated%numberOfAzimuthLevels;

        elevationAngleRad = m_startingElevationAngleRad +
                            static_cast<int>(elevationLevelIndex - m_numberOfElevationDiscreteAnglesOnASide)*m_resolutionElevationAngleRad;
        elevationAngleRad = Common::MinusPiToPi(elevationAngleRad);

        azimuthAngleRad = m_startingAzimuthAngleRad +
                          static_cast<int>(azimuthLevelIndex - m_numberOfAzimuthDiscreteAnglesOnASide)*m_resolutionAzimuthAngleRad;
        azimuthAngleRad = Common::MinusPiToPi(azimuthAngleRad);
    }

    double
    DirectionOfMotion::UtilityForAngleIndex(const unsigned int index)
    {
        double utility = 0.0;

        double azimuthAngleRad, elevationAngleRad;
        IndexToAngle(index, azimuthAngleRad, elevationAngleRad);

        double angleAzimuthDelta = Common::MinusPiToPi(azimuthAngleRad) - Common::MinusPiToPi(m_startingAzimuthAngleRad);
        angleAzimuthDelta = Common::MinusPiToPi(angleAzimuthDelta);

        double angleElevationDelta = Common::MinusPiToPi(elevationAngleRad) - Common::MinusPiToPi(m_startingElevationAngleRad);
        angleElevationDelta = Common::MinusPiToPi(angleElevationDelta);

        if (angleAzimuthDelta < std::numeric_limits<double>::epsilon() &&
            angleElevationDelta < std::numeric_limits<double>::epsilon())
        {
            utility = 1.0;
        }

        return utility;
    }

    void
    DirectionOfMotion::GetEndPointFromConsensusDirection(const double desiredExpansionAzimuthAngleRad,
                                                           const double desiredExpansionElevationAngleRad,
                                                           const Common::Position& startPoint,
                                                           const double distanceBetweenStartAndEnd,
                                                           Common::Position& endPoint)
    {
        endPoint.x = distanceBetweenStartAndEnd * (std::cos( desiredExpansionElevationAngleRad ) * std::cos( desiredExpansionAzimuthAngleRad )) +
                startPoint.x;
        endPoint.y = distanceBetweenStartAndEnd * (std::cos( desiredExpansionElevationAngleRad ) * std::sin( desiredExpansionAzimuthAngleRad )) +
                startPoint.y;
        endPoint.z = distanceBetweenStartAndEnd * std::sin( desiredExpansionElevationAngleRad ) +
                startPoint.z;
    }

}  // namespace DistributedFormation