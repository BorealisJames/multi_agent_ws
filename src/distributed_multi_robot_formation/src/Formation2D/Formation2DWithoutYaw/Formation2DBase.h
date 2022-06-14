//
// Created by benson on 26/1/21.
//

#pragma once

#include <Eigen/Core>
#include <memory>

#include "../../Common/Common.h"

namespace Formation2DWithoutYaw
{

class Formation2DBase
{
public:
    typedef std::shared_ptr<Formation2DBase> Ptr;

    Formation2DBase(uint32_t numberOfAgents, DistributedFormation::Common::Formation2DType formationType)
    : m_numberOfAgents(numberOfAgents)
    , m_formationType(formationType)
    , m_desiredDistance(1.5)
    , m_desiredX(0)
    , m_desiredY(0)
    , m_desiredYaw(0)
    , m_desiredSize(1)
    {
    }

    void SetDesiredDistanceBetweenAgents (const double& desiredDistance)
    {
        m_desiredDistance = desiredDistance;
    }
    double GetDesiredDistanceBetweenAgents () const
    {
        return m_desiredDistance;
    }

    void SetDesiredPositionYawAndSize (const double& desiredX, const double& desiredY,
                                    const double& desiredYaw,
                                    const double& desiredSize)
    {
        m_desiredX = desiredX;
        m_desiredY = desiredY;
        m_desiredYaw = desiredYaw;
        m_desiredSize = desiredSize;
    }

    DistributedFormation::Common::Formation2DType GetFormationType () const
    {
        return m_formationType;
    }

    double GetNumberOfAgents () const
    {
        return m_numberOfAgents;
    }

    virtual void GetFormationPositions (const double d_x, const double d_y,
                                        const double d_size,
                                        Eigen::Matrix<double, Eigen::Dynamic, 1>& positions2DInFormation) const = 0;

    virtual void GetFormationJacobian (const double d_x, const double d_y,
                                       const double d_size,
                                       Eigen::Matrix<double, Eigen::Dynamic, 3>& positions2DJacobian) const = 0;


protected:

    uint32_t m_numberOfAgents;
    DistributedFormation::Common::Formation2DType m_formationType;
    double m_desiredDistance;
    double m_desiredX, m_desiredY, m_desiredYaw, m_desiredSize;
};

}   // namespace Formation2DWithoutYaw



