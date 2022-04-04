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

    Formation2DBase()
    : m_desiredDistance(1.5)
    , m_numberOfAgents(3)
    , m_desiredX(0)
    , m_desiredY(0)
    , m_desiredYaw(0)
    , m_desiredSize(1)
    , m_formationType(DistributedFormation::Common::Formation2DType::NO_FORMATION)
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
    virtual void SetNumberOfAgents () = 0;

    double m_desiredDistance;
    uint32_t m_numberOfAgents;
    double m_desiredX, m_desiredY, m_desiredYaw, m_desiredSize;
    DistributedFormation::Common::Formation2DType m_formationType;
};

}   // namespace Formation2DWithoutYaw



