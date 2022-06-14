//
// Created by benson on 5/8/21.
//

#pragma once

#include <Eigen/Core>
#include <memory>

#include "../../Common/Common.h"

namespace Formation3DWithOnlyYaw
{

class Formation3DBase
{
public:
    typedef std::shared_ptr<Formation3DBase> Ptr;

    Formation3DBase(uint32_t numberOfAgents, DistributedFormation::Common::Formation3DType formationType)
    : m_numberOfAgents(numberOfAgents)
    , m_formationType(formationType)
    , m_desiredDistance(1.5)
    , m_desiredX(0)
    , m_desiredY(0)
    , m_desiredZ(0)
    , m_desiredQw(1)
    , m_desiredQx(0)
    , m_desiredQy(0)
    , m_desiredQz(0)
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

    void SetDesiredPositionRotationAndSize (const double& desiredX, const double& desiredY, const double& desiredZ,
                                         const double& desiredQw, const double& desiredQx, const double& desiredQy, const double& desiredQz,
                                         const double& desiredSize)
    {
        m_desiredX = desiredX;
        m_desiredY = desiredY;
        m_desiredZ = desiredZ;

        Eigen::Vector4d desiredQ (desiredQw, desiredQx, desiredQy, desiredQz);
        desiredQ.normalize();
        m_desiredQw = desiredQ(0);
        m_desiredQx = desiredQ(1);
        m_desiredQy = desiredQ(2);
        m_desiredQz = desiredQ(3);

        m_desiredSize = desiredSize;
    }

    DistributedFormation::Common::Formation3DType GetFormationType () const
    {
        return m_formationType;
    }

    double GetNumberOfAgents () const
    {
        return m_numberOfAgents;
    }

    //roll first then pitch then yaw
    virtual void GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                        const double d_yaw,
                                        const double d_size,
                                        Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const = 0;

    virtual void GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                                       const double d_yaw,
                                       const double d_size,
                                       Eigen::Matrix<double, Eigen::Dynamic, 5>& positions3DJacobian) const = 0;


protected:

    uint32_t m_numberOfAgents;
    DistributedFormation::Common::Formation3DType m_formationType;
    double m_desiredDistance;
    double m_desiredX, m_desiredY, m_desiredZ, m_desiredQw, m_desiredQx, m_desiredQy, m_desiredQz, m_desiredSize;

};

}   // namespace Formation3DWithOnlyYaw



