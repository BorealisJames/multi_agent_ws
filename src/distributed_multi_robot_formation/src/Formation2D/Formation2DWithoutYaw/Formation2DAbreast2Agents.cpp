//
// Created by benson on 25/1/21.
//

#include "Formation2DAbreast2Agents.h"

namespace Formation2DWithoutYaw
{
    Formation2DAbreast2Agents::Formation2DAbreast2Agents()
    : Formation2DBase(2, DistributedFormation::Common::Formation2DType::ABREAST_FORMATION)
    {
    }

    void
    Formation2DAbreast2Agents::GetFormationPositions (const double d_x, const double d_y,
                                                      const double d_size,
                                                      Eigen::Matrix<double, Eigen::Dynamic, 1>& positions2DInFormation) const
    {
        positions2DInFormation.resize(4,1);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        positions2DInFormation(0,0) = m_desiredX + d_x + (l/2) * std::sin(m_desiredYaw );
        positions2DInFormation(1,0) = m_desiredY + d_y + (l/2) * std::cos(m_desiredYaw);
        positions2DInFormation(2,0) = m_desiredX + d_x - (l/2) * std::sin(m_desiredYaw);
        positions2DInFormation(3,0) = m_desiredY + d_y - (l/2) * std::cos(m_desiredYaw);
    }

    void
    Formation2DAbreast2Agents::GetFormationJacobian (const double d_x, const double d_y,
                                                     const double d_size,
                                                     Eigen::Matrix<double, Eigen::Dynamic, 3>& positions2DJacobian) const
    {
        positions2DJacobian.resize(4,3);

        //first row pt1x
        positions2DJacobian(0,0) = 1;
        positions2DJacobian(0,1) = 0;
        positions2DJacobian(0,2) = 0.5 * m_desiredDistance * std::sin(m_desiredYaw);

        //second row pt1y
        positions2DJacobian(1,0) = 0;
        positions2DJacobian(1,1) = 1;
        positions2DJacobian(1,2) = 0.5 * m_desiredDistance * std::cos(m_desiredYaw);

        //third row pt2x
        positions2DJacobian(2,0) = 1;
        positions2DJacobian(2,1) = 0;
        positions2DJacobian(2,2) = -0.5 * m_desiredDistance * std::sin(m_desiredYaw);

        //fourth row pt2y
        positions2DJacobian(3,0) = 0;
        positions2DJacobian(3,1) = 1;
        positions2DJacobian(3,2) = -0.5 * m_desiredDistance * std::cos(m_desiredYaw);
    }

}   // namespace Formation2DWithoutYaw