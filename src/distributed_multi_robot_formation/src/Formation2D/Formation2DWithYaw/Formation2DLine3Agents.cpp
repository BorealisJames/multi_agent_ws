//
// Created by benson on 25/1/21.
//

#include "Formation2DLine3Agents.h"

namespace Formation2DWithYaw
{
    Formation2DLine3Agents::Formation2DLine3Agents()
    : Formation2DBase()
    {
        m_numberOfAgents = 3;
        m_formationType = DistributedFormation::Common::Formation2DType::LINE_FORMATION;
    }

    void
    Formation2DLine3Agents::GetFormationPositions (const double d_x, const double d_y,
                                                   const double d_yaw,
                                                   const double d_size,
                                                   Eigen::Matrix<double, Eigen::Dynamic, 1>& positions2DInFormation) const
    {
        positions2DInFormation.resize(6,1);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        positions2DInFormation(0,0) = m_desiredX + d_x + l * std::cos(m_desiredYaw + d_yaw);
        positions2DInFormation(1,0) = m_desiredY + d_y + l * std::sin(m_desiredYaw + d_yaw);
        positions2DInFormation(2,0) = m_desiredX + d_x;
        positions2DInFormation(3,0) = m_desiredY + d_y;
        positions2DInFormation(4,0) = m_desiredX + d_x - l * std::cos(m_desiredYaw + d_yaw);
        positions2DInFormation(5,0) = m_desiredY + d_y - l * std::sin(m_desiredYaw + d_yaw);
    }

    void
    Formation2DLine3Agents::GetFormationJacobian (const double d_x, const double d_y,
                                                  const double d_yaw,
                                                  const double d_size,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 4>& positions2DJacobian) const
    {
        positions2DJacobian.resize(6,4);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        //first row pt1x
        positions2DJacobian(0,0) = 1;
        positions2DJacobian(0,1) = 0;
        positions2DJacobian(0,2) = -l * std::sin(m_desiredYaw + d_yaw);
        positions2DJacobian(0,3) = m_desiredDistance * std::cos(m_desiredYaw + d_yaw);

        //second row pt1y
        positions2DJacobian(1,0) = 0;
        positions2DJacobian(1,1) = 1;
        positions2DJacobian(1,2) = l * std::cos(m_desiredYaw + d_yaw);
        positions2DJacobian(1,3) = m_desiredDistance * std::sin(m_desiredYaw + d_yaw);

        //third row pt2x
        positions2DJacobian(2,0) = 1;
        positions2DJacobian(2,1) = 0;
        positions2DJacobian(2,2) = 0;
        positions2DJacobian(2,3) = 0;

        //fourth row pt2y
        positions2DJacobian(3,0) = 0;
        positions2DJacobian(3,1) = 1;
        positions2DJacobian(3,2) = 0;
        positions2DJacobian(3,3) = 0;

        //fifth row pt3x
        positions2DJacobian(4,0) = 1;
        positions2DJacobian(4,1) = 0;
        positions2DJacobian(4,2) = l * std::sin(m_desiredYaw + d_yaw);
        positions2DJacobian(4,3) = -m_desiredDistance * std::cos(m_desiredYaw + d_yaw);

        //sixth row pt3x
        positions2DJacobian(5,0) = 0;
        positions2DJacobian(5,1) = 1;
        positions2DJacobian(5,2) = -l * std::cos(m_desiredYaw + d_yaw);
        positions2DJacobian(5,3) = -m_desiredDistance * std::sin(m_desiredYaw + d_yaw);
    }

    void
    Formation2DLine3Agents::SetNumberOfAgents ()
    {
        m_numberOfAgents = 3;
    }

}   // namespace Formation2DWithYaw