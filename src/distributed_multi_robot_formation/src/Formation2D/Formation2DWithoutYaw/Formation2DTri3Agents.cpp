//
// Created by benson on 25/1/21.
//

#include "Formation2DTri3Agents.h"

namespace Formation2DWithoutYaw
{

    Formation2DTri3Agents::Formation2DTri3Agents()
    : Formation2DBase()
    {
        m_numberOfAgents = 3;
        m_formationType = DistributedFormation::Common::Formation2DType::TRIANGLE_FORMATION;
    }

    void
    Formation2DTri3Agents::GetFormationPositions (const double d_x, const double d_y,
                                                  const double d_size,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 1>& positions2DInFormation) const
    {
        positions2DInFormation.resize(6,1);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        positions2DInFormation(0,0) = m_desiredX + d_x + l * k_oneOverSqrtOfThree * std::cos(m_desiredYaw);
        positions2DInFormation(1,0) = m_desiredY + d_y + l * k_oneOverSqrtOfThree * std::sin(m_desiredYaw);
        positions2DInFormation(2,0) = m_desiredX + d_x + l * k_oneOverSqrtOfThree * std::cos(m_desiredYaw + 2.0*M_PI/3.0);
        positions2DInFormation(3,0) = m_desiredY + d_y + l * k_oneOverSqrtOfThree * std::sin(m_desiredYaw + 2.0*M_PI/3.0);
        positions2DInFormation(4,0) = m_desiredX + d_x + l * k_oneOverSqrtOfThree * std::cos(m_desiredYaw - 2.0*M_PI/3.0);
        positions2DInFormation(5,0) = m_desiredY + d_y + l * k_oneOverSqrtOfThree * std::sin(m_desiredYaw - 2.0*M_PI/3.0);
    }

    void
    Formation2DTri3Agents::GetFormationJacobian (const double d_x, const double d_y,
                                                 const double d_size,
                                                 Eigen::Matrix<double, Eigen::Dynamic, 3>& positions2DJacobian) const
    {
        positions2DJacobian.resize(6,3);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        //first row pt1x
        positions2DJacobian(0,0) = 1;
        positions2DJacobian(0,1) = 0;
        positions2DJacobian(0,2) =  m_desiredDistance * k_oneOverSqrtOfThree * std::cos(m_desiredYaw);

        //second row pt1y
        positions2DJacobian(1,0) = 0;
        positions2DJacobian(1,1) = 1;
        positions2DJacobian(1,2) = m_desiredDistance * k_oneOverSqrtOfThree * std::sin(m_desiredYaw);

        //third row pt2x
        positions2DJacobian(2,0) = 1;
        positions2DJacobian(2,1) = 0;
        positions2DJacobian(2,2) = m_desiredDistance * k_oneOverSqrtOfThree * std::cos(m_desiredYaw + 2.0*M_PI/3.0);

        //fourth row pt2y
        positions2DJacobian(3,0) = 0;
        positions2DJacobian(3,1) = 1;
        positions2DJacobian(3,2) = m_desiredDistance * k_oneOverSqrtOfThree * std::sin(m_desiredYaw + 2.0*M_PI/3.0);

        //fifth row pt3x
        positions2DJacobian(4,0) = 1;
        positions2DJacobian(4,1) = 0;
        positions2DJacobian(4,2) = m_desiredDistance * k_oneOverSqrtOfThree * std::cos(m_desiredYaw - 2.0*M_PI/3.0);

        //sixth row pt3y
        positions2DJacobian(5,0) = 0;
        positions2DJacobian(5,1) = 1;
        positions2DJacobian(5,2) = m_desiredDistance * k_oneOverSqrtOfThree * std::sin(m_desiredYaw - 2.0*M_PI/3.0);
    }

    void
    Formation2DTri3Agents::SetNumberOfAgents ()
    {
        m_numberOfAgents = 3;
    }

}   // namespace Formation2DWithoutYaw