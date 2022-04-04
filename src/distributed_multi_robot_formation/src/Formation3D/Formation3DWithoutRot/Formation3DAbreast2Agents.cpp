//
// Created by benson on 5/8/21.
//

#include "Formation3DAbreast2Agents.h"

namespace Formation3DWithoutRot
{
    Formation3DAbreast2Agents::Formation3DAbreast2Agents()
    : Formation3DBase()
    {
        m_numberOfAgents = 2;
        m_formationType = DistributedFormation::Common::Formation3DType::ABREAST_FORMATION;
    }

    void
    Formation3DAbreast2Agents::GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                                      const double d_size,
                                                      Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const
    {
        positions3DInFormation.resize(6,1);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        //desiredQ and deltaQ are assumed to be unit quaternions which has been normalized before passing in
        Eigen::Quaterniond desiredQ (m_desiredQw, m_desiredQx, m_desiredQy, m_desiredQz);

        Eigen::Quaterniond pt1 (0, 0, l/2, 0);
        Eigen::Quaterniond pt1_rotated = desiredQ*pt1*desiredQ.conjugate();

        Eigen::Quaterniond pt2 (0, 0, -l/2, 0);
        Eigen::Quaterniond pt2_rotated = desiredQ*pt2*desiredQ.conjugate();

        positions3DInFormation(0,0) = m_desiredX + d_x + pt1_rotated.x();
        positions3DInFormation(1,0) = m_desiredY + d_y + pt1_rotated.y();
        positions3DInFormation(2,0) = m_desiredZ + d_z + pt1_rotated.z();
        positions3DInFormation(3,0) = m_desiredX + d_x + pt2_rotated.x();
        positions3DInFormation(4,0) = m_desiredY + d_y + pt2_rotated.y();
        positions3DInFormation(5,0) = m_desiredZ + d_z + pt2_rotated.z();
    }

    void
    Formation3DAbreast2Agents::GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                                                     const double d_size,
                                                     Eigen::Matrix<double, Eigen::Dynamic, 4>& positions3DJacobian) const
    {
        positions3DJacobian.resize(6,4);

        //1st row pt1x
        positions3DJacobian(0,0) = 1;
        positions3DJacobian(0,1) = 0;
        positions3DJacobian(0,2) = 0;
        positions3DJacobian(0,3) = -m_desiredDistance*m_desiredQw*m_desiredQz + m_desiredDistance*m_desiredQx*m_desiredQy;

        //2nd row pt1y
        positions3DJacobian(1,0) = 0;
        positions3DJacobian(1,1) = 1;
        positions3DJacobian(1,2) = 0;
        positions3DJacobian(1,3) = (1.0/2.0)*m_desiredDistance*pow(m_desiredQw, 2) - 1.0/2.0*m_desiredDistance*pow(m_desiredQx, 2) + (1.0/2.0)*m_desiredDistance*pow(m_desiredQy, 2) - 1.0/2.0*m_desiredDistance*pow(m_desiredQz, 2);

        //3rd row pt1z
        positions3DJacobian(2,0) = 0;
        positions3DJacobian(2,1) = 0;
        positions3DJacobian(2,2) = 1;
        positions3DJacobian(2,3) = m_desiredDistance*m_desiredQw*m_desiredQx + m_desiredDistance*m_desiredQy*m_desiredQz;


        //4th row pt2x
        positions3DJacobian(3,0) = 1;
        positions3DJacobian(3,1) = 0;
        positions3DJacobian(3,2) = 0;
        positions3DJacobian(3,3) = m_desiredDistance*m_desiredQw*m_desiredQz - m_desiredDistance*m_desiredQx*m_desiredQy;

        //5th row pt2y
        positions3DJacobian(4,0) = 0;
        positions3DJacobian(4,1) = 1;
        positions3DJacobian(4,2) = 0;
        positions3DJacobian(4,3) = -1.0/2.0*m_desiredDistance*pow(m_desiredQw, 2) + (1.0/2.0)*m_desiredDistance*pow(m_desiredQx, 2) - 1.0/2.0*m_desiredDistance*pow(m_desiredQy, 2) + (1.0/2.0)*m_desiredDistance*pow(m_desiredQz, 2);

        //6th row pt2z
        positions3DJacobian(5,0) = 0;
        positions3DJacobian(5,1) = 0;
        positions3DJacobian(5,2) = 1;
        positions3DJacobian(5,3) = -m_desiredDistance*m_desiredQw*m_desiredQx - m_desiredDistance*m_desiredQy*m_desiredQz;
    }

    void
    Formation3DAbreast2Agents::SetNumberOfAgents ()
    {
        m_numberOfAgents = 2;
    }

}   // namespace Formation3DWithoutRot