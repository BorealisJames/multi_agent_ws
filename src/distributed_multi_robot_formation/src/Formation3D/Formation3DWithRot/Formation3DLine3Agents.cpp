//
// Created by benson on 5/8/21.
//

#include "Formation3DLine3Agents.h"

namespace Formation3DWithRot
{
    Formation3DLine3Agents::Formation3DLine3Agents()
    : Formation3DBase()
    {
        m_numberOfAgents = 3;
        m_formationType = DistributedFormation::Common::Formation3DType::LINE_FORMATION;
    }

    void
    Formation3DLine3Agents::GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                                   const double d_qw, const double d_qx, const double d_qy, const double d_qz,
                                                   const double d_size,
                                                   Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const
    {
        positions3DInFormation.resize(9,1);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        Eigen::Quaterniond newQ (m_desiredQw+d_qw, m_desiredQx+d_qx, m_desiredQy+d_qy, m_desiredQz+d_qz);
        newQ.normalize();

        Eigen::Quaterniond pt1 (0, l, 0, 0);
        Eigen::Quaterniond pt1_rotated = newQ*pt1*newQ.conjugate();

        Eigen::Quaterniond pt3 (0, -l, 0, 0);
        Eigen::Quaterniond pt3_rotated = newQ*pt3*newQ.conjugate();

        positions3DInFormation(0,0) = m_desiredX + d_x + pt1_rotated.x();
        positions3DInFormation(1,0) = m_desiredY + d_y + pt1_rotated.y();
        positions3DInFormation(2,0) = m_desiredZ + d_z + pt1_rotated.z();
        positions3DInFormation(3,0) = m_desiredX + d_x;
        positions3DInFormation(4,0) = m_desiredY + d_y;
        positions3DInFormation(5,0) = m_desiredZ + d_z;
        positions3DInFormation(6,0) = m_desiredX + d_x + pt3_rotated.x();
        positions3DInFormation(7,0) = m_desiredY + d_y + pt3_rotated.y();
        positions3DInFormation(8,0) = m_desiredZ + d_z + pt3_rotated.z();
    }

    void
    Formation3DLine3Agents::GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                                                  const double d_qw, const double d_qx, const double d_qy, const double d_qz,
                                                  const double d_size,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 8>& positions3DJacobian) const
    {
        positions3DJacobian.resize(9,8);

        //1st row pt1x
        positions3DJacobian(0,0) = 1;
        positions3DJacobian(0,1) = 0;
        positions3DJacobian(0,2) = 0;
        positions3DJacobian(0,3) = m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qw + m_desiredQw, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qx + m_desiredQx, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qy + m_desiredQy, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(2*d_qw + 2*m_desiredQw)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(0,4) = m_desiredDistance*pow(d_qw + m_desiredQw, 2)*(-2*d_qx - 2*m_desiredQx)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*pow(d_qx + m_desiredQx, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*pow(d_qy + m_desiredQy, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(2*d_qx + 2*m_desiredQx)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(0,5) = m_desiredDistance*pow(d_qw + m_desiredQw, 2)*(-2*d_qy - 2*m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*pow(d_qx + m_desiredQx, 2)*(-2*d_qy - 2*m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qy - 2*m_desiredQy)*pow(d_qy + m_desiredQy, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qy - 2*m_desiredQy)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(2*d_qy + 2*m_desiredQy)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(0,6) = m_desiredDistance*pow(d_qw + m_desiredQw, 2)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*pow(d_qx + m_desiredQx, 2)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*pow(d_qy + m_desiredQy, 2)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qz - 2*m_desiredQz)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(2*d_qz + 2*m_desiredQz)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(0,7) = m_desiredDistance*pow(d_qw + m_desiredQw, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) + m_desiredDistance*pow(d_qx + m_desiredQx, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) - m_desiredDistance*pow(d_qy + m_desiredQy, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) - m_desiredDistance*pow(d_qz + m_desiredQz, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));

        //2nd row pt1y
        positions3DJacobian(1,0) = 0;
        positions3DJacobian(1,1) = 1;
        positions3DJacobian(1,2) = 0;
        positions3DJacobian(1,3) = 2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qw + m_desiredQw)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(1,4) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qx - 2*m_desiredQx)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(1,5) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qy - 2*m_desiredQy)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(-2*d_qy - 2*m_desiredQy)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(1,6) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qz - 2*m_desiredQz)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qw + m_desiredQw)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2);
        positions3DJacobian(1,7) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(d_qz + m_desiredQz)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));

        //3rd row pt1z
        positions3DJacobian(2,0) = 0;
        positions3DJacobian(2,1) = 0;
        positions3DJacobian(2,2) = 1;
        positions3DJacobian(2,3) = -2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qw + m_desiredQw)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qx + m_desiredQx)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(2,4) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qx - 2*m_desiredQx)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*(d_qx + m_desiredQx)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(2,5) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qy - 2*m_desiredQy)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qw + m_desiredQw)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(-2*d_qy - 2*m_desiredQy)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2);
        positions3DJacobian(2,6) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(d_qy + m_desiredQy)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(-2*d_qz - 2*m_desiredQz)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(2,7) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(d_qy + m_desiredQy)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) + 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_qz + m_desiredQz)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));


        //4th row pt2x
        positions3DJacobian(3,0) = 1;
        positions3DJacobian(3,1) = 0;
        positions3DJacobian(3,2) = 0;
        positions3DJacobian(3,3) = 0;
        positions3DJacobian(3,4) = 0;
        positions3DJacobian(3,5) = 0;
        positions3DJacobian(3,6) = 0;
        positions3DJacobian(3,7) = 0;

        //5th row pt2y
        positions3DJacobian(4,0) = 0;
        positions3DJacobian(4,1) = 1;
        positions3DJacobian(4,2) = 0;
        positions3DJacobian(4,3) = 0;
        positions3DJacobian(4,4) = 0;
        positions3DJacobian(4,5) = 0;
        positions3DJacobian(4,6) = 0;
        positions3DJacobian(4,7) = 0;

        //6th row pt2z
        positions3DJacobian(5,0) = 0;
        positions3DJacobian(5,1) = 0;
        positions3DJacobian(5,2) = 1;
        positions3DJacobian(5,3) = 0;
        positions3DJacobian(5,4) = 0;
        positions3DJacobian(5,5) = 0;
        positions3DJacobian(5,6) = 0;
        positions3DJacobian(5,7) = 0;


        //7th row pt3x
        positions3DJacobian(6,0) = 1;
        positions3DJacobian(6,1) = 0;
        positions3DJacobian(6,2) = 0;
        positions3DJacobian(6,3) = -m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qw + m_desiredQw, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy +m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qx + m_desiredQx, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qy + m_desiredQy, 2)*(d_size +m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(2*d_qw + 2*m_desiredQw)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(6,4) = -m_desiredDistance*pow(d_qw + m_desiredQw, 2)*(-2*d_qx - 2*m_desiredQx)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy +m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*pow(d_qx + m_desiredQx, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*pow(d_qy + m_desiredQy, 2)*(d_size +m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*(2*d_qx + 2*m_desiredQx)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(6,5) = -m_desiredDistance*pow(d_qw + m_desiredQw, 2)*(-2*d_qy - 2*m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy +m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*pow(d_qx + m_desiredQx, 2)*(-2*d_qy - 2*m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qy - 2*m_desiredQy)*pow(d_qy + m_desiredQy, 2)*(d_size +m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qy - 2*m_desiredQy)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(2*d_qy + 2*m_desiredQy)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(6,6) = -m_desiredDistance*pow(d_qw + m_desiredQw, 2)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy +m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - m_desiredDistance*pow(d_qx + m_desiredQx, 2)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*pow(d_qy + m_desiredQy, 2)*(-2*d_qz - 2*m_desiredQz)*(d_size +m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(-2*d_qz - 2*m_desiredQz)*pow(d_qz + m_desiredQz, 2)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + m_desiredDistance*(2*d_qz + 2*m_desiredQz)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(6,7) = -m_desiredDistance*pow(d_qw + m_desiredQw, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) - m_desiredDistance*pow(d_qx + m_desiredQx, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) + m_desiredDistance*pow(d_qy + m_desiredQy, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) + m_desiredDistance*pow(d_qz + m_desiredQz, 2)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));

        //8th row pt3y
        positions3DJacobian(7,0) = 0;
        positions3DJacobian(7,1) = 1;
        positions3DJacobian(7,2) = 0;
        positions3DJacobian(7,3) = -2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qw + m_desiredQw)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(7,4) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qx - 2*m_desiredQx)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(7,5) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qy - 2*m_desiredQy)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(-2*d_qy - 2*m_desiredQy)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(7,6) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qz - 2*m_desiredQz)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qw + m_desiredQw)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2);
        positions3DJacobian(7,7) = -2*m_desiredDistance*(d_qw + m_desiredQw)*(d_qz + m_desiredQz)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_qy + m_desiredQy)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));

        //9th row pt3z
        positions3DJacobian(8,0) = 0;
        positions3DJacobian(8,1) = 0;
        positions3DJacobian(8,2) = 1;
        positions3DJacobian(8,3) = 2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qw + m_desiredQw)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(-2*d_qw - 2*m_desiredQw)*(d_qx + m_desiredQx)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(8,4) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qx - 2*m_desiredQx)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(-2*d_qx - 2*m_desiredQx)*(d_qx + m_desiredQx)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(8,5) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(-2*d_qy - 2*m_desiredQy)*(d_qy + m_desiredQy)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) + 2*m_desiredDistance*(d_qw + m_desiredQw)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(-2*d_qy - 2*m_desiredQy)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2);
        positions3DJacobian(8,6) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(d_qy + m_desiredQy)*(-2*d_qz - 2*m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx,2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(-2*d_qz - 2*m_desiredQz)*(d_qz + m_desiredQz)*(d_size + m_desiredSize)/pow(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2), 2) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_size + m_desiredSize)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
        positions3DJacobian(8,7) = 2*m_desiredDistance*(d_qw + m_desiredQw)*(d_qy + m_desiredQy)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2)) - 2*m_desiredDistance*(d_qx + m_desiredQx)*(d_qz + m_desiredQz)/(pow(d_qw + m_desiredQw, 2) + pow(d_qx + m_desiredQx, 2) + pow(d_qy + m_desiredQy, 2) + pow(d_qz + m_desiredQz, 2));
    }

    void
    Formation3DLine3Agents::SetNumberOfAgents ()
    {
        m_numberOfAgents = 3;
    }

}   // namespace Formation3DWithRot