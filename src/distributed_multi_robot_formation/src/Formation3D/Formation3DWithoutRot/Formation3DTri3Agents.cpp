//
// Created by benson on 5/8/21.
//

#include "Formation3DTri3Agents.h"
#include<iostream>

namespace Formation3DWithoutRot
{
    Formation3DTri3Agents::Formation3DTri3Agents()
    : Formation3DBase(3, DistributedFormation::Common::Formation3DType::TRIANGLE_FORMATION)
    {
    }

    void
    Formation3DTri3Agents::GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                                  const double d_size,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const
    {
        positions3DInFormation.resize(9,1);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        //desiredQ and deltaQ are assumed to be unit quaternions which has been normalized before passing in
        Eigen::Quaterniond desiredQ (m_desiredQw, m_desiredQx, m_desiredQy, m_desiredQz);

        Eigen::Quaterniond pt1 (0, l*k_oneOverSqrtOfThree, 0, 0);
        Eigen::Quaterniond pt1_rotated = desiredQ*pt1*desiredQ.conjugate();

        Eigen::Quaterniond pt2 (0, l * k_oneOverSqrtOfThree * std::cos(2.0*M_PI/3.0), l * k_oneOverSqrtOfThree * std::sin(2.0*M_PI/3.0), 0);
        Eigen::Quaterniond pt2_rotated = desiredQ*pt2*desiredQ.conjugate();

        Eigen::Quaterniond pt3 (0, l * k_oneOverSqrtOfThree * std::cos(-2.0*M_PI/3.0), l * k_oneOverSqrtOfThree * std::sin(-2.0*M_PI/3.0), 0);
        Eigen::Quaterniond pt3_rotated = desiredQ*pt3*desiredQ.conjugate();

        positions3DInFormation(0,0) = m_desiredX + d_x + pt1_rotated.x();
        positions3DInFormation(1,0) = m_desiredY + d_y + pt1_rotated.y();
        positions3DInFormation(2,0) = m_desiredZ + d_z + pt1_rotated.z();
        positions3DInFormation(3,0) = m_desiredX + d_x + pt2_rotated.x();
        positions3DInFormation(4,0) = m_desiredY + d_y + pt2_rotated.y();
        positions3DInFormation(5,0) = m_desiredZ + d_z + pt2_rotated.z();
        positions3DInFormation(6,0) = m_desiredX + d_x + pt3_rotated.x();
        positions3DInFormation(7,0) = m_desiredY + d_y + pt3_rotated.y();
        positions3DInFormation(8,0) = m_desiredZ + d_z + pt3_rotated.z();
    }

    void
    Formation3DTri3Agents::GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                                                  const double d_size,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 4>& positions3DJacobian) const
    {
        positions3DJacobian.resize(9,4);

        //1st row pt1x
        positions3DJacobian(0,0) = 1;
        positions3DJacobian(0,1) = 0;
        positions3DJacobian(0,2) = 0;
        positions3DJacobian(0,3) = k_oneOverSqrtOfThree*m_desiredDistance*pow(m_desiredQw, 2) + k_oneOverSqrtOfThree*m_desiredDistance*pow(m_desiredQx, 2) - k_oneOverSqrtOfThree*m_desiredDistance*pow(m_desiredQy, 2) - k_oneOverSqrtOfThree*m_desiredDistance*pow(m_desiredQz, 2);

        //2nd row pt1y
        positions3DJacobian(1,0) = 0;
        positions3DJacobian(1,1) = 1;
        positions3DJacobian(1,2) = 0;
        positions3DJacobian(1,3) = 2*k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*m_desiredQz + 2*k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*m_desiredQy;

        //3rd row pt1z
        positions3DJacobian(2,0) = 0;
        positions3DJacobian(2,1) = 0;
        positions3DJacobian(2,2) = 1;
        positions3DJacobian(2,3) = -2*k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*m_desiredQy + 2*k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*m_desiredQz;


        //4th row pt2x
        positions3DJacobian(3,0) = 1;
        positions3DJacobian(3,1) = 0;
        positions3DJacobian(3,2) = 0;
        positions3DJacobian(3,3) = m_desiredQw*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*cos((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*sin((2.0/3.0)*M_PI)) - m_desiredQx*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*cos((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*sin((2.0/3.0)*M_PI)) + m_desiredQy*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*sin((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*cos((2.0/3.0)*M_PI)) - m_desiredQz*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*sin((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*cos((2.0/3.0)*M_PI));

        //5th row pt2y
        positions3DJacobian(4,0) = 0;
        positions3DJacobian(4,1) = 1;
        positions3DJacobian(4,2) = 0;
        positions3DJacobian(4,3) = m_desiredQw*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*sin((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*cos((2.0/3.0)*M_PI)) - m_desiredQx*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*sin((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*cos((2.0/3.0)*M_PI)) - m_desiredQy*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*cos((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*sin((2.0/3.0)*M_PI)) + m_desiredQz*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*cos((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*sin((2.0/3.0)*M_PI));

        //6th row pt2z
        positions3DJacobian(5,0) = 0;
        positions3DJacobian(5,1) = 0;
        positions3DJacobian(5,2) = 1;
        positions3DJacobian(5,3) = m_desiredQw*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*sin((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*cos((2.0/3.0)*M_PI)) + m_desiredQx*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*sin((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*cos((2.0/3.0)*M_PI)) - m_desiredQy*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*cos((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*sin((2.0/3.0)*M_PI)) - m_desiredQz*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*cos((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*sin((2.0/3.0)*M_PI));


        //7th row pt3x
        positions3DJacobian(6,0) = 1;
        positions3DJacobian(6,1) = 0;
        positions3DJacobian(6,2) = 0;
        positions3DJacobian(6,3) = m_desiredQw*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*cos((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*sin((2.0/3.0)*M_PI)) - m_desiredQx*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*cos((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*sin((2.0/3.0)*M_PI)) + m_desiredQy*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*sin((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*cos((2.0/3.0)*M_PI)) - m_desiredQz*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*sin((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*cos((2.0/3.0)*M_PI));

        //8th row pt3y
        positions3DJacobian(7,0) = 0;
        positions3DJacobian(7,1) = 1;
        positions3DJacobian(7,2) = 0;
        positions3DJacobian(7,3) = m_desiredQw*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*sin((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*cos((2.0/3.0)*M_PI)) - m_desiredQx*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*sin((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*cos((2.0/3.0)*M_PI)) - m_desiredQy*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*cos((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*sin((2.0/3.0)*M_PI)) + m_desiredQz*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*cos((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*sin((2.0/3.0)*M_PI));

        //9th row pt3z
        positions3DJacobian(8,0) = 0;
        positions3DJacobian(8,1) = 0;
        positions3DJacobian(8,2) = 1;
        positions3DJacobian(8,3) = m_desiredQw*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*sin((2.0/3.0)*M_PI) - k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*cos((2.0/3.0)*M_PI)) + m_desiredQx*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*sin((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*cos((2.0/3.0)*M_PI)) - m_desiredQy*(k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQw*cos((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQz*sin((2.0/3.0)*M_PI)) - m_desiredQz*(-k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQx*cos((2.0/3.0)*M_PI) + k_oneOverSqrtOfThree*m_desiredDistance*m_desiredQy*sin((2.0/3.0)*M_PI));
    }

}   // namespace Formation3DWithoutRot