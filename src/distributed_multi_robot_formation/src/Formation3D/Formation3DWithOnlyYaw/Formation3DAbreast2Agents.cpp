//
// Created by benson on 5/8/21.
//

#include "Formation3DAbreast2Agents.h"

namespace Formation3DWithOnlyYaw
{
    Formation3DAbreast2Agents::Formation3DAbreast2Agents()
    : Formation3DBase()
    {
        m_numberOfAgents = 2;
        m_formationType = DistributedFormation::Common::Formation3DType::ABREAST_FORMATION;
    }

    void
    Formation3DAbreast2Agents::GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                                      const double d_yaw,
                                                      const double d_size,
                                                      Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const
    {
        positions3DInFormation.resize(6,1);

        double l = m_desiredDistance*(m_desiredSize+d_size);

        //desiredQ and deltaQ are assumed to be unit quaternions which has been normalized before passing in
        Eigen::Quaterniond desiredQ (m_desiredQw, m_desiredQx, m_desiredQy, m_desiredQz);
        double yaw_qw = std::cos(d_yaw/2.0);
        double yaw_qx = 0.0;
        double yaw_qy = 0.0;
        double yaw_qz = std::sin(d_yaw/2.0);
        Eigen::Quaterniond deltaQ (yaw_qw, yaw_qx, yaw_qy, yaw_qz);
        Eigen::Quaterniond newQ = deltaQ*desiredQ;

        Eigen::Quaterniond pt1 (0, 0, l/2, 0);
        Eigen::Quaterniond pt1_rotated = newQ*pt1*newQ.conjugate();

        Eigen::Quaterniond pt2 (0, 0, -l/2, 0);
        Eigen::Quaterniond pt2_rotated = newQ*pt2*newQ.conjugate();

        positions3DInFormation(0,0) = m_desiredX + d_x + pt1_rotated.x();
        positions3DInFormation(1,0) = m_desiredY + d_y + pt1_rotated.y();
        positions3DInFormation(2,0) = m_desiredZ + d_z + pt1_rotated.z();
        positions3DInFormation(3,0) = m_desiredX + d_x + pt2_rotated.x();
        positions3DInFormation(4,0) = m_desiredY + d_y + pt2_rotated.y();
        positions3DInFormation(5,0) = m_desiredZ + d_z + pt2_rotated.z();
    }

    void
    Formation3DAbreast2Agents::GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                                                     const double d_yaw,
                                                     const double d_size,
                                                     Eigen::Matrix<double, Eigen::Dynamic, 5>& positions3DJacobian) const
    {
        positions3DJacobian.resize(6,5);

        //1st row pt1x
        positions3DJacobian(0,0) = 1;
        positions3DJacobian(0,1) = 0;
        positions3DJacobian(0,2) = 0;
        positions3DJacobian(0,3) = (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQw*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQx*sin((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQx*cos((1.0/2.0)*d_yaw)+ (1.0/2.0)*m_desiredQy*sin((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw));
        positions3DJacobian(0,4) = (1.0/2.0)*m_desiredDistance*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw));

        //2nd row pt1y
        positions3DJacobian(1,0) = 0;
        positions3DJacobian(1,1) = 1;
        positions3DJacobian(1,2) = 0;
        positions3DJacobian(1,3) = (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQw*cos((1.0/2.0)*d_yaw) -1.0/2.0*m_desiredQz*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQw*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQz*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size +m_desiredSize)*(-1.0/2.0*m_desiredQx*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQx*sin((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw)- m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*sin((1.0/2.0)*d_yaw));
        positions3DJacobian(1,4) = (1.0/2.0)*m_desiredDistance*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*pow(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw), 2) - 1.0/2.0*m_desiredDistance*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw));

        //3rd row pt1z
        positions3DJacobian(2,0) = 0;
        positions3DJacobian(2,1) = 0;
        positions3DJacobian(2,2) = 1;
        positions3DJacobian(2,3) = -1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) -1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQw*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*sin((1.0/2.0)*d_yaw))*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*sin((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*cos((1.0/2.0)*d_yaw));
        positions3DJacobian(2,4) = -1.0/2.0*m_desiredDistance*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) -1.0/2.0*m_desiredDistance*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw));


        //4th row pt2x
        positions3DJacobian(3,0) = 1;
        positions3DJacobian(3,1) = 0;
        positions3DJacobian(3,2) = 0;
        positions3DJacobian(3,3) = -1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQw*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQx*sin((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQx*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*sin((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw));
        positions3DJacobian(3,4) = -1.0/2.0*m_desiredDistance*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) +(1.0/2.0)*m_desiredDistance*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw));

        //5th row pt2y
        positions3DJacobian(4,0) = 0;
        positions3DJacobian(4,1) = 1;
        positions3DJacobian(4,2) = 0;
        positions3DJacobian(4,3) = -1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQw*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQw*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQz*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQx*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQx*sin((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*sin((1.0/2.0)*d_yaw));
        positions3DJacobian(4,4) = -1.0/2.0*m_desiredDistance*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw)) -1.0/2.0*m_desiredDistance*pow(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw), 2) + (1.0/2.0)*m_desiredDistance*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw));

        //6th row pt2z
        positions3DJacobian(5,0) = 0;
        positions3DJacobian(5,1) = 0;
        positions3DJacobian(5,2) = 1;
        positions3DJacobian(5,3) = (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*cos((1.0/2.0)*d_yaw) -1.0/2.0*m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*sin((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(-1.0/2.0*m_desiredQw*cos((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*((1.0/2.0)*m_desiredQw*cos((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQz*sin((1.0/2.0)*d_yaw))*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw))*(-1.0/2.0*m_desiredQx*sin((1.0/2.0)*d_yaw) - 1.0/2.0*m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(d_size + m_desiredSize)*(m_desiredQw*cos((1.0/2.0)*d_yaw) -m_desiredQz*sin((1.0/2.0)*d_yaw))*((1.0/2.0)*m_desiredQx*sin((1.0/2.0)*d_yaw) + (1.0/2.0)*m_desiredQy*cos((1.0/2.0)*d_yaw));
        positions3DJacobian(5,4) = (1.0/2.0)*m_desiredDistance*(-m_desiredQw*sin((1.0/2.0)*d_yaw) - m_desiredQz*cos((1.0/2.0)*d_yaw))*(m_desiredQx*sin((1.0/2.0)*d_yaw) + m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(m_desiredQw*sin((1.0/2.0)*d_yaw) + m_desiredQz*cos((1.0/2.0)*d_yaw))*(-m_desiredQx*sin((1.0/2.0)*d_yaw) - m_desiredQy*cos((1.0/2.0)*d_yaw)) + (1.0/2.0)*m_desiredDistance*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw))*(-m_desiredQx*cos((1.0/2.0)*d_yaw) + m_desiredQy*sin((1.0/2.0)*d_yaw)) - 1.0/2.0*m_desiredDistance*(m_desiredQw*cos((1.0/2.0)*d_yaw) - m_desiredQz*sin((1.0/2.0)*d_yaw))*(m_desiredQx*cos((1.0/2.0)*d_yaw) - m_desiredQy*sin((1.0/2.0)*d_yaw));
    }

    void
    Formation3DAbreast2Agents::SetNumberOfAgents ()
    {
        m_numberOfAgents = 2;
    }

}   // namespace Formation3DWithOnlyYaw