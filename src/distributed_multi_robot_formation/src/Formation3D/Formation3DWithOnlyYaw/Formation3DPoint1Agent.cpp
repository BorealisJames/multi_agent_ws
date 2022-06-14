//
// Created by benson on 5/8/21.
//

#include "Formation3DPoint1Agent.h"

namespace Formation3DWithOnlyYaw
{
    Formation3DPoint1Agent::Formation3DPoint1Agent()
    : Formation3DBase(1, DistributedFormation::Common::Formation3DType::POINT_FORMATION)
    {
    }

    void
    Formation3DPoint1Agent::GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                                   const double d_yaw,
                                                   const double d_size,
                                                   Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const
    {
        positions3DInFormation.resize(3,1);

        positions3DInFormation(0,0) = m_desiredX + d_x;
        positions3DInFormation(1,0) = m_desiredY + d_y;
        positions3DInFormation(2,0) = m_desiredZ + d_z;
    }

    void
    Formation3DPoint1Agent::GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                                                     const double d_yaw,
                                                     const double d_size,
                                                     Eigen::Matrix<double, Eigen::Dynamic, 5>& positions3DJacobian) const
    {
        positions3DJacobian.resize(3,5);

        //1st row pt1x
        positions3DJacobian(0,0) = 1;
        positions3DJacobian(0,1) = 0;
        positions3DJacobian(0,2) = 0;
        positions3DJacobian(0,3) = 0;
        positions3DJacobian(0,4) = 0;

        //2nd row pt1y
        positions3DJacobian(1,0) = 0;
        positions3DJacobian(1,1) = 1;
        positions3DJacobian(1,2) = 0;
        positions3DJacobian(1,3) = 0;
        positions3DJacobian(1,4) = 0;

        //3rd row pt1z
        positions3DJacobian(2,0) = 0;
        positions3DJacobian(2,1) = 0;
        positions3DJacobian(2,2) = 1;
        positions3DJacobian(2,3) = 0;
        positions3DJacobian(2,4) = 0;
    }

}   // namespace Formation3DWithOnlyYaw