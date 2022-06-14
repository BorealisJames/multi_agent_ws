//
// Created by benson on 5/8/21.
//

#include "Formation3DPoint1Agent.h"

namespace Formation3DWithRot
{
    Formation3DPoint1Agent::Formation3DPoint1Agent()
    : Formation3DBase(1, DistributedFormation::Common::Formation3DType::POINT_FORMATION)
    {
    }

    void
    Formation3DPoint1Agent::GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                                   const double d_qw, const double d_qx, const double d_qy, const double d_qz,
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
                                                  const double d_qw, const double d_qx, const double d_qy, const double d_qz,
                                                  const double d_size,
                                                  Eigen::Matrix<double, Eigen::Dynamic, 8>& positions3DJacobian) const
    {
        positions3DJacobian.resize(3,8);

        //1st row pt1x
        positions3DJacobian(0,0) = 1;
        positions3DJacobian(0,1) = 0;
        positions3DJacobian(0,2) = 0;
        positions3DJacobian(0,3) = 0;
        positions3DJacobian(0,4) = 0;
        positions3DJacobian(0,5) = 0;
        positions3DJacobian(0,6) = 0;
        positions3DJacobian(0,7) = 0;

        //2nd row pt1y
        positions3DJacobian(1,0) = 0;
        positions3DJacobian(1,1) = 1;
        positions3DJacobian(1,2) = 0;
        positions3DJacobian(1,3) = 0;
        positions3DJacobian(1,4) = 0;
        positions3DJacobian(1,5) = 0;
        positions3DJacobian(1,6) = 0;
        positions3DJacobian(1,7) = 0;

        //3rd row pt1z
        positions3DJacobian(2,0) = 0;
        positions3DJacobian(2,1) = 0;
        positions3DJacobian(2,2) = 1;
        positions3DJacobian(2,3) = 0;
        positions3DJacobian(2,4) = 0;
        positions3DJacobian(2,5) = 0;
        positions3DJacobian(2,6) = 0;
        positions3DJacobian(2,7) = 0;
   }

}   // namespace Formation3DWithRot