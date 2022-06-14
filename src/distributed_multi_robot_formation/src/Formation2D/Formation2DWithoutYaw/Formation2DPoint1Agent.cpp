//
// Created by benson on 25/1/21.
//

#include "Formation2DPoint1Agent.h"

namespace Formation2DWithoutYaw
{
    Formation2DPoint1Agent::Formation2DPoint1Agent()
    : Formation2DBase(1, DistributedFormation::Common::Formation2DType::POINT_FORMATION)
    {
    }

    void
    Formation2DPoint1Agent::GetFormationPositions (const double d_x, const double d_y,
                                                      const double d_size,
                                                      Eigen::Matrix<double, Eigen::Dynamic, 1>& positions2DInFormation) const
    {
        positions2DInFormation.resize(2,1);

        positions2DInFormation(0,0) = m_desiredX + d_x;
        positions2DInFormation(1,0) = m_desiredY + d_y;
    }

    void
    Formation2DPoint1Agent::GetFormationJacobian (const double d_x, const double d_y,
                                                     const double d_size,
                                                     Eigen::Matrix<double, Eigen::Dynamic, 3>& positions2DJacobian) const
    {
        positions2DJacobian.resize(4,3);

        //first row pt1x
        positions2DJacobian(0,0) = 1;
        positions2DJacobian(0,1) = 0;
        positions2DJacobian(0,2) = 0;

        //second row pt1y
        positions2DJacobian(1,0) = 0;
        positions2DJacobian(1,1) = 1;
        positions2DJacobian(1,2) = 0;
    }

}   // namespace Formation2DWithoutYaw