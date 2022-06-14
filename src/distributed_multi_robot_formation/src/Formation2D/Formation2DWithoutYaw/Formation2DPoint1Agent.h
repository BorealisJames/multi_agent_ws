//
// Created by benson on 25/1/21.
//

#pragma once

#include <Eigen/Core>
#include <math.h>

#include "../../Common/Common.h"

#include "Formation2DBase.h"

namespace Formation2DWithoutYaw
{

class Formation2DPoint1Agent : public Formation2DBase
{
/*
 - point will have pt1 as close to the center point as possible

 - the center point is (desiredX+deltaX, desiredY+deltaY)
 - the direction which the formation points to is m_desiredYaw + d_yaw
 - the size of the formation is m_desiredSize+d_size
 */
public:
    Formation2DPoint1Agent();

    // output is [pt1x, pt1y]^T
    void GetFormationPositions (const double d_x, const double d_y,
                                const double d_size,
                                Eigen::Matrix<double, Eigen::Dynamic, 1>& positions2DInFormation) const override;

        // output is [d_pt1x/d_deltaX, d_pt1x/d_deltaY, d_pt1x/d_deltaSize ;
        //            d_pt1y/d_deltaX, d_pt1y/d_deltaY, d_pt1y/d_deltaSize ]
    void GetFormationJacobian (const double d_x, const double d_y,
                               const double d_size,
                               Eigen::Matrix<double, Eigen::Dynamic, 3>& positions2DJacobian) const override;
};

}   // namespace Formation2DWithoutYaw
