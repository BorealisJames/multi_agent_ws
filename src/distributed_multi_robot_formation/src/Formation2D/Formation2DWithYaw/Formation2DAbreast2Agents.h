//
// Created by benson on 25/1/21.
//

#pragma once

#include <Eigen/Core>
#include <math.h>

#include "../../Common/Common.h"

#include "Formation2DBase.h"

namespace Formation2DWithYaw
{

class Formation2DAbreast2Agents : public Formation2DBase
{
/*
 - abreast will have pt1 on the left and pt2 on the right

 - the center point is (desiredX+deltaX, desiredY+deltaY)
 - the direction which the formation points to is m_desiredYaw + d_yaw
 - the size of the formation is m_desiredSize+d_size
 */
public:
        Formation2DAbreast2Agents();

    // output is [pt1x, pt1y, pt2x, pt2y, pt3x, pt3y]^T
    void GetFormationPositions (const double d_x, const double d_y,
                                const double d_yaw,
                                const double d_size,
                                Eigen::Matrix<double, Eigen::Dynamic, 1>& positions2DInFormation) const override;

        // output is [d_pt1x/d_deltaX, d_pt1x/d_deltaY, d_pt1x/d_deltaYaw, d_pt1x/d_deltaSize ;
        //            d_pt1y/d_deltaX, d_pt1y/d_deltaY, d_pt1y/d_deltaYaw, d_pt1y/d_deltaSize ;
        //            ...
        //            d_pt3y/d_deltaX, d_pt3y/d_deltaY, d_pt3y/d_deltaYaw, d_pt3y/d_deltaSize ;]
    void GetFormationJacobian (const double d_x, const double d_y,
                               const double d_yaw,
                               const double d_size,
                               Eigen::Matrix<double, Eigen::Dynamic, 4>& positions2DJacobian) const override;

protected:
    void SetNumberOfAgents () override;

};

}   // namespace Formation2DWithYaw
