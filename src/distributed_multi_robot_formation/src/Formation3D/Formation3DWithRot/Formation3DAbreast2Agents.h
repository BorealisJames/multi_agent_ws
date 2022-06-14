//
// Created by benson on 5/8/21.
//

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

#include "../../Common/Common.h"

#include "Formation3DBase.h"

namespace Formation3DWithRot
{

class Formation3DAbreast2Agents : public Formation3DBase
{
/*
 - abreast will have pt1 on the left and pt2 on the right

 - the center point is (desiredX+deltaX, desiredY+deltaY, desiredZ+deltaZ)
 - the direction which the formation points to is Q_delta*Q_desired
 - the size of the formation is desiredSize+deltaSize
 */
public:
    Formation3DAbreast2Agents();

    // output is [pt1x, pt1y, pt1z, pt2x, pt2y, pt2z]^T
    void GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                const double d_qw, const double d_qx, const double d_qy, const double d_qz,
                                const double d_size,
                                Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const override;

    // output is [d_pt1x/d_deltaX, d_pt1x/d_deltaY, d_pt1x/d_deltaZ, d_pt1x/d_deltaQw, ...  d_pt1x/d_deltaSize ;
    //            d_pt1y/d_deltaX, d_pt1y/d_deltaY, d_pt1y/d_deltaZ, d_pt1y/d_deltaQw, ...  d_pt1y/d_deltaSize ;
    //            ...
    //            d_pt2z/d_deltaX, d_pt2z/d_deltaY, d_pt2z/d_deltaZ, d_pt2z/d_deltaQw, ...  d_pt2z/d_deltaSize ;]
    void GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                               const double d_qw, const double d_qx, const double d_qy, const double d_qz,
                               const double d_size,
                               Eigen::Matrix<double, Eigen::Dynamic, 8>& positions3DJacobian) const override;
};

}   // namespace Formation3DWithRot
