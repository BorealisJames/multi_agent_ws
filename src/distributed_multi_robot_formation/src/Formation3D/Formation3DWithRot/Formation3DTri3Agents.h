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

class Formation3DTri3Agents : public Formation3DBase
{
/*
 - line will have the tip pointing to the desired direction
 - the point on the tip will be pt1
 - the next point will be pt2
 - the last point will be pt3

 - the center point is (desiredX+deltaX, desiredY+deltaY, desiredZ+deltaZ)
 - the direction which the formation points to is Q_delta*Q_desired
 - the size of the formation is desiredSize+deltaSize
 */
public:
    Formation3DTri3Agents();

    // output is [pt1x, pt1y, pt1z, pt2x, pt2y, pt2z, pt3x, pt3y, pt3z]^T
    void GetFormationPositions (const double d_x, const double d_y, const double d_z,
                                const double d_qw, const double d_qx, const double d_qy, const double d_qz,
                                const double d_size,
                                Eigen::Matrix<double, Eigen::Dynamic, 1>& positions3DInFormation) const override;

    // output is [d_pt1x/d_deltaX, d_pt1x/d_deltaY, d_pt1x/d_delyaZ, d_pt1x/d_deltaQw, ...  d_pt1x/d_deltaSize ;
    //            d_pt1y/d_deltaX, d_pt1y/d_deltaY, d_pt1y/d_delyaZ, d_pt1y/d_deltaQw, ...  d_pt1y/d_deltaSize ;
    //            ...
    //            d_pt3z/d_deltaX, d_pt3z/d_deltaY, d_pt3z/d_delyaZ, d_pt3z/d_deltaQw, ...  d_pt3z/d_deltaSize ;]
    void GetFormationJacobian (const double d_x, const double d_y, const double d_z,
                               const double d_qw, const double d_qx, const double d_qy, const double d_qz,
                               const double d_size,
                               Eigen::Matrix<double, Eigen::Dynamic, 8>& positions3DJacobian) const override;

protected:
    void SetNumberOfAgents () override;

};

}   // namespace Formation3DWithRot
