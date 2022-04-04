//
// Created by benson on 24/12/21.
//

#pragma once

#include <unordered_map>

#include "../Common/Common.h"
#include "Formation3DWithRot/Optimize3DFormation.h"
#include "Formation3DWithRot/Formation3DTri3Agents.h"
#include "Formation3DWithRot/Formation3DLine3Agents.h"
#include "Formation3DWithRot/Formation3DAbreast2Agents.h"
#include "Formation3DWithoutRot/Optimize3DFormation.h"
#include "Formation3DWithoutRot/Formation3DTri3Agents.h"
#include "Formation3DWithoutRot/Formation3DLine3Agents.h"
#include "Formation3DWithoutRot/Formation3DAbreast2Agents.h"
#include "Formation3DWithOnlyYaw//Optimize3DFormation.h"
#include "Formation3DWithOnlyYaw/Formation3DTri3Agents.h"
#include "Formation3DWithOnlyYaw/Formation3DLine3Agents.h"
#include "Formation3DWithOnlyYaw/Formation3DAbreast2Agents.h"

namespace Formation3D
{

    static bool GetOptimizedPositions3DInFormation(const DistributedFormation::Common::WORKSPACE ws,
                                                    const int numberOfAgents,
                                                    const double agentRadius,
                                                    const double& desiredAgentSeperationForAbreast,
                                                    const double& desiredAgentSeperationForTri,
                                                    const double& desiredAgentSeperationForLine,
                                                   const double& desiredX, const double& desiredY, const double& desiredZ,
                                                   const double& desiredQw, const double& desiredQx, const double& desiredQy, const double& desiredQz,
                                                   const double weightForDesiredPosition,
                                                   const double weightForDesiredRotation,
                                                   const double weightForDesiredSize,
                                                   const Eigen::Matrix<double, Eigen::Dynamic, 3>& halfplaneConstraintA,
                                                   const Eigen::Matrix<double, Eigen::Dynamic, 1>& halfplaneConstraintb,
                                                   const double priorityPenalty,
                                                   std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions,
                                                   double& optDeltaX,
                                                   double& optDeltaY,
                                                   double& optDeltaZ,
                                                   double& optDeltaQw,
                                                   double& optDeltaQx,
                                                   double& optDeltaQy,
                                                   double& optDeltaQz,
                                                   double& optDeltaSize,
                                                   DistributedFormation::Common::Formation3DType& optFormationType)

    {
        optVirtualPositions.clear();
        optDeltaX = 0;
        optDeltaY = 0;
        optDeltaZ = 0;
        optDeltaQw = 0;
        optDeltaQx = 0;
        optDeltaQy = 0;
        optDeltaQz = 0;
        optDeltaSize = 0;
        optFormationType = DistributedFormation::Common::Formation3DType::NO_FORMATION;

        bool retVal = false;
        switch (ws)
        {
            case DistributedFormation::Common::WORKSPACE::DIM_3_WITH_ROT:
            {
                Formation3DWithRot::Formation3DAbreast2Agents formation3DAbreast2Agents;
                formation3DAbreast2Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                            desiredQw, desiredQx, desiredQy, desiredQz,
                                                                            1.0);
                formation3DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForAbreast);
                Formation3DWithRot::Formation3DTri3Agents formation3DTri3Agents;
                formation3DTri3Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                        desiredQw, desiredQx, desiredQy, desiredQz,
                                                                        1.0);
                formation3DTri3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForTri);
                Formation3DWithRot::Formation3DLine3Agents formation3DLine3Agents;
                formation3DLine3Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                         desiredQw, desiredQx, desiredQy, desiredQz,
                                                                         1.0);
                formation3DLine3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForLine);

                Formation3DWithRot::Optimize3DFormation opt;
                opt.SetFormation3DAbreast2Agents(formation3DAbreast2Agents);
                opt.SetFormation3DTri3Agents(formation3DTri3Agents);
                opt.SetFormation3DLine3Agents(formation3DLine3Agents);

                retVal = opt.GetOptimizedPositions3DInFormation(numberOfAgents,
                                                                agentRadius,
                                                                weightForDesiredPosition,
                                                                weightForDesiredRotation,
                                                                weightForDesiredSize,
                                                                halfplaneConstraintA,
                                                                halfplaneConstraintb,
                                                                priorityPenalty,
                                                                optVirtualPositions,
                                                                optDeltaX,
                                                                optDeltaY,
                                                                optDeltaZ,
                                                                optDeltaQw,
                                                                optDeltaQx,
                                                                optDeltaQy,
                                                                optDeltaQz,
                                                                optDeltaSize,
                                                                optFormationType);

                break;
            }

            case DistributedFormation::Common::WORKSPACE::DIM_3_WITHOUT_ROT:
            {
                Formation3DWithoutRot::Formation3DAbreast2Agents formation3DAbreast2Agents;
                formation3DAbreast2Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                            desiredQw, desiredQx, desiredQy, desiredQz,
                                                                            1.0);
                formation3DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForAbreast);
                Formation3DWithoutRot::Formation3DTri3Agents formation3DTri3Agents;
                formation3DTri3Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                        desiredQw, desiredQx, desiredQy, desiredQz,
                                                                        1.0);
                formation3DTri3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForTri);
                Formation3DWithoutRot::Formation3DLine3Agents formation3DLine3Agents;
                formation3DLine3Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                         desiredQw, desiredQx, desiredQy, desiredQz,
                                                                         1.0);
                formation3DLine3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForLine);

                Formation3DWithoutRot::Optimize3DFormation opt;
                opt.SetFormation3DAbreast2Agents(formation3DAbreast2Agents);
                opt.SetFormation3DTri3Agents(formation3DTri3Agents);
                opt.SetFormation3DLine3Agents(formation3DLine3Agents);

                retVal = opt.GetOptimizedPositions3DInFormation(numberOfAgents,
                                                                agentRadius,
                                                                weightForDesiredPosition,
                                                                weightForDesiredSize,
                                                                halfplaneConstraintA,
                                                                halfplaneConstraintb,
                                                                priorityPenalty,
                                                                optVirtualPositions,
                                                                optDeltaX,
                                                                optDeltaY,
                                                                optDeltaZ,
                                                                optDeltaSize,
                                                                optFormationType);

                break;
            }

            case DistributedFormation::Common::WORKSPACE::DIM_3_WITH_ONLY_YAW:
            {
                Formation3DWithOnlyYaw::Formation3DAbreast2Agents formation3DAbreast2Agents;
                formation3DAbreast2Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                            desiredQw, desiredQx, desiredQy, desiredQz,
                                                                            1.0);
                formation3DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForAbreast);
                Formation3DWithOnlyYaw::Formation3DTri3Agents formation3DTri3Agents;
                formation3DTri3Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                        desiredQw, desiredQx, desiredQy, desiredQz,
                                                                        1.0);
                formation3DTri3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForTri);
                Formation3DWithOnlyYaw::Formation3DLine3Agents formation3DLine3Agents;
                formation3DLine3Agents.SetDesiredPositionRotationAndSize(desiredX, desiredY, desiredZ,
                                                                         desiredQw, desiredQx, desiredQy, desiredQz,
                                                                         1.0);
                formation3DLine3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForLine);

                Formation3DWithOnlyYaw::Optimize3DFormation opt;
                opt.SetFormation3DAbreast2Agents(formation3DAbreast2Agents);
                opt.SetFormation3DTri3Agents(formation3DTri3Agents);
                opt.SetFormation3DLine3Agents(formation3DLine3Agents);

                double optDeltaYaw;

                retVal = opt.GetOptimizedPositions3DInFormation(numberOfAgents,
                                                                agentRadius,
                                                                weightForDesiredPosition,
                                                                weightForDesiredRotation,
                                                                weightForDesiredSize,
                                                                halfplaneConstraintA,
                                                                halfplaneConstraintb,
                                                                priorityPenalty,
                                                                optVirtualPositions,
                                                                optDeltaX,
                                                                optDeltaY,
                                                                optDeltaZ,
                                                                optDeltaYaw,
                                                                optDeltaSize,
                                                                optFormationType);

                Eigen::Quaterniond desiredQ (desiredQw, desiredQx, desiredQy, desiredQz);
                double yaw_qw = std::cos(optDeltaYaw/2.0);
                double yaw_qx = 0.0;
                double yaw_qy = 0.0;
                double yaw_qz = std::sin(optDeltaYaw/2.0);
                Eigen::Quaterniond deltaQ (yaw_qw, yaw_qx, yaw_qy, yaw_qz);
                Eigen::Quaterniond newQ = deltaQ*desiredQ;

                optDeltaQw = newQ.w() - desiredQw;
                optDeltaQx = newQ.x() - desiredQx;
                optDeltaQy = newQ.y() - desiredQy;
                optDeltaQz = newQ.z() - desiredQz;

                break;
            }

            default:
                retVal = false;
                break;
        }

        //check that agents cannot be too close
        std::vector<DistributedFormation::Common::Position> optVirtualPositionsVec;
        for (auto&& optVirtualPosition : optVirtualPositions)
        {
            optVirtualPositionsVec.push_back(optVirtualPosition.second);
        }

        bool agentsAreSpacedSafelyApart = true;
        for (int i = 0; i < static_cast<int>(optVirtualPositionsVec.size())-1 && agentsAreSpacedSafelyApart; ++i)
        {
            DistributedFormation::Common::Position position1 = optVirtualPositionsVec.at(i);
            for (int j = i+1; j < static_cast<int>(optVirtualPositionsVec.size()) && agentsAreSpacedSafelyApart; ++j)
            {
                DistributedFormation::Common::Position position2 = optVirtualPositionsVec.at(j);

                double dist = std::sqrt(std::pow(position1.x-position2.x, 2) +
                                        std::pow(position1.y-position2.y, 2) +
                                        std::pow(position1.z-position2.z, 2));

                if (dist + FLT_EPSILON < 2*agentRadius)
                {
                    agentsAreSpacedSafelyApart=false;
                    break;
                }
            }
        }

        retVal &= agentsAreSpacedSafelyApart;

        return retVal;
    }





} // Formation3D

