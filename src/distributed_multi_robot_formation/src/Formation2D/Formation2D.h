//
// Created by benson on 24/12/21.
//

#pragma once

#include <unordered_map>

#include "../Common/Common.h"
#include "Formation2DWithYaw/Optimize2DFormation.h"
#include "Formation2DWithYaw/Formation2DPoint1Agent.h"
#include "Formation2DWithYaw/Formation2DTri3Agents.h"
#include "Formation2DWithYaw/Formation2DLine3Agents.h"
#include "Formation2DWithYaw/Formation2DAbreast2Agents.h"
#include "Formation2DWithoutYaw/Optimize2DFormation.h"
#include "Formation2DWithoutYaw/Formation2DPoint1Agent.h"
#include "Formation2DWithoutYaw/Formation2DTri3Agents.h"
#include "Formation2DWithoutYaw/Formation2DLine3Agents.h"
#include "Formation2DWithoutYaw/Formation2DAbreast2Agents.h"

namespace Formation2D
{

    static bool GetOptimizedPositions2DInFormation(const DistributedFormation::Common::WORKSPACE ws,
                                                    const int numberOfAgents,
                                                    const double agentRadius,
                                                    const double& desiredAgentSeperationForAbreast,
                                                    const double& desiredAgentSeperationForTri,
                                                    const double& desiredAgentSeperationForLine,
                                                    const double& desiredX,
                                                    const double& desiredY,
                                                    const double& desiredYaw,
                                                    const double weightForDesiredPosition,
                                                    const double weightForDesiredYaw,
                                                    const double weightForDesiredSize,
                                                    const Eigen::Matrix<double, Eigen::Dynamic, 2>& halfplaneConstraintA,
                                                    const Eigen::Matrix<double, Eigen::Dynamic, 1>& halfplaneConstraintb,
                                                    const double desiredFixedHeight,
                                                    const double priorityPenalty,
                                                    std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions,
                                                    double& optDeltaX,
                                                    double& optDeltaY,
                                                    double& optDeltaYaw,
                                                    double& optDeltaSize,
                                                    DistributedFormation::Common::Formation2DType& optFormationType)

    {
        optVirtualPositions.clear();
        optDeltaX = 0;
        optDeltaY = 0;
        optDeltaYaw = 0;
        optDeltaSize = 0;
        optFormationType = DistributedFormation::Common::Formation2DType::NO_FORMATION;

        bool retVal = false;
        switch (ws)
        {
            case DistributedFormation::Common::WORKSPACE::DIM_2_WITHOUT_YAW:
            {
                Formation2DWithoutYaw::Formation2DPoint1Agent formation2DPoint1Agent;
                formation2DPoint1Agent.SetDesiredPositionYawAndSize(desiredX,
                                                                    desiredY,
                                                                    desiredYaw,
                                                                    1.0);
                Formation2DWithoutYaw::Formation2DAbreast2Agents formation2DAbreast2Agents;
                formation2DAbreast2Agents.SetDesiredPositionYawAndSize(desiredX,
                                                                       desiredY,
                                                                       desiredYaw,
                                                                       1.0);
                formation2DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForAbreast);
                Formation2DWithoutYaw::Formation2DTri3Agents formation2DTri3Agents;
                formation2DTri3Agents.SetDesiredPositionYawAndSize(desiredX,
                                                                   desiredY,
                                                                   desiredYaw,
                                                                   1.0);
                formation2DTri3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForTri);
                Formation2DWithoutYaw::Formation2DLine3Agents formation2DLine3Agents;
                formation2DLine3Agents.SetDesiredPositionYawAndSize(desiredX,
                                                                    desiredY,
                                                                    desiredYaw,
                                                                    1.0);
                formation2DLine3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForLine);

                Formation2DWithoutYaw::Optimize2DFormation opt;
                opt.SetFormation2DPoint1Agent(formation2DPoint1Agent);
                opt.SetFormation2DAbreast2Agents(formation2DAbreast2Agents);
                opt.SetFormation2DTri3Agents(formation2DTri3Agents);
                opt.SetFormation2DLine3Agents(formation2DLine3Agents);

                retVal = opt.GetOptimizedPositions2DInFormation (numberOfAgents,
                                                                agentRadius,
                                                                weightForDesiredPosition,
                                                                weightForDesiredSize,
                                                                halfplaneConstraintA,
                                                                halfplaneConstraintb,
                                                                desiredFixedHeight,
                                                                priorityPenalty,
                                                                optVirtualPositions,
                                                                optDeltaX,
                                                                optDeltaY,
                                                                optDeltaSize,
                                                                optFormationType);

                break;
            }

            case DistributedFormation::Common::WORKSPACE::DIM_2_WITH_YAW:
            {
                Formation2DWithYaw::Formation2DPoint1Agent formation2DPoint1Agent;
                formation2DPoint1Agent.SetDesiredPositionYawAndSize(desiredX,
                                                                    desiredY,
                                                                    desiredYaw,
                                                                    1.0);
                Formation2DWithYaw::Formation2DAbreast2Agents formation2DAbreast2Agents;
                formation2DAbreast2Agents.SetDesiredPositionYawAndSize(desiredX,
                                                                       desiredY,
                                                                       desiredYaw,
                                                                       1.0);
                formation2DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForAbreast);
                Formation2DWithYaw::Formation2DTri3Agents formation2DTri3Agents;
                formation2DTri3Agents.SetDesiredPositionYawAndSize(desiredX,
                                                                   desiredY,
                                                                   desiredYaw,
                                                                   1.0);
                formation2DTri3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForTri);
                Formation2DWithYaw::Formation2DLine3Agents formation2DLine3Agents;
                formation2DLine3Agents.SetDesiredPositionYawAndSize(desiredX,
                                                                    desiredY,
                                                                    desiredYaw,
                                                                    1.0);
                formation2DLine3Agents.SetDesiredDistanceBetweenAgents(desiredAgentSeperationForLine);

                std::cout << "opt init" << std::endl;
                Formation2DWithYaw::Optimize2DFormation opt;
                opt.SetFormation2DPoint1Agent(formation2DPoint1Agent);
                std::cout << "opt SetFormation2DPoint1Agent done " << std::endl;
                opt.SetFormation2DAbreast2Agents(formation2DAbreast2Agents);
                std::cout << "opt SetFormation2DAbreast2Agents done " << std::endl;
                opt.SetFormation2DTri3Agents(formation2DTri3Agents);
                std::cout << "opt SetFormation2DTri3Agents done " << std::endl;
                opt.SetFormation2DLine3Agents(formation2DLine3Agents);
                std::cout << "opt.SetFormation2DLine3Agents done" << std::endl;
                std::cout << "opt.GetOptimizedPositions2DInFormation" << std::endl;
                retVal = opt.GetOptimizedPositions2DInFormation (numberOfAgents,
                                                                    agentRadius,
                                                                    weightForDesiredPosition,
                                                                    weightForDesiredYaw,
                                                                    weightForDesiredSize,
                                                                    halfplaneConstraintA,
                                                                    halfplaneConstraintb,
                                                                    desiredFixedHeight,
                                                                    priorityPenalty,
                                                                    optVirtualPositions,
                                                                    optDeltaX,
                                                                    optDeltaY,
                                                                    optDeltaYaw,
                                                                    optDeltaSize,
                                                                    optFormationType);

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
                                        std::pow(position1.y-position2.y, 2));

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





} // Formation2D

