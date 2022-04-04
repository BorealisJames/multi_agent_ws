//
// Created by benson on 26/1/21.
//

#pragma once

#include <cfloat>
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>
#include <iostream>
#include <unordered_map>

#include "../../Common/Common.h"
#include "Formation2DBase.h"
#include "Formation2DAbreast2Agents.h"
#include "Formation2DLine3Agents.h"
#include "Formation2DTri3Agents.h"
#include "Optimizer2DVarsConstrCost.h"

namespace Formation2DWithYaw
{

class Optimize2DFormation
{
public:
    Optimize2DFormation();

    void SetFormation2DAbreast2Agents (const Formation2DAbreast2Agents& formation2DAbreast2Agents);
    void SetFormation2DLine3Agents (const Formation2DLine3Agents& formation2DLine3Agents);
    void SetFormation2DTri3Agents (const Formation2DTri3Agents& formation2DTri3Agents);

    void RemoveAllFormations ();
    void RemoveFormation2DAbreast2Agents ();
    void RemoveFormation2DLine3Agents ();
    void RemoveFormation2DTri3Agents ();

    bool GetOptimizedPositions2DInFormation (const int numberOfAgents,
                                             const double agentRadius,
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
                                             DistributedFormation::Common::Formation2DType& optFormationType);

private:

    void PopulateVectorWithFormationInOrderOfPriority(const int numberOfAgents,
                                                      std::vector <Formation2DBase::Ptr>& formation2DBasePtrVec);

//    void ConvertVectorOf2DPointsToMatrixOf2DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
//                                                    Eigen::Matrix<double, Eigen::Dynamic, 2>& pointsMat);
    void ConvertVectorOf2DPointsToMapOf2DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
                                                 const double desiredHeight,
                                                 std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions);

    bool m_formation2DAbreast2AgentsInitialized;
    bool m_formation2DLine3AgentsInitialized;
    bool m_formation2DTri3AgentsInitialized;

    Formation2DAbreast2Agents m_formation2DAbreast2Agents;
    Formation2DLine3Agents m_formation2DLine3Agents;
    Formation2DTri3Agents m_formation2DTri3Agents;


};

}   // namespace Formation2DWithYaw