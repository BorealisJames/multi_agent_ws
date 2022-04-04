//
// Created by benson on 12/8/21.
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
#include "Formation3DBase.h"
#include "Formation3DAbreast2Agents.h"
#include "Formation3DLine3Agents.h"
#include "Formation3DTri3Agents.h"
#include "Optimizer3DVarsConstrCost.h"

namespace Formation3DWithOnlyYaw
{

class Optimize3DFormation
{
public:
    Optimize3DFormation();

    void SetFormation3DAbreast2Agents (const Formation3DAbreast2Agents& formation3DAbreast2Agents);
    void SetFormation3DLine3Agents (const Formation3DLine3Agents& formation3DLine3Agents);
    void SetFormation3DTri3Agents (const Formation3DTri3Agents& formation3DTri3Agents);

    void RemoveAllFormations ();
    void RemoveFormation3DAbreast2Agents ();
    void RemoveFormation3DLine3Agents ();
    void RemoveFormation3DTri3Agents ();

    bool GetOptimizedPositions3DInFormation (const int numberOfAgents,
                                             const double agentRadius,
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
                                             double& optDeltaYaw,
                                             double& optDeltaSize,
                                             DistributedFormation::Common::Formation3DType& optFormationType);

private:

    void PopulateVectorWithFormationInOrderOfPriority(const int numberOfAgents, std::vector <Formation3DBase::Ptr>& formation3DBasePtrVec);

//    void ConvertVectorOf3DPointsToMatrixOf3DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
//                                                    Eigen::Matrix<double, Eigen::Dynamic, 3>& pointsMat);
    void ConvertVectorOf3DPointsToMapOf3DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
                                                 std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions);

    bool m_formation3DAbreast2AgentsInitialized;
    bool m_formation3DLine3AgentsInitialized;
    bool m_formation3DTri3AgentsInitialized;

    Formation3DAbreast2Agents m_formation3DAbreast2Agents;
    Formation3DLine3Agents m_formation3DLine3Agents;
    Formation3DTri3Agents m_formation3DTri3Agents;

};

}   // namespace Formation3DWithOnlyYaw