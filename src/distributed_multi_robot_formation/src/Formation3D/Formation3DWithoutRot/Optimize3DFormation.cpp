//
// Created by benson on 12/8/21.
//

#include "Optimize3DFormation.h"

namespace Formation3DWithoutRot
{

    Optimize3DFormation::Optimize3DFormation()
    : m_formation3DPoint1AgentInitialized(false)
    , m_formation3DAbreast2AgentsInitialized(false)
    , m_formation3DLine3AgentsInitialized(false)
    , m_formation3DTri3AgentsInitialized(false)
    {}

    void
    Optimize3DFormation::SetFormation3DPoint1Agent (const Formation3DPoint1Agent& formation3DPoint1Agent)
    {
        m_formation3DPoint1Agent = formation3DPoint1Agent;
        m_formation3DPoint1AgentInitialized = true;
    }

    void
    Optimize3DFormation::SetFormation3DAbreast2Agents (const Formation3DAbreast2Agents& formation3DAbreast2Agents)
    {
        m_formation3DAbreast2Agents = formation3DAbreast2Agents;
        m_formation3DAbreast2AgentsInitialized = true;
    }

    void
    Optimize3DFormation::SetFormation3DLine3Agents (const Formation3DLine3Agents& formation3DLine3Agents)
    {
        m_formation3DLine3Agents = formation3DLine3Agents;
        m_formation3DLine3AgentsInitialized = true;
    }

    void
    Optimize3DFormation::SetFormation3DTri3Agents (const Formation3DTri3Agents& formation3DTri3Agents)
    {
        m_formation3DTri3Agents = formation3DTri3Agents;
        m_formation3DTri3AgentsInitialized = true;
    }

    void
    Optimize3DFormation::RemoveAllFormations ()
    {
        m_formation3DPoint1AgentInitialized = false;
        m_formation3DAbreast2AgentsInitialized = false;
        m_formation3DLine3AgentsInitialized = false;
        m_formation3DTri3AgentsInitialized = false;
    }

    void
    Optimize3DFormation::RemoveFormation3DPoint1Agent ()
    {
        m_formation3DPoint1AgentInitialized = false;
    }

    void
    Optimize3DFormation::RemoveFormation3DAbreast2Agents ()
    {
        m_formation3DAbreast2AgentsInitialized = false;
    }

    void
    Optimize3DFormation::RemoveFormation3DLine3Agents ()
    {
        m_formation3DLine3AgentsInitialized = false;
    }

    void
    Optimize3DFormation::RemoveFormation3DTri3Agents ()
    {
        m_formation3DTri3AgentsInitialized = false;
    }

    bool
    Optimize3DFormation::GetOptimizedPositions3DInFormation (const int numberOfAgents,
                                                             const double agentRadius,
                                                             const double weightForDesiredPosition,
                                                             const double weightForDesiredSize,
                                                             const Eigen::Matrix<double, Eigen::Dynamic, 3>& halfplaneConstraintA,
                                                             const Eigen::Matrix<double, Eigen::Dynamic, 1>& halfplaneConstraintb,
                                                             const double priorityPenalty,
                                                             std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions,
                                                             double& optDeltaX,
                                                             double& optDeltaY,
                                                             double& optDeltaZ,
                                                             double& optDeltaSize,
                                                             DistributedFormation::Common::Formation3DType& optFormationType)
    {
        bool retVal = false;

        optVirtualPositions.clear();

        //check if optimized points are feasible
        size_t numberOfConstraints = 0;
        if (halfplaneConstraintA.rows() != halfplaneConstraintb.rows())
        {
            return retVal;
        }
        else
        {
            numberOfConstraints = halfplaneConstraintA.rows();
        }

        //get formation to be pushed into vector according to priority;
        std::vector <Formation3DBase::Ptr> formation3DBasePtrVec;
        PopulateVectorWithFormationInOrderOfPriority(numberOfAgents, formation3DBasePtrVec);

        Eigen::Matrix<double, Eigen::Dynamic, 1> optPositions3DInFormationVec;
        double minCost = DBL_MAX;

        for (int f = 0; f < formation3DBasePtrVec.size(); f++)
        {
            Formation3DBase::Ptr formationPtr = formation3DBasePtrVec.at(f);
            double desiredDistanceInFormation = formationPtr->GetDesiredDistanceBetweenAgents();

            ifopt::Problem nlp;
            nlp.AddVariableSet(std::make_shared<Formation3DVariables>(desiredDistanceInFormation,
                                                                      agentRadius));
            nlp.AddConstraintSet(std::make_shared<Formation3DStaticConstraint>(halfplaneConstraintA,
                                                                               halfplaneConstraintb,
                                                                               agentRadius,
                                                                               formationPtr));
            nlp.AddCostSet(std::make_shared<Formation3DCost>(0.0,
                                                             0.0,
                                                             0.0,
                                                             0.0,
                                                             weightForDesiredPosition,
                                                             weightForDesiredSize));

            // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
            auto solver = std::make_shared<ifopt::IpoptSolver>();
            solver->SetOption("max_cpu_time", 5.0);
            solver->Solve(nlp);

            Eigen::VectorXd optVaraiblesVec = nlp.GetVariableValues();
            double optRefDeltaX = optVaraiblesVec(0);
            double optRefDeltaY = optVaraiblesVec(1);
            double optRefDeltaZ = optVaraiblesVec(2);
            double optRefDeltaSize = optVaraiblesVec(3);

            double optVaraiblesArray[4] = {optRefDeltaX,
                                           optRefDeltaY,
                                           optRefDeltaZ,
                                           optRefDeltaSize};
            double cost = nlp.EvaluateCostFunction(optVaraiblesArray);
            double discountedCost = (f * priorityPenalty) + cost;

            std::cout << "f: " << f
                        << ", discountedCost: " << discountedCost << std::endl;

            Eigen::Matrix<double, Eigen::Dynamic, 1> optRefPositions3DInFormationVec;
            formationPtr->GetFormationPositions(optRefDeltaX,
                                                optRefDeltaY,
                                                optRefDeltaZ,
                                                optRefDeltaSize,
                                                optRefPositions3DInFormationVec);

            //check if optimized points are feasible
            bool feasible = true;
            for (int j = 0; j < numberOfConstraints && feasible; j++)
            {
                for (int i = 0; i < optRefPositions3DInFormationVec.rows() && feasible; i= i + 3)
                {
                    if ((halfplaneConstraintA(j, 0) * optRefPositions3DInFormationVec(i) +
                         halfplaneConstraintA(j, 1) * optRefPositions3DInFormationVec(i + 1) +
                         halfplaneConstraintA(j, 2) * optRefPositions3DInFormationVec(i + 2)) >
                        (halfplaneConstraintb(j) - agentRadius + FLT_EPSILON))
                    {
                        feasible = false;
                    }
                }
            }

            if (feasible &&
                discountedCost<minCost)
            {
                minCost = discountedCost;

                optDeltaX = optRefDeltaX;
                optDeltaY = optRefDeltaY;
                optDeltaZ = optRefDeltaZ;
                optDeltaSize = optRefDeltaSize;
                optPositions3DInFormationVec = optRefPositions3DInFormationVec;
                optFormationType = formationPtr->GetFormationType();

                retVal = true;
            }
        }

        if (retVal)
        {
            ConvertVectorOf3DPointsToMapOf3DPoints (optPositions3DInFormationVec,
                                                    optVirtualPositions);
        }

        return retVal;
    }


    void
    Optimize3DFormation::PopulateVectorWithFormationInOrderOfPriority(const int numberOfAgents, std::vector <Formation3DBase::Ptr>& formation3DBasePtrVec)
    {
        formation3DBasePtrVec.clear();

        if (m_formation3DPoint1AgentInitialized && m_formation3DPoint1Agent.GetNumberOfAgents() == numberOfAgents)
        {
            Formation3DBase::Ptr ptr = std::make_shared<Formation3DPoint1Agent>(m_formation3DPoint1Agent);
            formation3DBasePtrVec.push_back(ptr);
        }

        if (m_formation3DAbreast2AgentsInitialized && m_formation3DAbreast2Agents.GetNumberOfAgents() == numberOfAgents)
        {
            Formation3DBase::Ptr ptr = std::make_shared<Formation3DAbreast2Agents>(m_formation3DAbreast2Agents);
            formation3DBasePtrVec.push_back(ptr);
        }

        if (m_formation3DTri3AgentsInitialized && m_formation3DTri3Agents.GetNumberOfAgents() == numberOfAgents)
        {
            Formation3DBase::Ptr ptr = std::make_shared<Formation3DTri3Agents>(m_formation3DTri3Agents);
            formation3DBasePtrVec.push_back(ptr);
        }

        if (m_formation3DLine3AgentsInitialized && m_formation3DLine3Agents.GetNumberOfAgents() == numberOfAgents)
        {
            Formation3DBase::Ptr ptr = std::make_shared<Formation3DLine3Agents>(m_formation3DLine3Agents);
            formation3DBasePtrVec.push_back(ptr);
        }
    }

//    void
//    Optimize3DFormation::ConvertVectorOf3DPointsToMatrixOf3DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
//                                                    Eigen::Matrix<double, Eigen::Dynamic, 3>& pointsMat)
//    {
//        size_t numberOfPoints = std::floor( pointsVec.rows()/3 );
//
//        pointsMat.resize(numberOfPoints, Eigen::NoChange);
//
//        for (int i = 0; i < numberOfPoints; i++)
//        {
//            pointsMat(i,0) = pointsVec(i*3);
//            pointsMat(i,1) = pointsVec(i*3 + 1);
//            pointsMat(i,2) = pointsVec(i*3 + 2);
//        }
//    }

    void
    Optimize3DFormation::ConvertVectorOf3DPointsToMapOf3DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
                                                                 std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions)
    {
        optVirtualPositions.clear();

        size_t numberOfPoints = std::floor( pointsVec.rows()/3 );

        for (int i = 0; i < numberOfPoints; i++)
        {
            DistributedFormation::Common::Position position;
            position.x = pointsVec(i*3);
            position.y = pointsVec(i*3 + 1);
            position.z = pointsVec(i*3 + 2);

            optVirtualPositions[i] = position;
        }
    }

}   // namespace Formation3DWithoutRot