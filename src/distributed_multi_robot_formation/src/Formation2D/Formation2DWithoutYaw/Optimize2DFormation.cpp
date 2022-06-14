//
// Created by benson on 25/1/21.
//

#include "Optimize2DFormation.h"

namespace Formation2DWithoutYaw
{

    Optimize2DFormation::Optimize2DFormation()
    : m_formation2DPoint1AgentInitialized(false)
    , m_formation2DAbreast2AgentsInitialized(false)
    , m_formation2DLine3AgentsInitialized(false)
    , m_formation2DTri3AgentsInitialized(false)
    {}

    void Optimize2DFormation::SetFormation2DPoint1Agent (const Formation2DPoint1Agent& formation2DPoint1Agent)
    {
        m_formation2DPoint1Agent = formation2DPoint1Agent;
        m_formation2DPoint1AgentInitialized = true;
    }

    void
    Optimize2DFormation::SetFormation2DAbreast2Agents (const Formation2DAbreast2Agents& formation2DAbreast2Agents)
    {
        m_formation2DAbreast2Agents = formation2DAbreast2Agents;
        m_formation2DAbreast2AgentsInitialized = true;
    }

    void
    Optimize2DFormation::SetFormation2DLine3Agents (const Formation2DLine3Agents& formation2DLine3Agents)
    {
        m_formation2DLine3Agents = formation2DLine3Agents;
        m_formation2DLine3AgentsInitialized = true;
    }

    void
    Optimize2DFormation::SetFormation2DTri3Agents (const Formation2DTri3Agents& formation2DTri3Agents)
    {
        m_formation2DTri3Agents = formation2DTri3Agents;
        m_formation2DTri3AgentsInitialized = true;
    }

    void
    Optimize2DFormation::RemoveAllFormations ()
    {
        m_formation2DPoint1AgentInitialized = false;
        m_formation2DAbreast2AgentsInitialized = false;
        m_formation2DLine3AgentsInitialized = false;
        m_formation2DTri3AgentsInitialized = false;
    }

    void
    Optimize2DFormation::RemoveFormation2DPoint1Agent ()
    {
        m_formation2DPoint1AgentInitialized = false;
    }

    void
    Optimize2DFormation::RemoveFormation2DAbreast2Agents ()
    {
        m_formation2DAbreast2AgentsInitialized = false;
    }

    void
    Optimize2DFormation::RemoveFormation2DLine3Agents ()
    {
        m_formation2DLine3AgentsInitialized = false;
    }

    void
    Optimize2DFormation::RemoveFormation2DTri3Agents ()
    {
        m_formation2DTri3AgentsInitialized = false;
    }

    bool
    Optimize2DFormation::GetOptimizedPositions2DInFormation (const int numberOfAgents,
                                                             const double agentRadius,
                                                             const double weightForDesiredPosition,
                                                             const double weightForDesiredSize,
                                                             const Eigen::Matrix<double, Eigen::Dynamic, 2>& halfplaneConstraintA,
                                                             const Eigen::Matrix<double, Eigen::Dynamic, 1>& halfplaneConstraintb,
                                                             const double desiredFixedHeight,
                                                             const double priorityPenalty,
                                                             std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions,
                                                             double& optDeltaX,
                                                             double& optDeltaY,
                                                             double& optDeltaSize,
                                                             DistributedFormation::Common::Formation2DType& optFormationType)
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
        std::vector <Formation2DBase::Ptr> formation2DBasePtrVec;
        PopulateVectorWithFormationInOrderOfPriority(numberOfAgents, formation2DBasePtrVec);

        Eigen::Matrix<double, Eigen::Dynamic, 1> optPositions2DInFormationVec;
        double minCost = DBL_MAX;

        for (int f = 0; f < formation2DBasePtrVec.size(); f++)
        {
            Formation2DBase::Ptr formationPtr = formation2DBasePtrVec.at(f);
            double desiredDistanceInFormation = formationPtr->GetDesiredDistanceBetweenAgents();

            ifopt::Problem nlp;
            nlp.AddVariableSet(std::make_shared<Formation2DVariables>(desiredDistanceInFormation,
                                                                      agentRadius));
            nlp.AddConstraintSet(std::make_shared<Formation2DStaticConstraint>(halfplaneConstraintA,
                                                                               halfplaneConstraintb,
                                                                               agentRadius,
                                                                               formationPtr));
            nlp.AddCostSet(std::make_shared<Formation2DCost>(0.0,
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
            double optRefDeltaSize = optVaraiblesVec(2);

            double optVaraiblesArray[3] = {optRefDeltaX,
                                           optRefDeltaY,
                                           optRefDeltaSize};
            double cost = nlp.EvaluateCostFunction(optVaraiblesArray);
            double discountedCost = (f * priorityPenalty) + cost;

            std::cout << "f: " << f
                        << ", discountedCost: " << discountedCost << std::endl;

            Eigen::Matrix<double, Eigen::Dynamic, 1> optRefPositions2DInFormationVec;
            formationPtr->GetFormationPositions(optRefDeltaX,
                                                optRefDeltaY,
                                                optRefDeltaSize,
                                                optRefPositions2DInFormationVec);

            //check if optimized points are feasible
            bool feasible = true;
            for (int j = 0; j < numberOfConstraints && feasible; j++)
            {
                for (int i = 0; i < optRefPositions2DInFormationVec.rows() && feasible; i= i + 2)
                {
                    if ((halfplaneConstraintA(j, 0) * optRefPositions2DInFormationVec(i) +
                         halfplaneConstraintA(j, 1) * optRefPositions2DInFormationVec(i + 1)) >
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
                optDeltaSize = optRefDeltaSize;
                optPositions2DInFormationVec = optRefPositions2DInFormationVec;
                optFormationType = formationPtr->GetFormationType();

                retVal = true;
            }
        }

        if (retVal)
        {
            ConvertVectorOf2DPointsToMapOf2DPoints (optPositions2DInFormationVec,
                                                    desiredFixedHeight,
                                                    optVirtualPositions);
        }

        return retVal;
    }


    void
    Optimize2DFormation::PopulateVectorWithFormationInOrderOfPriority(const int numberOfAgents,
                                                                      std::vector <Formation2DBase::Ptr>& formation2DBasePtrVec)
    {
        formation2DBasePtrVec.clear();

        if (m_formation2DPoint1AgentInitialized && m_formation2DPoint1Agent.GetNumberOfAgents() == numberOfAgents)
        {
            Formation2DBase::Ptr ptr = std::make_shared<Formation2DPoint1Agent>(m_formation2DPoint1Agent);
            formation2DBasePtrVec.push_back(ptr);
        }

        if (m_formation2DAbreast2AgentsInitialized && m_formation2DAbreast2Agents.GetNumberOfAgents() == numberOfAgents)
        {
            Formation2DBase::Ptr ptr = std::make_shared<Formation2DAbreast2Agents>(m_formation2DAbreast2Agents);
            formation2DBasePtrVec.push_back(ptr);
        }

        if (m_formation2DTri3AgentsInitialized && m_formation2DTri3Agents.GetNumberOfAgents() == numberOfAgents)
        {
            Formation2DBase::Ptr ptr = std::make_shared<Formation2DTri3Agents>(m_formation2DTri3Agents);
            formation2DBasePtrVec.push_back(ptr);
        }

        if (m_formation2DLine3AgentsInitialized && m_formation2DLine3Agents.GetNumberOfAgents() == numberOfAgents)
        {
            Formation2DBase::Ptr ptr = std::make_shared<Formation2DLine3Agents>(m_formation2DLine3Agents);
            formation2DBasePtrVec.push_back(ptr);
        }
    }

//    void
//    Optimize2DFormation::ConvertVectorOf2DPointsToMatrixOf2DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
//                                                    Eigen::Matrix<double, Eigen::Dynamic, 2>& pointsMat)
//    {
//        size_t numberOfPoints = std::floor( pointsVec.rows()/2 );
//
//        pointsMat.resize(numberOfPoints, Eigen::NoChange);
//
//        for (int i = 0; i < numberOfPoints; i++)
//        {
//            pointsMat(i,0) = pointsVec(i*2);
//            pointsMat(i,1) = pointsVec(i*2 + 1);
//        }
//    }

    void
    Optimize2DFormation::ConvertVectorOf2DPointsToMapOf2DPoints (const Eigen::Matrix<double, Eigen::Dynamic, 1>& pointsVec,
                                                                const double desiredHeight,
                                                                std::unordered_map<uint32_t, DistributedFormation::Common::Position>& optVirtualPositions)
    {
        optVirtualPositions.clear();

        size_t numberOfPoints = std::floor( pointsVec.rows()/2 );

        for (int i = 0; i < numberOfPoints; i++)
        {
            DistributedFormation::Common::Position position;
            position.x = pointsVec(i*2);
            position.y = pointsVec(i*2 + 1);
            position.z = desiredHeight;

            optVirtualPositions[i] = position;
        }
    }

}   // namespace Formation2DWithoutYaw