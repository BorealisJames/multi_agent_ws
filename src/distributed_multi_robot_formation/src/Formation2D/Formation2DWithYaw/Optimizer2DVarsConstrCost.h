//
// Created by benson on 26/1/21.
//

#pragma once

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include "../../Common/Common.h"
#include "Formation2DBase.h"

namespace Formation2DWithYaw
{

class Formation2DVariables : public ifopt::VariableSet
{
public:
    Formation2DVariables(const double desiredDistanceInFormation,
                         const double agentRadius)
    : Formation2DVariables(desiredDistanceInFormation,
                           agentRadius,
                           "var_formation2d")
    {}

    Formation2DVariables(const double desiredDistanceInFormation,
                         const double agentRadius,
                         const std::string& name)
    : VariableSet(4, name)
    , m_desiredDistanceInFormation(desiredDistanceInFormation)
    , m_agentRadius(agentRadius)
    , m_deltaX(0)
    , m_deltaY(0)
    , m_deltaYaw(0)
    , m_deltaSize(0)
    {
    }

    //set desired distance and agent radius
    void SetDesiredDistanceInFormationAndAgentRadius(const double desiredDistanceInFormation,
                                                     const double agentRadius)
    {
        m_desiredDistanceInFormation = desiredDistanceInFormation;
        m_agentRadius = agentRadius;
    }

    void SetVariables(const VectorXd& x) override
    {
        m_deltaX = x(0);
        m_deltaY = x(1);
        m_deltaYaw = x(2);
        m_deltaSize = x(3);
    }

    VectorXd GetValues() const override
    {
        return Eigen::Vector4d(m_deltaX, m_deltaY, m_deltaYaw, m_deltaSize);
    }

    VecBound GetBounds() const override
    {
        double sizeLowerBound = (2*m_agentRadius) / m_desiredDistanceInFormation;

        VecBound bounds(GetRows());
        bounds.at(0) = ifopt::NoBound;
        bounds.at(1) = ifopt::NoBound;
        bounds.at(2) = ifopt::NoBound;
        bounds.at(3) = ifopt::Bounds(sizeLowerBound-1.0, ifopt::inf);
        return bounds;
    }


private:
    double m_deltaX, m_deltaY, m_deltaYaw, m_deltaSize;

    double m_desiredDistanceInFormation, m_agentRadius;
};

class Formation2DStaticConstraint : public ifopt::ConstraintSet
{
public:
    Formation2DStaticConstraint(const Eigen::Matrix<double, Eigen::Dynamic, 2>& A,
                                const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
                                const double agentRadius,
                                const Formation2DBase::Ptr& formation)
   : Formation2DStaticConstraint(A, b, agentRadius, formation, "constraints_formation2d")
   {}

    Formation2DStaticConstraint(const Eigen::Matrix<double, Eigen::Dynamic, 2>& A,
                                const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
                                const double agentRadius,
                                const Formation2DBase::Ptr& formation,
                                const std::string& name)
   : ConstraintSet(b.rows()* formation->GetNumberOfAgents(), name)
   , m_A(A)
   , m_b(b)
   , m_agentRadius(agentRadius)
   {
       m_formationPtr = formation;
   }

    VectorXd GetValues() const override
    {
        Eigen::Vector4d x = GetVariables()->GetComponent("var_formation2d")->GetValues();

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormation;
        m_formationPtr->GetFormationPositions(x(0), x(1),
                                              x(2),
                                              x(3),
                                              positions2DInFormation);

        VectorXd g(GetRows());

        size_t numberOfLinearConstraintRows = m_A.rows();
        uint32_t numberOfPoints = m_formationPtr->GetNumberOfAgents();

        for (int i=0; i < numberOfLinearConstraintRows; ++i)
        {
            for (int p=0; p < numberOfPoints; ++p)
            {
                int constraintIndex = i*numberOfPoints+p;

                g(constraintIndex) = m_A(i,0) * positions2DInFormation(p*2) +
                                     m_A(i,1) * positions2DInFormation(p*2+1);
            }
        }

        return g;
    }

    VecBound GetBounds() const override
    {
        VecBound b(GetRows());

        size_t numberOfLinearConstraintRows = m_b.rows();
        uint32_t numberOfPoints = m_formationPtr->GetNumberOfAgents();

        for (int i=0; i < numberOfLinearConstraintRows; ++i)
        {
            for (int p=0; p < numberOfPoints; ++p)
            {
                int constraintIndex = i*numberOfPoints+p;
                b.at(constraintIndex) = ifopt::Bounds(-ifopt::inf, m_b(i) - m_agentRadius);
            }
        }

        return b;
    }

    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        if (var_set == "var_formation2d")
        {
            Eigen::Vector4d x = GetVariables()->GetComponent("var_formation2d")->GetValues();

            Eigen::Matrix<double, Eigen::Dynamic, 4> positions2DJacobian;
            m_formationPtr->GetFormationJacobian (x(0), x(1),
                                                x(2),
                                                x(3),
                                                positions2DJacobian);

            size_t numberOfLinearConstraintRows = m_A.rows();
            uint32_t numberOfPoints = m_formationPtr->GetNumberOfAgents();

            for (int i=0; i < numberOfLinearConstraintRows; ++i)
            {
                for (int p=0; p < numberOfPoints; ++p)
                {
                    int constraintIndex = i*numberOfPoints+p;

                    for (int j=0; j < 4; j++)
                    {
                        jac_block.coeffRef(constraintIndex, j) = m_A(i,0) * positions2DJacobian(p*2,j) +
                                                                 m_A(i,1) * positions2DJacobian(p*2+1,j);

                    }
                }
            }
        }
    }

private:
    Eigen::Matrix<double, Eigen::Dynamic, 2> m_A;
    Eigen::Matrix<double, Eigen::Dynamic, 1> m_b;
    double m_agentRadius;
    std::shared_ptr<Formation2DBase> m_formationPtr;
};

class Formation2DCost : public ifopt::CostTerm
{
public:
    Formation2DCost(const double desiredDeltaX,
                    const double desiredDeltaY,
                    const double desiredDeltaYaw,
                    const double desiredDeltaSize,
                    const double weightForGoal,
                    const double weightForYaw,
                    const double weightForSize)
    : Formation2DCost(desiredDeltaX,
                      desiredDeltaY,
                      desiredDeltaYaw,
                      desiredDeltaSize,
                      weightForGoal,
                      weightForYaw,
                      weightForSize,
                      "cost_formation2d") {}

    Formation2DCost(const double desiredDeltaX,
                    const double desiredDeltaY,
                    const double desiredDeltaYaw,
                    const double desiredDeltaSize,
                    const double weightForGoal,
                    const double weightForYaw,
                    const double weightForSize,
                    const std::string& name)
    : CostTerm(name)
    , m_desiredDeltaX(desiredDeltaX)
    , m_desiredDeltaY(desiredDeltaY)
    , m_desiredDeltaYaw(desiredDeltaYaw)
    , m_desiredDeltaSize(desiredDeltaSize)
    , m_weightForGoal(weightForGoal)
    , m_weightForYaw(weightForYaw)
    , m_weightForSize(weightForSize)
    {
        m_desiredDeltaYaw = DistributedFormation::Common::MinusPiToPi(m_desiredDeltaYaw);

        double normlizeWeights = 1 / (m_weightForGoal + m_weightForYaw + m_weightForSize);
        m_weightForGoal *= normlizeWeights;
        m_weightForYaw *= normlizeWeights;
        m_weightForSize *= normlizeWeights;
    }

    double GetCost() const override
    {
        Eigen::Vector4d x = GetVariables()->GetComponent("var_formation2d")->GetValues();
        double optDX = x(0);
        double optDY = x(1);
        double optDYaw = x(2);
        double optDSize = x(3);

        double differenceInDesiredPositionSquared = (std::pow(m_desiredDeltaX-optDX,2) +
                                                     std::pow(m_desiredDeltaY-optDY,2));

        double differenceInDesiredYaw = m_desiredDeltaYaw - optDYaw;

        double differenceInDesiredSize = m_desiredDeltaSize - optDSize;

        double cost = m_weightForGoal * (differenceInDesiredPositionSquared) +
                      m_weightForYaw * (differenceInDesiredYaw * differenceInDesiredYaw) +
                      m_weightForSize * (differenceInDesiredSize * differenceInDesiredSize);

        return cost;
    }

    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
    {
        if (var_set == "var_formation2d") {
            Eigen::Vector4d x = GetVariables()->GetComponent("var_formation2d")->GetValues();
            double optDX = x(0);
            double optDY = x(1);
            double optDYaw = x(2);
            double optDSize = x(3);

            jac.coeffRef(0, 0) = m_weightForGoal*(-2*m_desiredDeltaX + 2*optDX);
            jac.coeffRef(0, 1) = m_weightForGoal*(-2*m_desiredDeltaY + 2*optDY);
            jac.coeffRef(0, 2) = m_weightForYaw*(-2*m_desiredDeltaYaw + 2*optDYaw);
            jac.coeffRef(0, 3) = m_weightForSize*(-2*m_desiredDeltaSize + 2*optDSize);
        }
    }

private:
    double m_desiredDeltaX;
    double m_desiredDeltaY;
    double m_desiredDeltaYaw;
    double m_desiredDeltaSize;

    double m_weightForGoal;
    double m_weightForYaw;
    double m_weightForSize;
};

}   // namespace Formation2DWithYaw
