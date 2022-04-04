//
// Created by benson on 5/8/21.
//

#pragma once

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include "../../Common/Common.h"
#include "Formation3DBase.h"

namespace Formation3DWithOnlyYaw
{

class Formation3DVariables : public ifopt::VariableSet
{
public:
    Formation3DVariables(const double desiredDistanceInFormation,
                         const double agentRadius)
    : Formation3DVariables(desiredDistanceInFormation,
                           agentRadius,
                           "var_formation3d")
    {}

    Formation3DVariables(const double desiredDistanceInFormation,
                         const double agentRadius,
                         const std::string& name)
    : VariableSet(5, name)
    , m_desiredDistanceInFormation(desiredDistanceInFormation)
    , m_agentRadius(agentRadius)
    , m_deltaX(0)
    , m_deltaY(0)
    , m_deltaZ(0)
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
        m_deltaZ = x(2);
        m_deltaYaw = x(3);
        m_deltaSize = x(4);
    }

    VectorXd GetValues() const override
    {
        Eigen::VectorXd x(GetRows());
        x(0) = m_deltaX;
        x(1) = m_deltaY;
        x(2) = m_deltaZ;
        x(3) = m_deltaYaw;
        x(4) = m_deltaSize;

        return x;
    }

    VecBound GetBounds() const override
    {
        double sizeLowerBound = (2*m_agentRadius) / m_desiredDistanceInFormation;

        VecBound bounds(GetRows());
        bounds.at(0) = ifopt::NoBound;
        bounds.at(1) = ifopt::NoBound;
        bounds.at(2) = ifopt::NoBound;

        bounds.at(3) = ifopt::NoBound;

        bounds.at(4) = ifopt::Bounds(sizeLowerBound-1.0, ifopt::inf);

        return bounds;
    }


private:
    double m_deltaX, m_deltaY, m_deltaZ, m_deltaYaw, m_deltaSize;

    double m_desiredDistanceInFormation, m_agentRadius;
};

class Formation3DStaticConstraint : public ifopt::ConstraintSet
{
public:
    Formation3DStaticConstraint(const Eigen::Matrix<double, Eigen::Dynamic, 3>& A,
                                const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
                                const double agentRadius,
                                const Formation3DBase::Ptr& formation)
   : Formation3DStaticConstraint(A, b, agentRadius, formation, "constraints_formation3d")
   {}

    Formation3DStaticConstraint(const Eigen::Matrix<double, Eigen::Dynamic, 3>& A,
                                const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
                                const double agentRadius,
                                const Formation3DBase::Ptr& formation,
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
        Eigen::VectorXd x = GetVariables()->GetComponent("var_formation3d")->GetValues();

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormation;
        m_formationPtr->GetFormationPositions(x(0), x(1), x(2),
                                              x(3), x(4),
                                              positions3DInFormation);

        VectorXd g(GetRows());

        size_t numberOfLinearConstraintRows = m_A.rows();
        uint32_t numberOfPoints = m_formationPtr->GetNumberOfAgents();

        int constraintIndex = 0;
        for (int i=0; i < numberOfLinearConstraintRows; ++i)
        {
            for (int p=0; p < numberOfPoints; ++p)
            {
                //constraintIndex at this iteration is equal to i*numberOfPoints+p
                g(constraintIndex) = m_A(i,0) * positions3DInFormation(p*3) +
                                     m_A(i,1) * positions3DInFormation(p*3+1) +
                                     m_A(i,2) * positions3DInFormation(p*3+2);

                constraintIndex++;
            }
        }

        return g;
    }

    VecBound GetBounds() const override
    {
        VecBound b(GetRows());

        size_t numberOfLinearConstraintRows = m_b.rows();
        uint32_t numberOfPoints = m_formationPtr->GetNumberOfAgents();

        int constraintIndex = 0;
        for (int i=0; i < numberOfLinearConstraintRows; ++i)
        {
            for (int p=0; p < numberOfPoints; ++p)
            {
                //constraintIndex at this iteration is equal to i*numberOfPoints+p
                b.at(constraintIndex) = ifopt::Bounds(-ifopt::inf, m_b(i) - m_agentRadius);
                constraintIndex++;
            }
        }

        return b;
    }

    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
    {
        if (var_set == "var_formation3d")
        {
            Eigen::VectorXd x = GetVariables()->GetComponent("var_formation3d")->GetValues();

            Eigen::Matrix<double, Eigen::Dynamic, 5> positions3DJacobian;
            m_formationPtr->GetFormationJacobian (x(0), x(1), x(2),
                                                  x(3), x(4),
                                                  positions3DJacobian);

            size_t numberOfLinearConstraintRows = m_A.rows();
            uint32_t numberOfPoints = m_formationPtr->GetNumberOfAgents();

            int constraintIndex = 0;
            for (int i=0; i < numberOfLinearConstraintRows; ++i)
            {
                for (int p=0; p < numberOfPoints; ++p)
                {
                    //constraintIndex at this iteration is equal to i*numberOfPoints+p
                    for (int j=0; j < 5; j++)
                    {
                        jac_block.coeffRef(constraintIndex, j) = m_A(i,0) * positions3DJacobian(p*3,j) +
                                                                 m_A(i,1) * positions3DJacobian(p*3+1,j) +
                                                                 m_A(i,2) * positions3DJacobian(p*3+2,j);
                    }
                    constraintIndex++;
                }
            }
        }
    }

private:
    Eigen::Matrix<double, Eigen::Dynamic, 3> m_A;
    Eigen::Matrix<double, Eigen::Dynamic, 1> m_b;
    double m_agentRadius;
    std::shared_ptr<Formation3DBase> m_formationPtr;
};

class Formation3DCost : public ifopt::CostTerm
{
public:
    Formation3DCost(const double desiredDeltaX,
                    const double desiredDeltaY,
                    const double desiredDeltaZ,
                    const double desiredDeltaYaw,
                    const double desiredDeltaSize,
                    const double weightForGoal,
                    const double weightForRotation,
                    const double weightForSize)
    : Formation3DCost(desiredDeltaX,
                      desiredDeltaY,
                      desiredDeltaZ,
                      desiredDeltaYaw,
                      desiredDeltaSize,
                      weightForGoal,
                      weightForRotation,
                      weightForSize,
                      "cost_formation3d") {}

    Formation3DCost(const double desiredDeltaX,
                    const double desiredDeltaY,
                    const double desiredDeltaZ,
                    const double desiredDeltaYaw,
                    const double desiredDeltaSize,
                    const double weightForGoal,
                    const double weightForRotation,
                    const double weightForSize,
                    const std::string& name)
    : CostTerm(name)
    , m_desiredDeltaX(desiredDeltaX)
    , m_desiredDeltaY(desiredDeltaY)
    , m_desiredDeltaZ(desiredDeltaZ)
    , m_desiredDeltaYaw(desiredDeltaYaw)
    , m_desiredDeltaSize(desiredDeltaSize)
    , m_weightForGoal(weightForGoal)
    , m_weightForRotation(weightForRotation)
    , m_weightForSize(weightForSize)
    {
        double normlizeWeights = 1 / (m_weightForGoal + m_weightForRotation + m_weightForSize);
        m_weightForGoal *= normlizeWeights;
        m_weightForRotation *= normlizeWeights;
        m_weightForSize *= normlizeWeights;
    }

    double GetCost() const override
    {
        Eigen::VectorXd x = GetVariables()->GetComponent("var_formation3d")->GetValues();
        double optDX = x(0);
        double optDY = x(1);
        double optDZ = x(2);
        double optDYaw = x(3);
        double optDSize = x(4);

        double differenceInDesiredPositionSquared = (std::pow(m_desiredDeltaX-optDX,2) +
                                                       std::pow(m_desiredDeltaY-optDY,2) +
                                                       std::pow(m_desiredDeltaZ-optDZ,2));

        double differenceInDesiredRotation = m_desiredDeltaYaw - optDYaw;

        double differenceInDesiredSize = m_desiredDeltaSize - optDSize;


        double cost = m_weightForGoal * (differenceInDesiredPositionSquared) +
                      m_weightForRotation * (differenceInDesiredRotation * differenceInDesiredRotation) +
                      m_weightForSize * (differenceInDesiredSize * differenceInDesiredSize);

        return cost;
    }

    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
    {
        if (var_set == "var_formation3d") {
            Eigen::VectorXd x = GetVariables()->GetComponent("var_formation3d")->GetValues();
            double optDX = x(0);
            double optDY = x(1);
            double optDZ = x(2);
            double optDYaw = x(3);
            double optDSize = x(4);

            jac.coeffRef(0, 0) = m_weightForGoal*(-2*m_desiredDeltaX + 2*optDX);
            jac.coeffRef(0, 1) = m_weightForGoal*(-2*m_desiredDeltaY + 2*optDY);
            jac.coeffRef(0, 2) = m_weightForGoal*(-2*m_desiredDeltaZ + 2*optDZ);

            jac.coeffRef(0, 3) = m_weightForRotation*(-2*m_desiredDeltaYaw + 2*optDYaw);

            jac.coeffRef(0, 4) = m_weightForSize*(-2*m_desiredDeltaSize + 2*optDSize);
        }
    }

private:
    double m_desiredDeltaX;
    double m_desiredDeltaY;
    double m_desiredDeltaZ;
    double m_desiredDeltaYaw;
    double m_desiredDeltaSize;

    double m_weightForGoal;
    double m_weightForRotation;
    double m_weightForSize;
};

}   // namespace Formation3DWithOnlyYaw
