//
// Created by benson on 28/1/21.
//

#include <gtest/gtest.h>
#include <ifopt/ipopt_solver.h>

#include "../Common/Common.h"
#include "../Formation2D/Formation2DWithoutYaw/Optimizer2DVarsConstrCost.h"
#include "../Formation2D/Formation2DWithoutYaw/Formation2DAbreast2Agents.h"
#include "../Formation2D/Formation2DWithoutYaw/Formation2DTri3Agents.h"
#include "../Formation2D/Formation2DWithoutYaw/Formation2DLine3Agents.h"
#include "../Formation2D/Formation2DWithoutYaw/Optimize2DFormation.h"

namespace Formation2DWithoutYaw
{
    TEST (Formation2DWithoutYaw, JacobianConstraintOnFormation2DAbreast2Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initYaw = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaSize = 0.0;
        double weightForGoal = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,2);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        b(1) = 2.0;

        Formation2DAbreast2Agents formation2DAbreast2Agents;
        formation2DAbreast2Agents.SetDesiredPositionYawAndSize(initX, initY,
                                                            initYaw,
                                                            initSize);
        formation2DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation2DBase::Ptr ptr = std::make_shared<Formation2DAbreast2Agents>(formation2DAbreast2Agents);

        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<Formation2DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation2DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation2DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 2;
        int numberOfVariables = 3;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXPositive;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX + delta, initDeltaY,
                                                        initDeltaSize,
                                                        positions2DInFormationXPositive);

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXNegative;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX - delta, initDeltaY,
                                                        initDeltaSize,
                                                        positions2DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXDelta(6,1);
        positions2DInFormationXDelta = positions2DInFormationXPositive - positions2DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYPositive;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY + delta,
                                                        initDeltaSize,
                                                        positions2DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYNegative;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY - delta,
                                                        initDeltaSize,
                                                        positions2DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYDelta(6,1);
        positions2DInFormationYDelta = positions2DInFormationYPositive - positions2DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizePositive;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaSize + delta,
                                                        positions2DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeNegative;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaSize - delta,
                                                        positions2DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeDelta(6,1);
        positions2DInFormationSizeDelta = positions2DInFormationSizePositive - positions2DInFormationSizeNegative;

        Eigen::MatrixXd expectedJacobian (4,3);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<2; j++)
            {
                expectedJacobian(i*2+j,0) = A(i,0)*positions2DInFormationXDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationXDelta(j*2+1)/(2*delta);
                expectedJacobian(i*2+j,1) = A(i,0)*positions2DInFormationYDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationYDelta(j*2+1)/(2*delta);
                expectedJacobian(i*2+j,2) = A(i,0)*positions2DInFormationSizeDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationSizeDelta(j*2+1)/(2*delta);
            }
        }

        for (int i=0; i<expectedJacobian.rows(); i++)
        {
            for (int j=0; j<expectedJacobian.cols(); j++)
            {
                double diff = expectedJacobian(i,j) - outputJacobian(i,j);
                EXPECT_LE(std::abs(diff), 0.01);
            }
        }

    }

    TEST (Formation2DWithoutYaw, JacobianConstraintOnFormation2DTri3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initYaw = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaSize = 0.0;
        double weightForGoal = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,2);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        b(1) = 2.0;

        Formation2DTri3Agents formation2DTri3Agents;
        formation2DTri3Agents.SetDesiredPositionYawAndSize(initX, initY,
                                                        initYaw,
                                                        initSize);
        formation2DTri3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation2DBase::Ptr ptr = std::make_shared<Formation2DTri3Agents>(formation2DTri3Agents);

        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<Formation2DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation2DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation2DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 3;
        int numberOfVariables = 3;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXPositive;
        formation2DTri3Agents.GetFormationPositions(initDeltaX + delta, initDeltaY,
                                                    initDeltaSize,
                                                    positions2DInFormationXPositive);

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXNegative;
        formation2DTri3Agents.GetFormationPositions(initDeltaX - delta, initDeltaY,
                                                    initDeltaSize,
                                                    positions2DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXDelta(6,1);
        positions2DInFormationXDelta = positions2DInFormationXPositive - positions2DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYPositive;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY + delta,
                                                    initDeltaSize,
                                                    positions2DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYNegative;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY - delta,
                                                    initDeltaSize,
                                                    positions2DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYDelta(6,1);
        positions2DInFormationYDelta = positions2DInFormationYPositive - positions2DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizePositive;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaSize + delta,
                                                    positions2DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeNegative;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaSize - delta,
                                                    positions2DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeDelta(6,1);
        positions2DInFormationSizeDelta = positions2DInFormationSizePositive - positions2DInFormationSizeNegative;

        Eigen::MatrixXd expectedJacobian (6,3);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<3; j++)
            {
                expectedJacobian(i*3+j,0) = A(i,0)*positions2DInFormationXDelta(j*2)/(2*delta) +
                                                    A(i,1)*positions2DInFormationXDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,1) = A(i,0)*positions2DInFormationYDelta(j*2)/(2*delta) +
                                                    A(i,1)*positions2DInFormationYDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,2) = A(i,0)*positions2DInFormationSizeDelta(j*2)/(2*delta) +
                                                    A(i,1)*positions2DInFormationSizeDelta(j*2+1)/(2*delta);
            }
        }

        for (int i=0; i<expectedJacobian.rows(); i++)
        {
            for (int j=0; j<expectedJacobian.cols(); j++)
            {
                double diff = expectedJacobian(i,j) - outputJacobian(i,j);
                EXPECT_LE(std::abs(diff), 0.01);
            }
        }

    }

    TEST (Formation2DWithoutYaw, JacobianConstraintOnFormation2DLine3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initYaw = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaSize = 0.0;
        double weightForGoal = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,2);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        b(1) = 2.0;

        Formation2DLine3Agents formation2DLine3Agents;
        formation2DLine3Agents.SetDesiredPositionYawAndSize(initX, initY,
                                                        initYaw,
                                                        initSize);
        formation2DLine3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation2DBase::Ptr ptr = std::make_shared<Formation2DLine3Agents>(formation2DLine3Agents);

        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<Formation2DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation2DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation2DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 3;
        int numberOfVariables = 3;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXPositive;
        formation2DLine3Agents.GetFormationPositions(initDeltaX + delta, initDeltaY,
                                                    initDeltaSize,
                                                    positions2DInFormationXPositive);

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXNegative;
        formation2DLine3Agents.GetFormationPositions(initDeltaX - delta, initDeltaY,
                                                    initDeltaSize,
                                                    positions2DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXDelta(6,1);
        positions2DInFormationXDelta = positions2DInFormationXPositive - positions2DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYPositive;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY + delta,
                                                    initDeltaSize,
                                                    positions2DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYNegative;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY - delta,
                                                    initDeltaSize,
                                                    positions2DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYDelta(6,1);
        positions2DInFormationYDelta = positions2DInFormationYPositive - positions2DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizePositive;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaSize + delta,
                                                    positions2DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeNegative;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaSize - delta,
                                                    positions2DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeDelta(6,1);
        positions2DInFormationSizeDelta = positions2DInFormationSizePositive - positions2DInFormationSizeNegative;

        Eigen::MatrixXd expectedJacobian (6,3);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<3; j++)
            {
                expectedJacobian(i*3+j,0) = A(i,0)*positions2DInFormationXDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationXDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,1) = A(i,0)*positions2DInFormationYDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationYDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,2) = A(i,0)*positions2DInFormationSizeDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationSizeDelta(j*2+1)/(2*delta);
            }
        }

        for (int i=0; i<expectedJacobian.rows(); i++)
        {
            for (int j=0; j<expectedJacobian.cols(); j++)
            {
                double diff = expectedJacobian(i,j) - outputJacobian(i,j);
                EXPECT_LE(std::abs(diff), 0.01);
            }
        }

    }

    TEST (Formation2DWithoutYaw, JacobianCostOnFormation2DAbreast2Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initYaw = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,2);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        b(1) = 2.0;

        Formation2DAbreast2Agents formation2DAbreast2Agents;
        formation2DAbreast2Agents.SetDesiredPositionYawAndSize(initX, initY,
                                                            initYaw,
                                                            initSize);
        formation2DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation2DBase::Ptr ptr = std::make_shared<Formation2DAbreast2Agents>(formation2DAbreast2Agents);

        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<Formation2DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation2DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation2DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForSize));

        double x[3] = { initDeltaX, initDeltaY,  initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(3, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[3] = {initDeltaX + delta, initDeltaY, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[3] = {initDeltaX - delta, initDeltaY, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[3] = {initDeltaX, initDeltaY + delta, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[3] = {initDeltaX, initDeltaY - delta, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xSizePositive[3] = {initDeltaX, initDeltaY, initDeltaSize + delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[3] = {initDeltaX, initDeltaY, initDeltaSize - delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::Vector4d expectedGradient;
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<3; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation2DWithoutYaw, JacobianCostOnFormation2DTri3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initYaw = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,2);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        b(1) = 2.0;

        Formation2DTri3Agents formation2DTri3Agents;
        formation2DTri3Agents.SetDesiredPositionYawAndSize(initX, initY,
                                                        initYaw,
                                                        initSize);
        formation2DTri3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation2DBase::Ptr ptr = std::make_shared<Formation2DTri3Agents>(formation2DTri3Agents);

        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<Formation2DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation2DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation2DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForSize));

        double x[3] = { initDeltaX, initDeltaY, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(3, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[3] = {initDeltaX + delta, initDeltaY, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[3] = {initDeltaX - delta, initDeltaY, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[3] = {initDeltaX, initDeltaY + delta, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[3] = {initDeltaX, initDeltaY - delta, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xSizePositive[3] = {initDeltaX, initDeltaY, initDeltaSize + delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[3] = {initDeltaX, initDeltaY, initDeltaSize - delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::Vector4d expectedGradient;
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<3; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation2DWithoutYaw, JacobianCostOnFormation2DLine3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;
        bool fixRotation = false;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initYaw = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,2);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        b(1) = 2.0;

        Formation2DLine3Agents formation2DLine3Agents;
        formation2DLine3Agents.SetDesiredPositionYawAndSize(initX, initY,
                                                         initYaw,
                                                         initSize);
        formation2DLine3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation2DBase::Ptr ptr = std::make_shared<Formation2DLine3Agents>(formation2DLine3Agents);

        ifopt::Problem nlp;
        nlp.AddVariableSet(std::make_shared<Formation2DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation2DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation2DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForSize));

        double x[3] = { initDeltaX, initDeltaY, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(3, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[3] = {initDeltaX + delta, initDeltaY, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[3] = {initDeltaX - delta, initDeltaY, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[3] = {initDeltaX, initDeltaY + delta, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[3] = {initDeltaX, initDeltaY - delta, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xSizePositive[3] = {initDeltaX, initDeltaY, initDeltaSize + delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[3] = {initDeltaX, initDeltaY, initDeltaSize - delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::Vector4d expectedGradient;
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<3; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation2DWithoutYaw, OptimizedResultOnFormation2DTri3Agents)
    {
        int numberOfAgents = 3;
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;
        double desiredFixedHeight = 2.0;
        double priorityPenalty = 0.0;

        double initX = 10.0;
        double initY = 0.0;
        double initYaw = 0.0;
        double initSize = 1.0;

        double desiredDeltaX = 0.0;
        double desiredDeltaY = 0.0;
        double weightForGoal = 1.0;
        double weightForSize = 100.0;

        Eigen::Matrix<double, Eigen::Dynamic, 2> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(1,2);
        b.resize(1,1);

        A(0,0) = 1;
        A(0,1) = 0;
        b(0) = 4.0;

        Formation2DTri3Agents formation2DTri3Agents;
        formation2DTri3Agents.SetDesiredPositionYawAndSize(initX, initY,
                                                        initYaw,
                                                        initSize);
        formation2DTri3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation2DBase::Ptr ptr = std::make_shared<Formation2DTri3Agents>(formation2DTri3Agents);

        Optimize2DFormation opt;
        opt.SetFormation2DTri3Agents(formation2DTri3Agents);

        std::unordered_map<uint32_t, DistributedFormation::Common::Position> optVirtualPositions;
        double optDeltaX, optDeltaY, optDeltaSize;
        DistributedFormation::Common::Formation2DType formation2DType;
        bool optSuccessful;

        optSuccessful = opt.GetOptimizedPositions2DInFormation(numberOfAgents,
                                                               agentRadius,
                                                               weightForGoal,
                                                               weightForSize,
                                                               A,
                                                               b,
                                                               desiredFixedHeight,
                                                               priorityPenalty,
                                                               optVirtualPositions,
                                                               optDeltaX,
                                                               optDeltaY,
                                                               optDeltaSize,
                                                               formation2DType);

        EXPECT_TRUE(optSuccessful);

        double distanceFromCentroidToFront =  desiredDistanceInFormation/std::sqrt(3.0);

        double expectedOptDeltaX = (b(0) - agentRadius - distanceFromCentroidToFront) - initX;
        expectedOptDeltaX = std::min(expectedOptDeltaX, desiredDeltaX);
        double errorOptDeltaX = std::abs(optDeltaX - expectedOptDeltaX);
        double errorOptDeltaY = std::abs(optDeltaY - desiredDeltaY);
        double errorOptDeltaSize = std::abs(optDeltaSize - 0.0);

        EXPECT_LE(errorOptDeltaX, 0.1);
        EXPECT_LE(errorOptDeltaY, 0.1);
        EXPECT_LE(errorOptDeltaSize, 0.1);
    }
} // namespace DistributedFormation