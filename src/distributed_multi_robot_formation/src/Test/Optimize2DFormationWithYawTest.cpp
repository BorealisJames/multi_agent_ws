//
// Created by benson on 28/1/21.
//

#include <gtest/gtest.h>
#include <ifopt/ipopt_solver.h>

#include "../Common/Common.h"
#include "../Formation2D/Formation2DWithYaw/Optimizer2DVarsConstrCost.h"
#include "../Formation2D/Formation2DWithYaw/Formation2DAbreast2Agents.h"
#include "../Formation2D/Formation2DWithYaw/Formation2DTri3Agents.h"
#include "../Formation2D/Formation2DWithYaw/Formation2DLine3Agents.h"
#include "../Formation2D/Formation2DWithYaw/Optimize2DFormation.h"

namespace Formation2DWithYaw
{
    TEST (Formation2DWithYaw, JacobianConstraintOnFormation2DAbreast2Agents)
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
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaYaw = 1.57;
        double desiredDeltaSize = 0.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
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
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 2;
        int numberOfVariables = 4;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXPositive;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX + delta, initDeltaY,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions2DInFormationXPositive);

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXNegative;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX - delta, initDeltaY,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions2DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXDelta(6,1);
        positions2DInFormationXDelta = positions2DInFormationXPositive - positions2DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYPositive;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY + delta,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions2DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYNegative;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY - delta,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions2DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYDelta(6,1);
        positions2DInFormationYDelta = positions2DInFormationYPositive - positions2DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingPositive;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw + delta,
                                                        initDeltaSize,
                                                        positions2DInFormationHeadingPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingNegative;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw - delta,
                                                        initDeltaSize,
                                                        positions2DInFormationHeadingNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingDelta(6,1);
        positions2DInFormationHeadingDelta = positions2DInFormationHeadingPositive - positions2DInFormationHeadingNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizePositive;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                        initDeltaYaw,
                                                    initDeltaSize + delta,
                                                        positions2DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeNegative;
        formation2DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                        initDeltaYaw,
                                                    initDeltaSize - delta,
                                                        positions2DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeDelta(6,1);
        positions2DInFormationSizeDelta = positions2DInFormationSizePositive - positions2DInFormationSizeNegative;

        Eigen::MatrixXd expectedJacobian (4,4);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<2; j++)
            {
                expectedJacobian(i*2+j,0) = A(i,0)*positions2DInFormationXDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationXDelta(j*2+1)/(2*delta);
                expectedJacobian(i*2+j,1) = A(i,0)*positions2DInFormationYDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationYDelta(j*2+1)/(2*delta);
                expectedJacobian(i*2+j,2) = A(i,0)*positions2DInFormationHeadingDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationHeadingDelta(j*2+1)/(2*delta);
                expectedJacobian(i*2+j,3) = A(i,0)*positions2DInFormationSizeDelta(j*2)/(2*delta) +
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

    TEST (Formation2DWithYaw, JacobianConstraintOnFormation2DTri3Agents)
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
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaYaw = 1.57;
        double desiredDeltaSize = 0.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
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
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 3;
        int numberOfVariables = 4;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXPositive;
        formation2DTri3Agents.GetFormationPositions(initDeltaX + delta, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationXPositive);

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXNegative;
        formation2DTri3Agents.GetFormationPositions(initDeltaX - delta, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXDelta(6,1);
        positions2DInFormationXDelta = positions2DInFormationXPositive - positions2DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYPositive;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY + delta,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYNegative;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY - delta,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYDelta(6,1);
        positions2DInFormationYDelta = positions2DInFormationYPositive - positions2DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingPositive;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw + delta,
                                                    initDeltaSize,
                                                    positions2DInFormationHeadingPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingNegative;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw - delta,
                                                    initDeltaSize,
                                                    positions2DInFormationHeadingNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingDelta(6,1);
        positions2DInFormationHeadingDelta = positions2DInFormationHeadingPositive - positions2DInFormationHeadingNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizePositive;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize + delta,
                                                    positions2DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeNegative;
        formation2DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize - delta,
                                                    positions2DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeDelta(6,1);
        positions2DInFormationSizeDelta = positions2DInFormationSizePositive - positions2DInFormationSizeNegative;

        Eigen::MatrixXd expectedJacobian (6,4);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<3; j++)
            {
                expectedJacobian(i*3+j,0) = A(i,0)*positions2DInFormationXDelta(j*2)/(2*delta) +
                                                    A(i,1)*positions2DInFormationXDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,1) = A(i,0)*positions2DInFormationYDelta(j*2)/(2*delta) +
                                                    A(i,1)*positions2DInFormationYDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,2) = A(i,0)*positions2DInFormationHeadingDelta(j*2)/(2*delta) +
                                                    A(i,1)*positions2DInFormationHeadingDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,3) = A(i,0)*positions2DInFormationSizeDelta(j*2)/(2*delta) +
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

    TEST (Formation2DWithYaw, JacobianConstraintOnFormation2DLine3Agents)
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
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaYaw = 1.57;
        double desiredDeltaSize = 0.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
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
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 3;
        int numberOfVariables = 4;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXPositive;
        formation2DLine3Agents.GetFormationPositions(initDeltaX + delta, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationXPositive);

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXNegative;
        formation2DLine3Agents.GetFormationPositions(initDeltaX - delta, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationXDelta(6,1);
        positions2DInFormationXDelta = positions2DInFormationXPositive - positions2DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYPositive;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY + delta,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYNegative;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY - delta,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions2DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationYDelta(6,1);
        positions2DInFormationYDelta = positions2DInFormationYPositive - positions2DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingPositive;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw + delta,
                                                    initDeltaSize,
                                                    positions2DInFormationHeadingPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingNegative;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw - delta,
                                                    initDeltaSize,
                                                    positions2DInFormationHeadingNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationHeadingDelta(6,1);
        positions2DInFormationHeadingDelta = positions2DInFormationHeadingPositive - positions2DInFormationHeadingNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizePositive;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize + delta,
                                                    positions2DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeNegative;
        formation2DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY,
                                                    initDeltaYaw,
                                                    initDeltaSize - delta,
                                                    positions2DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions2DInFormationSizeDelta(6,1);
        positions2DInFormationSizeDelta = positions2DInFormationSizePositive - positions2DInFormationSizeNegative;

        Eigen::MatrixXd expectedJacobian (6,4);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<3; j++)
            {
                expectedJacobian(i*3+j,0) = A(i,0)*positions2DInFormationXDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationXDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,1) = A(i,0)*positions2DInFormationYDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationYDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,2) = A(i,0)*positions2DInFormationHeadingDelta(j*2)/(2*delta) +
                                            A(i,1)*positions2DInFormationHeadingDelta(j*2+1)/(2*delta);
                expectedJacobian(i*3+j,3) = A(i,0)*positions2DInFormationSizeDelta(j*2)/(2*delta) +
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

    TEST (Formation2DWithYaw, JacobianCostOnFormation2DAbreast2Agents)
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
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaYaw = 1.57;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
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
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        double x[4] = { initDeltaX, initDeltaY,  initDeltaYaw, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(4, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[4] = {initDeltaX + delta, initDeltaY, initDeltaYaw, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[4] = {initDeltaX - delta, initDeltaY, initDeltaYaw, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[4] = {initDeltaX, initDeltaY + delta, initDeltaYaw, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[4] = {initDeltaX, initDeltaY - delta, initDeltaYaw, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xHeadingPositive[4] = {initDeltaX, initDeltaY, initDeltaYaw + delta, initDeltaSize};
        double costHeadingPositive = nlp.EvaluateCostFunction(xHeadingPositive);
        double xHeadingNegative[4] = {initDeltaX, initDeltaY, initDeltaYaw - delta, initDeltaSize};
        double costHeadingNegative = nlp.EvaluateCostFunction(xHeadingNegative);

        double xSizePositive[4] = {initDeltaX, initDeltaY, initDeltaYaw, initDeltaSize + delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[4] = {initDeltaX, initDeltaY, initDeltaYaw, initDeltaSize - delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::Vector4d expectedGradient;
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costHeadingPositive - costHeadingNegative)/(2*delta);
        expectedGradient(3) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<4; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation2DWithYaw, JacobianCostOnFormation2DTri3Agents)
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
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaYaw = 1.57;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
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
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        double x[4] = { initDeltaX, initDeltaY,  initDeltaYaw, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(4, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[4] = {initDeltaX + delta, initDeltaY, initDeltaYaw, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[4] = {initDeltaX - delta, initDeltaY, initDeltaYaw, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[4] = {initDeltaX, initDeltaY + delta, initDeltaYaw, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[4] = {initDeltaX, initDeltaY - delta, initDeltaYaw, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xHeadingPositive[4] = {initDeltaX, initDeltaY, initDeltaYaw + delta, initDeltaSize};
        double costHeadingPositive = nlp.EvaluateCostFunction(xHeadingPositive);
        double xHeadingNegative[4] = {initDeltaX, initDeltaY, initDeltaYaw - delta, initDeltaSize};
        double costHeadingNegative = nlp.EvaluateCostFunction(xHeadingNegative);

        double xSizePositive[4] = {initDeltaX, initDeltaY, initDeltaYaw, initDeltaSize + delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[4] = {initDeltaX, initDeltaY, initDeltaYaw, initDeltaSize - delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::Vector4d expectedGradient;
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costHeadingPositive - costHeadingNegative)/(2*delta);
        expectedGradient(3) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<4; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation2DWithYaw, JacobianCostOnFormation2DLine3Agents)
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
        double initDeltaYaw = 1.57;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaYaw = 0.785;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
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
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        double x[4] = { initDeltaX, initDeltaY,  initDeltaYaw, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(4, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[4] = {initDeltaX + delta, initDeltaY, initDeltaYaw, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[4] = {initDeltaX - delta, initDeltaY, initDeltaYaw, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[4] = {initDeltaX, initDeltaY + delta, initDeltaYaw, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[4] = {initDeltaX, initDeltaY - delta, initDeltaYaw, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xHeadingPositive[4] = {initDeltaX, initDeltaY, initDeltaYaw + delta, initDeltaSize};
        double costHeadingPositive = nlp.EvaluateCostFunction(xHeadingPositive);
        double xHeadingNegative[4] = {initDeltaX, initDeltaY, initDeltaYaw - delta, initDeltaSize};
        double costHeadingNegative = nlp.EvaluateCostFunction(xHeadingNegative);

        double xSizePositive[4] = {initDeltaX, initDeltaY, initDeltaYaw, initDeltaSize + delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[4] = {initDeltaX, initDeltaY, initDeltaYaw, initDeltaSize - delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::Vector4d expectedGradient;
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costHeadingPositive - costHeadingNegative)/(2*delta);
        expectedGradient(3) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<4; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation2DWithYaw, OptimizedResultOnFormation2DTri3Agents)
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
        double desiredDeltaYaw = 0.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
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
        double optDeltaX, optDeltaY, optDeltaYaw, optDeltaSize;
        DistributedFormation::Common::Formation2DType formation2DType;
        bool optSuccessful;

        optSuccessful = opt.GetOptimizedPositions2DInFormation(numberOfAgents,
                                                               agentRadius,
                                                               weightForGoal,
                                                               weightForRotation,
                                                               weightForSize,
                                                               A,
                                                               b,
                                                               desiredFixedHeight,
                                                               priorityPenalty,
                                                               optVirtualPositions,
                                                               optDeltaX,
                                                               optDeltaY,
                                                               optDeltaYaw,
                                                               optDeltaSize,
                                                               formation2DType);

        EXPECT_TRUE(optSuccessful);

        double distanceFromCentroidToFront =  desiredDistanceInFormation/std::sqrt(3.0);

        double expectedOptDeltaX = (b(0) - agentRadius - distanceFromCentroidToFront) - initX;
        expectedOptDeltaX = std::min(expectedOptDeltaX, desiredDeltaX);
        double errorOptDeltaX = std::abs(optDeltaX - expectedOptDeltaX);
        double errorOptDeltaY = std::abs(optDeltaY - desiredDeltaY);
        double errorOptDeltaYaw = std::abs(optDeltaYaw - desiredDeltaYaw);
        double errorOptDeltaSize = std::abs(optDeltaSize - 0.0);

        EXPECT_LE(errorOptDeltaX, 0.1);
        EXPECT_LE(errorOptDeltaY, 0.1);
        EXPECT_LE(errorOptDeltaYaw, 0.1);
        EXPECT_LE(errorOptDeltaSize, 0.1);
    }
} // namespace DistributedFormation