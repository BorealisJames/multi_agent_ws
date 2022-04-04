//
// Created by benson on 17/8/21.
//

#include <gtest/gtest.h>
#include <ifopt/ipopt_solver.h>

#include "../Common/Common.h"
#include "../Formation3D/Formation3DWithOnlyYaw/Optimizer3DVarsConstrCost.h"
#include "../Formation3D/Formation3DWithOnlyYaw/Formation3DAbreast2Agents.h"
#include "../Formation3D/Formation3DWithOnlyYaw/Formation3DTri3Agents.h"
#include "../Formation3D/Formation3DWithOnlyYaw/Formation3DLine3Agents.h"
#include "../Formation3D/Formation3DWithOnlyYaw/Optimize3DFormation.h"

namespace Formation3DWithOnlyYaw
{
    TEST (Formation3DWithOnlyYaw, JacobianConstraintOnFormation3DAbreast2Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initZ = 0.0;
        double initQw = 1.0;
        double initQx = 0.0;
        double initQy = 0.0;
        double initQz = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaZ = 0.0;
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaZ = 0.0;
        double desiredDeltaYaw = 0.3;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,3);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        A(0,2) = 0;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        A(1,2) = 0;
        b(1) = 2.0;

        Formation3DAbreast2Agents formation3DAbreast2Agents;
        formation3DAbreast2Agents.SetDesiredPositionRotationAndSize(initX, initY, initZ,
                                                                    initQw, initQx, initQy, initQz,
                                                                    initSize);
        formation3DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation3DBase::Ptr ptr = std::make_shared<Formation3DAbreast2Agents>(formation3DAbreast2Agents);

        ifopt::Problem nlp;

        nlp.AddVariableSet(std::make_shared<Formation3DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation3DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation3DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaZ,
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 2;
        int numberOfVariables = 5;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXPositive;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX + delta, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationXPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXNegative;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX - delta, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXDelta(6,1);
        positions3DInFormationXDelta = positions3DInFormationXPositive - positions3DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYPositive;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY + delta, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYNegative;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY - delta, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYDelta(6,1);
        positions3DInFormationYDelta = positions3DInFormationYPositive - positions3DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZPositive;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ + delta,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationZPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZNegative;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ - delta,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationZNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZDelta(6,1);
        positions3DInFormationZDelta = positions3DInFormationZPositive - positions3DInFormationZNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawPositive;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw + delta,
                                                        initDeltaSize,
                                                        positions3DInFormationYawPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawNegative;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw - delta,
                                                        initDeltaSize,
                                                        positions3DInFormationYawNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawDelta(6,1);
        positions3DInFormationYawDelta = positions3DInFormationYawPositive - positions3DInFormationYawNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizePositive;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize + delta,
                                                        positions3DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizeNegative;
        formation3DAbreast2Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize - delta,
                                                        positions3DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizeDelta(6,1);
        positions3DInFormationSizeDelta = positions3DInFormationSizePositive - positions3DInFormationSizeNegative;

        //ignore the last constraint which is to have unit quat
        Eigen::MatrixXd expectedJacobian (numberOfConstraints*numberOfPoints,numberOfVariables);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<2; j++)
            {
                expectedJacobian(i*2+j,0) = A(i,0)*positions3DInFormationXDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationXDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationXDelta(j*3+2)/(2*delta);
                expectedJacobian(i*2+j,1) = A(i,0)*positions3DInFormationYDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationYDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationYDelta(j*3+2)/(2*delta);
                expectedJacobian(i*2+j,2) = A(i,0)*positions3DInFormationZDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationZDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationZDelta(j*3+2)/(2*delta);
                expectedJacobian(i*2+j,3) = A(i,0)*positions3DInFormationYawDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationYawDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationYawDelta(j*3+2)/(2*delta);
                expectedJacobian(i*2+j,4) = A(i,0)*positions3DInFormationSizeDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationSizeDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationSizeDelta(j*3+2)/(2*delta);
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

    TEST (Formation3DWithOnlyYaw, JacobianConstraintOnFormation3DTri3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initZ = 0.0;
        double initQw = 1.0;
        double initQx = 0.0;
        double initQy = 0.0;
        double initQz = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaZ = 0.0;
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaZ = 0.0;
        double desiredDeltaYaw = 0.3;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,3);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        A(0,2) = 0;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        A(1,2) = 0;
        b(1) = 2.0;

        Formation3DTri3Agents formation3DTri3Agents;
        formation3DTri3Agents.SetDesiredPositionRotationAndSize(initX, initY, initZ,
                                                                initQw, initQx, initQy, initQz,
                                                                initSize);
        formation3DTri3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation3DBase::Ptr ptr = std::make_shared<Formation3DTri3Agents>(formation3DTri3Agents);

        ifopt::Problem nlp;

        nlp.AddVariableSet(std::make_shared<Formation3DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation3DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation3DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaZ,
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 3;
        int numberOfVariables = 5;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXPositive;
        formation3DTri3Agents.GetFormationPositions(initDeltaX + delta, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationXPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXNegative;
        formation3DTri3Agents.GetFormationPositions(initDeltaX - delta, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXDelta(9,1);
        positions3DInFormationXDelta = positions3DInFormationXPositive - positions3DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYPositive;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY + delta, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYNegative;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY - delta, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYDelta(9,1);
        positions3DInFormationYDelta = positions3DInFormationYPositive - positions3DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZPositive;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ + delta,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationZPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZNegative;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ - delta,
                                                        initDeltaYaw,
                                                        initDeltaSize,
                                                        positions3DInFormationZNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZDelta(9,1);
        positions3DInFormationZDelta = positions3DInFormationZPositive - positions3DInFormationZNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawPositive;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw + delta,
                                                        initDeltaSize,
                                                    positions3DInFormationYawPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawNegative;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw - delta,
                                                        initDeltaSize,
                                                    positions3DInFormationYawNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawDelta(9,1);
        positions3DInFormationYawDelta = positions3DInFormationYawPositive - positions3DInFormationYawNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizePositive;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize + delta,
                                                        positions3DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizeNegative;
        formation3DTri3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                        initDeltaYaw,
                                                        initDeltaSize - delta,
                                                        positions3DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizeDelta(9,1);
        positions3DInFormationSizeDelta = positions3DInFormationSizePositive - positions3DInFormationSizeNegative;

        //ignore the last constraint which is to have unit quat
        Eigen::MatrixXd expectedJacobian (numberOfConstraints*numberOfPoints,numberOfVariables);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<3; j++)
            {
                expectedJacobian(i*3+j,0) = A(i,0)*positions3DInFormationXDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationXDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationXDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,1) = A(i,0)*positions3DInFormationYDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationYDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationYDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,2) = A(i,0)*positions3DInFormationZDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationZDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationZDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,3) = A(i,0)*positions3DInFormationYawDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationYawDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationYawDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,4) = A(i,0)*positions3DInFormationSizeDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationSizeDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationSizeDelta(j*3+2)/(2*delta);
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

    TEST (Formation3DWithOnlyYaw, JacobianConstraintOnFormation3DLine3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initZ = 0.0;
        double initQw = 1.0;
        double initQx = 0.0;
        double initQy = 0.0;
        double initQz = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaZ = 0.0;
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaZ = 0.0;
        double desiredDeltaYaw = 0.3;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,3);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        A(0,2) = 0;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        A(1,2) = 0;
        b(1) = 2.0;

        Formation3DLine3Agents formation3DLine3Agents;
        formation3DLine3Agents.SetDesiredPositionRotationAndSize(initX, initY, initZ,
                                                                 initQw, initQx, initQy, initQz,
                                                                 initSize);
        formation3DLine3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation3DBase::Ptr ptr = std::make_shared<Formation3DLine3Agents>(formation3DLine3Agents);

        ifopt::Problem nlp;

        nlp.AddVariableSet(std::make_shared<Formation3DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation3DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation3DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaZ,
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        Eigen::MatrixXd outputJacobian = Eigen::MatrixXd(nlp.GetJacobianOfConstraints());

        int numberOfConstraints = 2;
        int numberOfPoints = 3;
        int numberOfVariables = 5;
        ASSERT_EQ(numberOfConstraints*numberOfPoints, outputJacobian.rows());
        ASSERT_EQ(numberOfVariables, outputJacobian.cols());

        double delta = 0.01;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXPositive;
        formation3DLine3Agents.GetFormationPositions(initDeltaX + delta, initDeltaY, initDeltaZ,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions3DInFormationXPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXNegative;
        formation3DLine3Agents.GetFormationPositions(initDeltaX - delta, initDeltaY, initDeltaZ,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions3DInFormationXNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationXDelta(9,1);
        positions3DInFormationXDelta = positions3DInFormationXPositive - positions3DInFormationXNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYPositive;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY + delta, initDeltaZ,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions3DInFormationYPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYNegative;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY - delta, initDeltaZ,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions3DInFormationYNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYDelta(9,1);
        positions3DInFormationYDelta = positions3DInFormationYPositive - positions3DInFormationYNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZPositive;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ + delta,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions3DInFormationZPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZNegative;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ - delta,
                                                    initDeltaYaw,
                                                    initDeltaSize,
                                                    positions3DInFormationZNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationZDelta(9,1);
        positions3DInFormationZDelta = positions3DInFormationZPositive - positions3DInFormationZNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawPositive;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                    initDeltaYaw + delta,
                                                    initDeltaSize,
                                                     positions3DInFormationYawPositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawNegative;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                    initDeltaYaw - delta,
                                                    initDeltaSize,
                                                     positions3DInFormationYawNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationYawDelta(9,1);
        positions3DInFormationYawDelta = positions3DInFormationYawPositive - positions3DInFormationYawNegative;

        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizePositive;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                    initDeltaYaw,
                                                    initDeltaSize + delta,
                                                    positions3DInFormationSizePositive);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizeNegative;
        formation3DLine3Agents.GetFormationPositions(initDeltaX, initDeltaY, initDeltaZ,
                                                    initDeltaYaw,
                                                    initDeltaSize - delta,
                                                    positions3DInFormationSizeNegative);
        Eigen::Matrix<double, Eigen::Dynamic, 1> positions3DInFormationSizeDelta(9,1);
        positions3DInFormationSizeDelta = positions3DInFormationSizePositive - positions3DInFormationSizeNegative;

        //ignore the last constraint which is to have unit quat
        Eigen::MatrixXd expectedJacobian (numberOfConstraints*numberOfPoints,numberOfVariables);

        for (int i=0; i<2; i++)
        {
            for (int j=0; j<3; j++)
            {
                expectedJacobian(i*3+j,0) = A(i,0)*positions3DInFormationXDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationXDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationXDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,1) = A(i,0)*positions3DInFormationYDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationYDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationYDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,2) = A(i,0)*positions3DInFormationZDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationZDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationZDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,3) = A(i,0)*positions3DInFormationYawDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationYawDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationYawDelta(j*3+2)/(2*delta);
                expectedJacobian(i*3+j,4) = A(i,0)*positions3DInFormationSizeDelta(j*3)/(2*delta) +
                                            A(i,1)*positions3DInFormationSizeDelta(j*3+1)/(2*delta) +
                                            A(i,2)*positions3DInFormationSizeDelta(j*3+2)/(2*delta);
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

    TEST (Formation3DWithOnlyYaw, JacobianCostOnFormation3DAbreast2Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initZ = 0.0;
        double initQw = 1.0;
        double initQx = 0.0;
        double initQy = 0.0;
        double initQz = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaZ = 0.0;
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaZ = 0.0;
        double desiredDeltaYaw = 0.3;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,3);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        A(0,2) = 0;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        A(1,2) = 0;
        b(1) = 2.0;

        Formation3DAbreast2Agents formation3DAbreast2Agents;
        formation3DAbreast2Agents.SetDesiredPositionRotationAndSize(initX, initY, initZ,
                                                                    initQw, initQx, initQy, initQz,
                                                                    initSize);
        formation3DAbreast2Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation3DBase::Ptr ptr = std::make_shared<Formation3DAbreast2Agents>(formation3DAbreast2Agents);

        ifopt::Problem nlp;

        nlp.AddVariableSet(std::make_shared<Formation3DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation3DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation3DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaZ,
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        double x[5] = {initDeltaX, initDeltaY,  initDeltaZ, initDeltaYaw, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(5, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[5] = {initDeltaX+delta, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[5] = {initDeltaX-delta, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[5] = {initDeltaX, initDeltaY+delta, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[5] = {initDeltaX, initDeltaY-delta, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xZPositive[5] = {initDeltaX, initDeltaY, initDeltaZ+delta, initDeltaYaw, initDeltaSize};
        double costZPositive = nlp.EvaluateCostFunction(xZPositive);
        double xZNegative[5] = {initDeltaX, initDeltaY, initDeltaZ-delta, initDeltaYaw, initDeltaSize};
        double costZNegative = nlp.EvaluateCostFunction(xZNegative);

        double xYawPositive[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw+delta, initDeltaSize};
        double costYawPositive = nlp.EvaluateCostFunction(xYawPositive);
        double xYawNegative[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw-delta, initDeltaSize};
        double costYawNegative = nlp.EvaluateCostFunction(xYawNegative);

        double xSizePositive[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize+delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize-delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::VectorXd expectedGradient(5);
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costZPositive - costZNegative)/(2*delta);
        expectedGradient(3) = (costYawPositive - costYawNegative)/(2*delta);
        expectedGradient(4) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<5; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation3DWithOnlyYaw, JacobianCostOnFormation3DTri3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initZ = 0.0;
        double initQw = 1.0;
        double initQx = 0.0;
        double initQy = 0.0;
        double initQz = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaZ = 0.0;
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaZ = 0.0;
        double desiredDeltaYaw = 0.3;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,3);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        A(0,2) = 0;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        A(1,2) = 0;
        b(1) = 2.0;

        Formation3DTri3Agents formation3DTri3Agents;
        formation3DTri3Agents.SetDesiredPositionRotationAndSize(initX, initY, initZ,
                                                                initQw, initQx, initQy, initQz,
                                                                initSize);
        formation3DTri3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation3DBase::Ptr ptr = std::make_shared<Formation3DTri3Agents>(formation3DTri3Agents);

        ifopt::Problem nlp;

        nlp.AddVariableSet(std::make_shared<Formation3DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation3DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation3DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaZ,
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        double x[5] = {initDeltaX, initDeltaY,  initDeltaZ, initDeltaYaw, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(5, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[5] = {initDeltaX+delta, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[5] = {initDeltaX-delta, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[5] = {initDeltaX, initDeltaY+delta, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[5] = {initDeltaX, initDeltaY-delta, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xZPositive[5] = {initDeltaX, initDeltaY, initDeltaZ+delta, initDeltaYaw, initDeltaSize};
        double costZPositive = nlp.EvaluateCostFunction(xZPositive);
        double xZNegative[5] = {initDeltaX, initDeltaY, initDeltaZ-delta, initDeltaYaw, initDeltaSize};
        double costZNegative = nlp.EvaluateCostFunction(xZNegative);

        double xYawPositive[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw+delta, initDeltaSize};
        double costYawPositive = nlp.EvaluateCostFunction(xYawPositive);
        double xYawNegative[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw-delta, initDeltaSize};
        double costYawNegative = nlp.EvaluateCostFunction(xYawNegative);

        double xSizePositive[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize+delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize-delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::VectorXd expectedGradient(5);
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costZPositive - costZNegative)/(2*delta);
        expectedGradient(3) = (costYawPositive - costYawNegative)/(2*delta);
        expectedGradient(4) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<5; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation3DWithOnlyYaw, JacobianCostOnFormation3DLine3Agents)
    {
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;

        // init must be at origin for test as the jacobian is found using drone frame
        double initX = 0.0;
        double initY = 0.0;
        double initZ = 0.0;
        double initQw = 1.0;
        double initQx = 0.0;
        double initQy = 0.0;
        double initQz = 0.0;
        double initSize = 1.0;

        double initDeltaX = 0.0;
        double initDeltaY = 0.0;
        double initDeltaZ = 0.0;
        double initDeltaYaw = 0.0;
        double initDeltaSize = 0.0;

        double desiredDeltaX = 10.0;
        double desiredDeltaY = 0.0;
        double desiredDeltaZ = 0.0;
        double desiredDeltaYaw = 0.3;
        double desiredDeltaSize = 1.0;
        double weightForGoal = 1.0;
        double weightForRotation = 1.0;
        double weightForSize = 1.0;

        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(2,3);
        b.resize(2,1);

        A(0,0) = 0;
        A(0,1) = 1;
        A(0,2) = 0;
        b(0) = 2.0;
        A(1,0) = 0;
        A(1,1) = -1;
        A(1,2) = 0;
        b(1) = 2.0;

        Formation3DLine3Agents formation3DLine3Agents;
        formation3DLine3Agents.SetDesiredPositionRotationAndSize(initX, initY, initZ,
                                                                 initQw, initQx, initQy, initQz,
                                                                 initSize);
        formation3DLine3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);
        Formation3DBase::Ptr ptr = std::make_shared<Formation3DLine3Agents>(formation3DLine3Agents);

        ifopt::Problem nlp;

        nlp.AddVariableSet(std::make_shared<Formation3DVariables>(desiredDistanceInFormation,
                                                                  agentRadius));
        nlp.AddConstraintSet(std::make_shared<Formation3DStaticConstraint>(A,
                                                                           b,
                                                                           agentRadius,
                                                                           ptr));
        nlp.AddCostSet(std::make_shared<Formation3DCost>(desiredDeltaX,
                                                         desiredDeltaY,
                                                         desiredDeltaZ,
                                                         desiredDeltaYaw,
                                                         desiredDeltaSize,
                                                         weightForGoal,
                                                         weightForRotation,
                                                         weightForSize));

        double x[5] = {initDeltaX, initDeltaY,  initDeltaZ, initDeltaYaw, initDeltaSize};
        Eigen::VectorXd outputGradient = nlp.EvaluateCostFunctionGradient(x);

        ASSERT_EQ(5, outputGradient.rows());

        double delta = 0.01;

        double xXPositive[5] = {initDeltaX+delta, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costXPositive = nlp.EvaluateCostFunction(xXPositive);
        double xXNegative[5] = {initDeltaX-delta, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costXNegative = nlp.EvaluateCostFunction(xXNegative);

        double xYPositive[5] = {initDeltaX, initDeltaY+delta, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costYPositive = nlp.EvaluateCostFunction(xYPositive);
        double xYNegative[5] = {initDeltaX, initDeltaY-delta, initDeltaZ, initDeltaYaw, initDeltaSize};
        double costYNegative = nlp.EvaluateCostFunction(xYNegative);

        double xZPositive[5] = {initDeltaX, initDeltaY, initDeltaZ+delta, initDeltaYaw, initDeltaSize};
        double costZPositive = nlp.EvaluateCostFunction(xZPositive);
        double xZNegative[5] = {initDeltaX, initDeltaY, initDeltaZ-delta, initDeltaYaw, initDeltaSize};
        double costZNegative = nlp.EvaluateCostFunction(xZNegative);

        double xYawPositive[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw+delta, initDeltaSize};
        double costYawPositive = nlp.EvaluateCostFunction(xYawPositive);
        double xYawNegative[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw-delta, initDeltaSize};
        double costYawNegative = nlp.EvaluateCostFunction(xYawNegative);

        double xSizePositive[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize+delta};
        double costSizePositive = nlp.EvaluateCostFunction(xSizePositive);
        double xSizeNegative[5] = {initDeltaX, initDeltaY, initDeltaZ, initDeltaYaw, initDeltaSize-delta};
        double costSizeNegative = nlp.EvaluateCostFunction(xSizeNegative);

        Eigen::VectorXd expectedGradient(5);
        expectedGradient(0) = (costXPositive - costXNegative)/(2*delta);
        expectedGradient(1) = (costYPositive - costYNegative)/(2*delta);
        expectedGradient(2) = (costZPositive - costZNegative)/(2*delta);
        expectedGradient(3) = (costYawPositive - costYawNegative)/(2*delta);
        expectedGradient(4) = (costSizePositive - costSizeNegative)/(2*delta);

        for (int i=0; i<5; i++)
        {
            double diff = expectedGradient(i) - outputGradient(i);
            EXPECT_LE(std::abs(diff), 0.01);
        }
    }

    TEST (Formation3DWithOnlyYaw, OptimizedResultOnFormation3DTri3Agents)
    {
        int numberOfAgents = 3;
        double agentRadius = 0.5;
        double desiredDistanceInFormation = 1.5;
        double priorityPenalty = 0.0;

        double initX = 3.0;
        double initY = 3.0;
        double initZ = 1.0;
        double initQw = 1.0;
        double initQx = 0.0;
        double initQy = 0.0;
        double initQz = 0.0;
        double initSize = 1.0;

        double weightForGoal = 1000.0;
        double weightForRotation = 1.0;
        double weightForSize = 5000.0;

        Eigen::Matrix<double, Eigen::Dynamic, 3> A;
        Eigen::Matrix<double, Eigen::Dynamic, 1> b;
        A.resize(1,3);
        b.resize(1,1);

        A(0,0) = 1;
        A(0,1) = 0;
        A(0,2) = 0;
        b(0) = 4.0;
        Formation3DTri3Agents formation3DTri3Agents;
        formation3DTri3Agents.SetDesiredPositionRotationAndSize(initX,
                                                                initY,
                                                                initZ,
                                                                initQw,
                                                                initQx,
                                                                initQy,
                                                                initQz,
                                                                initSize);
        formation3DTri3Agents.SetDesiredDistanceBetweenAgents(desiredDistanceInFormation);

        Optimize3DFormation opt;
        opt.SetFormation3DTri3Agents(formation3DTri3Agents);

        std::unordered_map<uint32_t, DistributedFormation::Common::Position> optVirtualPositions;
        double optDeltaX, optDeltaY, optDeltaZ, optDeltaYaw, optDeltaSize;
        DistributedFormation::Common::Formation3DType formation3DType;
        bool optSuccessful;
        optSuccessful = opt.GetOptimizedPositions3DInFormation(numberOfAgents,
                                                               agentRadius,
                                                               weightForGoal,
                                                               weightForRotation,
                                                               weightForSize,
                                                               A,
                                                               b,
                                                               priorityPenalty,
                                                               optVirtualPositions,
                                                               optDeltaX,
                                                               optDeltaY,
                                                               optDeltaZ,
                                                               optDeltaYaw,
                                                               optDeltaSize,
                                                               formation3DType);

        EXPECT_TRUE(optSuccessful);

        double distanceFromCentroidToFront =  desiredDistanceInFormation/std::sqrt(3.0);

        double expectedOptDeltaX = (b(0) - agentRadius - distanceFromCentroidToFront) - initX;
        expectedOptDeltaX = std::min(expectedOptDeltaX, 0.0);
        double errorOptDeltaX = std::abs(optDeltaX - expectedOptDeltaX);
        double errorOptDeltaY = std::abs(optDeltaY - 0.0);
        double errorOptDeltaZ = std::abs(optDeltaZ - 0.0);
        double errorOptDeltaYaw = std::abs(optDeltaYaw - 0.0);
        double errorOptDeltaSize = std::abs(optDeltaSize - 0.0);

        EXPECT_LE(errorOptDeltaX, 0.1);
        EXPECT_LE(errorOptDeltaY, 0.1);
        EXPECT_LE(errorOptDeltaZ, 0.1);
        EXPECT_LE(errorOptDeltaYaw, 0.01);
        EXPECT_LE(errorOptDeltaSize, 0.1);
    }
} // namespace Formation3DWithOnlyYaw
