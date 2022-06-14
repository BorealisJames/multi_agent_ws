//
// Created by benson on 5/11/21.
//

#include <gtest/gtest.h>
#include <set>

#include "../Common/Common.h"
#include "../FollowMeGoalGenerator/FollowMeGoalGenerator.h"

namespace DistributedFormation
{
    TEST (FollowMeGoalGenerator, GetGoalFromHumanPosesAndCentroidPose)
    {
        Common::Pose centroidPose;
        Common::Pose subGoal;
        bool goalObtained;
        double poseReachedRadius = 1.0;
        double stepSizeInOneDim = 1.0;
        double minDistFromLatestPose = 5 * std::sqrt(2*stepSizeInOneDim*stepSizeInOneDim);

        FollowMeGoalGenerator followMeGoalGenerator;
        followMeGoalGenerator.SetParams(poseReachedRadius, minDistFromLatestPose);

        double humanHeight = 2.0;
        std::vector<Common::Pose> historyOfHumanPosesLong;
        for (int i = 0; i < 15; i++)
        {
            Common::Pose pose;
            pose.position.x = i * stepSizeInOneDim;
            pose.position.y = i * stepSizeInOneDim;
            pose.position.z = humanHeight;

            historyOfHumanPosesLong.push_back(pose);
        }

        std::vector<Common::Pose> historyOfHumanPosesShort;
        for (int i = 0; i < 4; i++)
        {
            Common::Pose pose;
            pose.position.x = i * stepSizeInOneDim;
            pose.position.y = i * stepSizeInOneDim;
            pose.position.z = humanHeight;

            historyOfHumanPosesShort.push_back(pose);
        }

        centroidPose.position.x = 0;
        centroidPose.position.y = 0;
        centroidPose.position.z = 2;
        goalObtained = followMeGoalGenerator.GetGoalFromPosesToTrackAndAvgOfExtremaPose(historyOfHumanPosesShort,
                                                                                        centroidPose,
                                                                                        subGoal);
        EXPECT_FALSE(goalObtained);

        centroidPose.position.x = -2;
        centroidPose.position.y = -2;
        centroidPose.position.z = 1.5;
        goalObtained = followMeGoalGenerator.GetGoalFromPosesToTrackAndAvgOfExtremaPose(historyOfHumanPosesLong,
                                                                                        centroidPose,
                                                                                        subGoal);
        EXPECT_TRUE(goalObtained);
        EXPECT_DOUBLE_EQ(subGoal.position.x, 0.0);
        EXPECT_DOUBLE_EQ(subGoal.position.y, 0.0);
        EXPECT_DOUBLE_EQ(subGoal.position.z, 2.0);

        centroidPose.position.x = 3.5;
        centroidPose.position.y = 3.5;
        centroidPose.position.z = 2.0;
        goalObtained = followMeGoalGenerator.GetGoalFromPosesToTrackAndAvgOfExtremaPose(historyOfHumanPosesLong,
                                                                                        centroidPose,
                                                                                        subGoal);
        EXPECT_TRUE(goalObtained);
        EXPECT_DOUBLE_EQ(subGoal.position.x, 5.0);
        EXPECT_DOUBLE_EQ(subGoal.position.y, 5.0);
        EXPECT_DOUBLE_EQ(subGoal.position.z, 2.0);
    }


} // namespace DistributedFormation