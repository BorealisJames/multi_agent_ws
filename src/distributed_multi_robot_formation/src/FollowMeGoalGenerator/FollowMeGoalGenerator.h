//
// Created by benson on 2/11/21.
//

#pragma once

#include <cfloat>
#include <iostream>
#include <vector>

#include "../Common/Common.h"

namespace DistributedFormation
{
class FollowMeGoalGenerator
{
public:
    FollowMeGoalGenerator();

    void SetParams(double poseReachedRadius, double minDistFromLatestPose);

    bool GetGoalFromPosesToTrackAndAvgOfExtremaPose(const std::vector<Common::Pose>& historyOfHumanPoses, const Common::Pose& centroidPose,
                                                    Common::Pose& subGoal);

private:

    void UpdateWithCentroidPose(const Common::Pose& centroidPose);

    void InterpolatePosesToTrack(const std::vector<Common::Pose>& posesToTrackBefore, std::vector<Common::Pose>& posesToTrackAfter);

    double m_poseReachedRadius;
    double m_minDistFromLatestPose;

    Common::Pose m_goal;
    bool m_goalExist;

    std::vector<Common::Pose> m_posesToTrack;

};

}  // namespace DistributedFormation
