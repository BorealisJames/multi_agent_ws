//
// Created by benson on 2/11/21.
//

#pragma once

#include <vector>
#include <cfloat>

#include "../Common/Common.h"

namespace DistributedFormation
{
class FollowMeGoalGenerator
{
public:
    FollowMeGoalGenerator();

    void SetParams(double poseReachedRadius, double minDistFromLatestPose);

    bool GetGoalFromHumanPosesAndAvgOfExtremaPose(const std::vector<Common::Pose>& historyOfHumanPoses, const Common::Pose& centroidPose,
                                                  Common::Pose& subGoal);

private:

    void UpdateWithCentroidPose(const Common::Pose& centroidPose);

    bool PointHasWentPastSegment(const Common::Position& point,
                                 const Common::Position& segmentStart, const Common::Position& segmentEnd);

    double m_poseReachedRadius;
    double m_minDistFromLatestPose;

    Common::Pose m_goal;
    bool m_goalExist;

    std::vector<Common::Pose> m_historyOfHumanPoses;

};

}  // namespace DistributedFormation
