//
// Created by benson on 2/11/21.
//

#include "FollowMeGoalGenerator.h"

namespace DistributedFormation
{

    FollowMeGoalGenerator::FollowMeGoalGenerator()
    : m_poseReachedRadius(0.0)
    , m_minDistFromLatestPose(0.0)
    , m_posesToTrack()
    , m_goal()
    , m_goalExist(false)
    {

    }

    void
    FollowMeGoalGenerator::SetParams(double poseReachedRadius, double minDistFromLatestPose)
    {
        m_poseReachedRadius = poseReachedRadius;
        m_minDistFromLatestPose = minDistFromLatestPose;
    }

    bool
    FollowMeGoalGenerator::GetGoalFromPosesToTrackAndAvgOfExtremaPose(const std::vector<Common::Pose>& historyOfHumanPoses, const Common::Pose& centroidPose,
                                                                      Common::Pose& subGoal)
    {
        std::vector<Common::Pose> posesToTrackBeforeInterpolation;
        for (auto&&historyOfHumanPose : historyOfHumanPoses)
        {
            if (historyOfHumanPose.position.z>=1.0)
            {
                posesToTrackBeforeInterpolation.push_back(historyOfHumanPose);
            }
        }

        std::vector<Common::Pose> posesToTrackAfterInterpolation;
        InterpolatePosesToTrack(posesToTrackBeforeInterpolation, posesToTrackAfterInterpolation);
        m_posesToTrack.clear();
        m_posesToTrack = posesToTrackAfterInterpolation;

        UpdateWithCentroidPose(centroidPose);

        subGoal = m_goal;
        return m_goalExist;
    }

    void
    FollowMeGoalGenerator::UpdateWithCentroidPose(const Common::Pose& centroidPose)
    {
        m_goalExist = false;
        m_goal = centroidPose;

        if (m_posesToTrack.size() >= 2)
        {
            int indexStartErase = m_posesToTrack.size();
            Common::Position latestPosition = m_posesToTrack.back().position;
            for (int i = 0; i < m_posesToTrack.size(); i++)
            {
                double dist = std::sqrt (std::pow(m_posesToTrack.at(i).position.x - latestPosition.x, 2) +
                                         std::pow(m_posesToTrack.at(i).position.y - latestPosition.y, 2) +
                                         std::pow(m_posesToTrack.at(i).position.z - latestPosition.z, 2));

                if (dist < m_minDistFromLatestPose)
                {
                    indexStartErase = i;
                    break;
                }
            }
            m_posesToTrack.erase (m_posesToTrack.begin() + indexStartErase,
                                  m_posesToTrack.begin() + m_posesToTrack.size());

            // unable to proceed cause there are no poses to track
            if (m_posesToTrack.empty())
            {
                return;
            }

            //find projected position of centroid on path
            Common::Pose nearestProjectedPose = m_posesToTrack.at(0);
            double smallestDistanceSqr = DBL_MAX;
            int startIndex = 0;
            for (int i = 0; (i+1) < m_posesToTrack.size(); i++)
            {
                Eigen::Vector3d startToEndVector (m_posesToTrack.at(i + 1).position.x - m_posesToTrack.at(i).position.x,
                                                  m_posesToTrack.at(i + 1).position.y - m_posesToTrack.at(i).position.y,
                                                  m_posesToTrack.at(i + 1).position.z - m_posesToTrack.at(i).position.z);

                Eigen::Vector3d startToPointVector (centroidPose.position.x - m_posesToTrack.at(i).position.x,
                                                    centroidPose.position.y - m_posesToTrack.at(i).position.y,
                                                    centroidPose.position.z - m_posesToTrack.at(i).position.z);

                double valueOfPointAlongSegment = startToPointVector.dot(startToEndVector) / startToEndVector.squaredNorm();

                Common::Pose projectedPose;
                if (valueOfPointAlongSegment > 1.0)
                {
                    projectedPose = m_posesToTrack.at(i + 1);
                }
                else if (valueOfPointAlongSegment < 0.0)
                {
                    projectedPose = m_posesToTrack.at(i);
                }
                else
                {
                    Eigen::Vector3d projectedPosition = Eigen::Vector3d(m_posesToTrack.at(i).position.x,
                                                                        m_posesToTrack.at(i).position.y,
                                                                        m_posesToTrack.at(i).position.z) + valueOfPointAlongSegment*startToEndVector.normalized();
                    projectedPose.position.x = projectedPosition.x();
                    projectedPose.position.y = projectedPosition.y();
                    projectedPose.position.z = projectedPosition.z();
                    projectedPose.headingRad = m_posesToTrack.at(i + 1).headingRad;
                }

                double distSqr = std::pow(centroidPose.position.x - projectedPose.position.x, 2) +
                                 std::pow(centroidPose.position.y - projectedPose.position.y, 2) +
                                 std::pow(centroidPose.position.z - projectedPose.position.z, 2);

                if (distSqr<=smallestDistanceSqr)
                {
                    smallestDistanceSqr = distSqr;
                    nearestProjectedPose = projectedPose;
                    startIndex = i;
                }
            }

            int numberOfElementsToErase = -1;
            for (int i = startIndex; (i+1) < m_posesToTrack.size(); i++)
            {
                double dist = std::sqrt (std::pow(m_posesToTrack.at(i).position.x - nearestProjectedPose.position.x, 2) +
                                         std::pow(m_posesToTrack.at(i).position.y - nearestProjectedPose.position.y, 2) +
                                         std::pow(m_posesToTrack.at(i).position.z - nearestProjectedPose.position.z, 2));

                if (dist <= m_poseReachedRadius)
                {
                    numberOfElementsToErase = i+1;
                }
                else if (numberOfElementsToErase >= 0)
                {
                    break;
                }
            }
            if (numberOfElementsToErase >= 0 && !m_posesToTrack.empty())
            {
                m_posesToTrack.erase (m_posesToTrack.begin(), m_posesToTrack.begin() + numberOfElementsToErase);
            }

            if (!m_posesToTrack.empty())
            {
                m_goalExist = true;
                m_goal = m_posesToTrack.at(0);
            }
        }

        return;
    }

    void
    FollowMeGoalGenerator::InterpolatePosesToTrack(const std::vector<Common::Pose>& posesToTrackBefore, std::vector<Common::Pose>& posesToTrackAfter)
    {
        if (posesToTrackBefore.empty())
        {
            posesToTrackAfter.clear();
            return;
        }

        for (int i = 0; (i+1) < posesToTrackBefore.size(); i++)
        {
            posesToTrackAfter.push_back(posesToTrackBefore.at(i));

            Eigen::Vector3d p1 (posesToTrackBefore.at(i).position.x,
                                posesToTrackBefore.at(i).position.y,
                                posesToTrackBefore.at(i).position.z);
            Eigen::Vector3d p2 (posesToTrackBefore.at(i+1).position.x,
                                posesToTrackBefore.at(i+1).position.y,
                                posesToTrackBefore.at(i+1).position.z);

            Eigen::Vector3d vecP1ToP2 (p2 - p1);
            double distanceBetweenP1andP2 = vecP1ToP2.norm();
            double headingRad = std::atan2(vecP1ToP2.y(), vecP1ToP2.x());

            double distanceToIncrease = m_poseReachedRadius;
            vecP1ToP2.normalize();
            while (distanceToIncrease < distanceBetweenP1andP2)
            {
                Eigen::Vector3d interpolatePoint = p1 + distanceToIncrease*vecP1ToP2;
                Common::Pose pose;
                pose.position.x = interpolatePoint.x();
                pose.position.y = interpolatePoint.y();
                pose.position.z = interpolatePoint.z();
                pose.headingRad = headingRad;

                posesToTrackAfter.push_back(pose);

                distanceToIncrease += m_poseReachedRadius;
            }
        }

        posesToTrackAfter.push_back(posesToTrackBefore.back());
    }

}  // namespace DistributedFormation