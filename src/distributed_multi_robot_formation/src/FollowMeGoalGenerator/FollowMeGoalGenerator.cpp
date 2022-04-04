//
// Created by benson on 2/11/21.
//

#include "FollowMeGoalGenerator.h"

namespace DistributedFormation
{

    FollowMeGoalGenerator::FollowMeGoalGenerator()
    : m_poseReachedRadius(0.0)
    , m_minDistFromLatestPose(0.0)
    , m_historyOfHumanPoses()
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
    FollowMeGoalGenerator::GetGoalFromHumanPosesAndAvgOfExtremaPose(const std::vector<Common::Pose>& historyOfHumanPoses, const Common::Pose& centroidPose,
                                                                    Common::Pose& subGoal)
    {
        m_historyOfHumanPoses.clear();

        for (auto&&historyOfHumanPose : historyOfHumanPoses)
        {
            if (historyOfHumanPose.position.z>=1.0)
            {
                m_historyOfHumanPoses.push_back(historyOfHumanPose);
            }
        }

        UpdateWithCentroidPose(centroidPose);

        subGoal = m_goal;
        return m_goalExist;
    }

    void
    FollowMeGoalGenerator::UpdateWithCentroidPose(const Common::Pose& centroidPose)
    {
        m_goalExist = false;
        m_goal = centroidPose;

        if (m_historyOfHumanPoses.size() >= 2)
        {
            double distToFront = 0.0;
            int indexToMaintainMinDistFromLatestPose = m_historyOfHumanPoses.size() - 1;
            for (int i = m_historyOfHumanPoses.size() - 1; i >= 1; i--)
            {
                double dist = std::sqrt (std::pow(m_historyOfHumanPoses.at(i).position.x - m_historyOfHumanPoses.at(i - 1).position.x, 2) +
                                         std::pow(m_historyOfHumanPoses.at(i).position.y - m_historyOfHumanPoses.at(i - 1).position.y, 2) +
                                         std::pow(m_historyOfHumanPoses.at(i).position.z - m_historyOfHumanPoses.at(i - 1).position.z, 2));

                distToFront += dist;

                if (distToFront < m_minDistFromLatestPose)
                {
                    indexToMaintainMinDistFromLatestPose = i-2;
                }
                else
                {
                    break;
                }
            }

            if (indexToMaintainMinDistFromLatestPose>=0)
            {
                int numberOfElementsToErase = -1;
                for (int i = 0; i <= indexToMaintainMinDistFromLatestPose-1; i++)
                {
                    double dist = std::sqrt (std::pow(m_historyOfHumanPoses.at(i).position.x - centroidPose.position.x, 2) +
                                             std::pow(m_historyOfHumanPoses.at(i).position.y - centroidPose.position.y, 2) +
                                             std::pow(m_historyOfHumanPoses.at(i).position.z - centroidPose.position.z, 2));

                    bool centroidWentPastSegment = false;
                    if (i>0)
                    {
                        centroidWentPastSegment = PointHasWentPastSegment(centroidPose.position,
                                                                          m_historyOfHumanPoses.at(i-1).position,
                                                                          m_historyOfHumanPoses.at(i).position);
                    }

                    if (dist <= m_poseReachedRadius || centroidWentPastSegment)
                    {
                        numberOfElementsToErase = i+1;
                    }
                }

                if (numberOfElementsToErase >= 0 && m_historyOfHumanPoses.size())
                {
                    m_historyOfHumanPoses.erase (m_historyOfHumanPoses.begin(), m_historyOfHumanPoses.begin() + numberOfElementsToErase);
                }

                if (!m_historyOfHumanPoses.empty())
                {
                    m_goalExist = true;
                    m_goal = m_historyOfHumanPoses.at(0);
                }
            }
        }

        return;
    }

    bool
    FollowMeGoalGenerator::PointHasWentPastSegment(const Common::Position& point,
                                                   const Common::Position& segmentStart, const Common::Position& segmentEnd)
    {
        Common::Position segmentVector;
        segmentVector.x = segmentEnd.x - segmentStart.x;
        segmentVector.y = segmentEnd.y - segmentStart.y;
        segmentVector.z = segmentEnd.z - segmentStart.z;

        Common::Position startToEndVector;
        startToEndVector.x = point.x - segmentEnd.x;
        startToEndVector.y = point.y - segmentEnd.y;
        startToEndVector.z = point.z - segmentEnd.z;

        //dot
        double dotValue = startToEndVector.x * segmentVector.x + startToEndVector.y * segmentVector.y + startToEndVector.z * segmentVector.z;

        if (dotValue>=0.0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

}  // namespace DistributedFormation