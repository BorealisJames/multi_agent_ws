//
// Created by benson on 25/1/22.
//

#include "GoTherePathTracker.h"

namespace DistributedGlobalPathPlanner
{


GoTherePathTracker::GoTherePathTracker()
: k_radiusForLastWP(2.0)
, m_pathToTrack()
, m_wpIndex(0)
, m_pathCompleted(false)
{
}

bool
GoTherePathTracker::InitPathToTrack(const std::vector<Common::Pose>& pathToTrack)
{
    m_pathToTrack.clear();
    m_wpIndex = 0;
    m_pathCompleted = false;

    if (pathToTrack.size()<1)
    {
        return false;
    }

    m_pathToTrack = pathToTrack;

    return true;
}

std::vector<Common::Pose>
GoTherePathTracker::getPathToTrack() const
{
    return m_pathToTrack;
}

int
GoTherePathTracker::getWPIndex() const
{
    return m_wpIndex;
}

void
GoTherePathTracker::setWPIndex(const int& wpIndex)
{
    m_wpIndex = wpIndex;
}

bool
GoTherePathTracker::GetUpdatedPath(const Common::Pose& currentPose,
                                        std::vector<Common::Pose>& updatedPath)
{
    updatedPath.clear();

    if (m_pathToTrack.size()<1)
    {
        return false;
    }

    //update last wp reached
    double distToLastWP = (m_pathToTrack.back().position - currentPose.position).norm();

    if (distToLastWP <= k_radiusForLastWP)
    {
        m_pathCompleted = true;
    }

    //if last wp is reached
    if (m_pathCompleted)
    {
        return true;
    }

    //handle single point progress. Since there is only 1 point the updated path is always that point
    if (m_pathToTrack.size()==1)
    {
        updatedPath.push_back(m_pathToTrack.at(0));
        return true;
    }

    Eigen::Vector3d startPt;

    double value = 1.1;

    while (value >= 1.0 && !m_pathCompleted)
    {

        m_wpIndex = std::min((int)m_wpIndex, (int)m_pathToTrack.size()-2);

        Eigen::Vector3d pt1ToCurrPositionVec = currentPose.position - m_pathToTrack.at(m_wpIndex).position;
        Eigen::Vector3d pt1ToPt2Vec = m_pathToTrack.at(m_wpIndex+1).position - m_pathToTrack.at(m_wpIndex).position;

        value = pt1ToCurrPositionVec.dot(pt1ToPt2Vec) / pt1ToPt2Vec.squaredNorm();

        if (value>=1.0)
        {
            m_wpIndex++;

            // if reached end of path
            if (m_wpIndex >= m_pathToTrack.size()-1)
            {
                m_pathCompleted = true;
                return true;
            }
        }
        else
        {
            //value = std::max(value, 0.0);
            //startPt = value*pt1ToPt2Vec + m_pathToTrack.at(m_wpIndex).position;
            startPt = m_pathToTrack.at(m_wpIndex).position;
            break;
        }
    }

    Common::Pose startPose;
    startPose.position = startPt;
    startPose.headingRad = std::atan2(m_pathToTrack.at(m_wpIndex+1).position.y() - m_pathToTrack.at(m_wpIndex).position.y(),
                                      m_pathToTrack.at(m_wpIndex+1).position.x() - m_pathToTrack.at(m_wpIndex).position.x());
    updatedPath.push_back(startPose);

    for (int i = m_wpIndex+1; i < m_pathToTrack.size(); i++)
    {
        updatedPath.push_back(m_pathToTrack.at(i));
    }

    return true;
}

bool
GoTherePathTracker::GetProgressOfPointALongPathToTrack(const Eigen::Vector3d& point, double& progress)
{
    progress = 0.0;

    if (m_pathToTrack.size()<1)
    {
        return false;
    }

    if (m_pathCompleted)
    {
        progress = m_pathToTrack.size()-1;
        return true;
    }

    //handle single point progress. Nearer to goal means larger progress
    if (m_pathToTrack.size() == 1)
    {
        double distToGoal = (point-m_pathToTrack.at(0).position).norm();
        progress = 1.0 / (distToGoal + 1.0);
        return true;
    }

    progress =  m_wpIndex;
    int wpIndex = std::min(m_wpIndex, (int)m_pathToTrack.size()-2);
    double value = 1.1;
    while (value >= 1.0)
    {
        Eigen::Vector3d pt1ToCurrPositionVec = point - m_pathToTrack.at(wpIndex).position;
        Eigen::Vector3d pt1ToPt2Vec = m_pathToTrack.at(wpIndex+1).position - m_pathToTrack.at(wpIndex).position;

        value = pt1ToCurrPositionVec.dot(pt1ToPt2Vec) / pt1ToPt2Vec.squaredNorm();

        if (value>=1.0)
        {
            wpIndex++;
            progress = wpIndex;
            if (wpIndex >= m_pathToTrack.size()-1)
            {
                break;
            }
        }
        else
        {
            progress = wpIndex + std::max(value, 0.0);
            break;
        }
    }

    return true;

}

}  // namespace DistributedGlobalPathPlanner