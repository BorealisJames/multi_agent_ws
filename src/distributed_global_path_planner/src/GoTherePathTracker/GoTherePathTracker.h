//
// Created by benson on 25/1/22.
//

#pragma once

#include "../Common/Common.h"

#include <iostream>

namespace DistributedGlobalPathPlanner
{

class GoTherePathTracker
{
public:
    GoTherePathTracker(void);

    std::vector<Common::Pose> getPathToTrack() const;

    int getWPIndex() const;

    void setWPIndex(const int& wpIndex);

    bool InitPathToTrack(const std::vector<Common::Pose>& pathToTrack);

    bool GetUpdatedPath(const Common::Pose& currentPose,
                        std::vector<Common::Pose>& updatedPath);

    bool GetProgressOfPointALongPathToTrack(const Eigen::Vector3d& point, double& progress);


private:
    const double k_radiusForLastWP;
    std::vector<Common::Pose> m_pathToTrack;
    int m_wpIndex;
    bool m_pathCompleted;

};

}  // namespace DistributedGlobalPathPlanner
