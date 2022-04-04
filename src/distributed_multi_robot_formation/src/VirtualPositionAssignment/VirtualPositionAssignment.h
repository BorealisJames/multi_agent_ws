//
// Created by benson on 27/1/21.
//

#pragma once

#include <cfloat>
#include <unordered_map>
#include <vector>

#include "Munkres.h"
#include "../Common/Common.h"

namespace DistributedFormation
{

class VirtualPositionAssignment
{
public:

    void AssignPositionUsingMunkres(const std::unordered_map<uint32_t, Common::Position>& optVirtualPositions,
                                    const std::unordered_map<int32_t, Common::Pose>& poseOfAgentsInTeam,
                                    std::unordered_map<int32_t, uint32_t>& assignedTaskMap);

    void AssignPositionUsingGreedy(const std::unordered_map<uint32_t, Common::Position>& optVirtualPositions,
                                    const std::unordered_map<int32_t, Common::Pose>& poseOfAgentsInTeam,
                                    std::unordered_map<int32_t, uint32_t>& assignedTaskMap);
};

} // namespace DistributedFormation
