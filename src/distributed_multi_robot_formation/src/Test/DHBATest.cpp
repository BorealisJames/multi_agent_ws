//
// Created by benson on 28/1/21.
//

#include <gtest/gtest.h>
#include <set>

#include "../Common/Common.h"
#include "../VirtualPositionAssignment/VirtualPositionAssignment.h"
//#include "../DHBA/VirtualPositionAssignment.h"

namespace DistributedFormation
{
    TEST (DHBA, AssignPosition)
    {
        VirtualPositionAssignment virtualPositionAssignment;

        std::unordered_map<uint32_t, Common::Position> optVirtualPositions;
        Common::Position vp1;
        vp1.x = 0;
        vp1.y = 1;
        vp1.z = 5;
        optVirtualPositions[1] = vp1;
        Common::Position vp10;
        vp10.x = 2;
        vp10.y = 1;
        vp10.z = 5;
        optVirtualPositions[10] = vp10;
        Common::Position vp100;
        vp100.x = 10;
        vp100.y = 1;
        vp100.z = 5;
        optVirtualPositions[100] = vp100;

        std::unordered_map<int32_t, Common::Pose> poseOfAgentsInTeam;
        Common::Pose agent1;
        agent1.position.x = 0;
        agent1.position.y = 0;
        agent1.position.z = 5;
        poseOfAgentsInTeam[1] = agent1;
        Common::Pose agent10;
        agent10.position.x = -2;
        agent10.position.y = 0;
        agent10.position.z = 5;
        poseOfAgentsInTeam[10] = agent10;
        Common::Pose agent100;
        agent100.position.x = -10;
        agent100.position.y = 0;
        agent100.position.z = 5;
        poseOfAgentsInTeam[100] = agent100;

        std::unordered_map<int32_t, uint32_t> assignedTaskMap;
        virtualPositionAssignment.AssignPositionUsingMunkres(optVirtualPositions,
                                                             poseOfAgentsInTeam,
                                                             assignedTaskMap);

        ASSERT_TRUE(assignedTaskMap.find(1)!=assignedTaskMap.end());
        EXPECT_EQ(assignedTaskMap[1], 100);

        ASSERT_TRUE(assignedTaskMap.find(10)!=assignedTaskMap.end());
        EXPECT_EQ(assignedTaskMap[10], 10);

        ASSERT_TRUE(assignedTaskMap.find(100)!=assignedTaskMap.end());
        EXPECT_EQ(assignedTaskMap[100], 1);
    }

} // namespace DistributedFormation