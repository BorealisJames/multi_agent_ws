//
// Created by benson on 27/1/21.
//

#include "VirtualPositionAssignment.h"

namespace DistributedFormation
{

    void VirtualPositionAssignment::AssignPositionUsingMunkres(const std::unordered_map<uint32_t, Common::Position>& optVirtualPositions,
                                                               const std::unordered_map<int32_t, Common::Pose>& poseOfAgentsInTeam,
                                                               std::unordered_map<int32_t, uint32_t>& assignedTaskMap)
    {
        assignedTaskMap.clear();

        std::vector<int32_t> participatingAgentsVec;
        for (auto&& agent : poseOfAgentsInTeam)
        {
            participatingAgentsVec.push_back(agent.first);
        }
        std::vector<uint32_t> participatingTasksVec;
        for (auto&& task : optVirtualPositions)
        {
            participatingTasksVec.push_back(task.first);
        }

        int nrows = participatingAgentsVec.size();
        int ncols = participatingTasksVec.size();
        Matrix<double> matrix(nrows, ncols);

        //populating cost matrix
        for (int row = 0; row < nrows; row++)
        {
            int agentOfInterest = participatingAgentsVec.at(row);
            auto agentPositionItr = poseOfAgentsInTeam.find(agentOfInterest);

            for (int col = 0; col < ncols; col++)
            {
                double cost = std::numeric_limits<double>::infinity();

                uint32_t taskOfInterest = participatingTasksVec.at(col);
                auto optVirtualPositionsItr = optVirtualPositions.find(taskOfInterest);

                if (agentPositionItr != poseOfAgentsInTeam.end() && optVirtualPositionsItr != optVirtualPositions.end())
                {
                    double dist = std::sqrt(std::pow(agentPositionItr->second.position.x-optVirtualPositionsItr->second.x,2) +
                                            std::pow(agentPositionItr->second.position.y-optVirtualPositionsItr->second.y,2) +
                                            std::pow(agentPositionItr->second.position.z-optVirtualPositionsItr->second.z,2));

                    cost = dist * dist;
                }

                matrix(row, col) = cost;
            }
        }
        Munkres<double> m;
        m.solve(matrix);

        for (int row = 0; row < nrows; row++)
        {
            for (int col = 0; col < ncols; col++)
            {
                if (matrix(row, col) == 0)
                {
                    int32_t agentOfInterest = participatingAgentsVec.at(row);
                    uint32_t taskOfInterest = participatingTasksVec.at(col);

                    assignedTaskMap[agentOfInterest] = taskOfInterest;
                }
            }
        }
    }

    void VirtualPositionAssignment::AssignPositionUsingGreedy(const std::unordered_map<uint32_t, Common::Position>& optVirtualPositions,
                                                               const std::unordered_map<int32_t, Common::Pose>& poseOfAgentsInTeam,
                                                               std::unordered_map<int32_t, uint32_t>& assignedTaskMap)
    {
        assignedTaskMap.clear();

        std::vector<int32_t> participatingAgentsVec;
        for (auto&& agent : poseOfAgentsInTeam)
        {
            participatingAgentsVec.push_back(agent.first);
        }
        sort(participatingAgentsVec.begin(), participatingAgentsVec.end());

        std::vector<uint32_t> participatingTasksVec;
        for (auto&& task : optVirtualPositions)
        {
            participatingTasksVec.push_back(task.first);
        }
        sort(participatingTasksVec.begin(), participatingTasksVec.end());

        for (int i = 0; i < participatingTasksVec.size(); i++)
        {
            uint32_t taskOfInterest = participatingTasksVec.at(i);
            auto optVirtualPositionsItr = optVirtualPositions.find(taskOfInterest);

            double shortestDistance = DBL_MAX;
            int agentIndexAssignedToTask = -1;

            for (int j = 0; j < participatingAgentsVec.size(); j++)
            {
                int agentOfInterest = participatingAgentsVec.at(j);
                auto agentPositionItr = poseOfAgentsInTeam.find(agentOfInterest);

                if (agentPositionItr != poseOfAgentsInTeam.end() && optVirtualPositionsItr != optVirtualPositions.end())
                {
                    double distance = std::sqrt(std::pow(agentPositionItr->second.position.x-optVirtualPositionsItr->second.x,2) +
                                                std::pow(agentPositionItr->second.position.y-optVirtualPositionsItr->second.y,2) +
                                                std::pow(agentPositionItr->second.position.z-optVirtualPositionsItr->second.z,2));

                    if (distance<=shortestDistance)
                    {
                        shortestDistance = distance;
                        agentIndexAssignedToTask = j;
                    }
                }
            }

            if (agentIndexAssignedToTask != -1)
            {
                assignedTaskMap[participatingAgentsVec.at(agentIndexAssignedToTask)] = taskOfInterest;
                participatingAgentsVec.erase (participatingAgentsVec.begin()+agentIndexAssignedToTask);
            }
        }
    }

}   // namespace DistributedFormation