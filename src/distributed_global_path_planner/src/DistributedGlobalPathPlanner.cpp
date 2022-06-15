//
// Created by benson on 13/1/21.
//

#include "DistributedGlobalPathPlanner.h"

namespace DistributedGlobalPathPlanner
{
    DistributedGlobalPathPlanner::DistributedGlobalPathPlanner()
    : m_ps()
    {}

    void DistributedGlobalPathPlanner::SetParameters(const Common::DistributedGlobalPathParams& distributedGlobalPathParams,
                                                     const pathplanning::PathPlanningParams& pathPlanningParams)
    {
        m_ps.SetDistributedGlobalPathPlannerParams(distributedGlobalPathParams, pathPlanningParams);
    }

    void DistributedGlobalPathPlanner::AttachHandler(const std::shared_ptr<DistributedGlobalPathPlannerHandler>& handlerPtr)
    {
        m_ps.AttachHandler(handlerPtr);
    }

    void DistributedGlobalPathPlanner::RunDistributedGlobalPathPlanner()
    {
        m_ps.SyncPhasesOfAgents();
    }

}  // namespace DistributedFormation
