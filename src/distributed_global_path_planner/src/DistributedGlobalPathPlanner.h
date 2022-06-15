#pragma once

#include "DistributedGlobalPathPlannerHandler.h"
#include "PhaseSynchronizer/PhaseSynchronizer.h"

// Purpose: grab required inputs and manage the state

namespace DistributedGlobalPathPlanner
{

class DistributedGlobalPathPlanner
{
public:
    DistributedGlobalPathPlanner();

    void SetParameters(const Common::DistributedGlobalPathParams& distributedGlobalPathParams,
                       const pathplanning::PathPlanningParams& pathPlanningParams);

    void AttachHandler(const std::shared_ptr<DistributedGlobalPathPlannerHandler>& handlerPtr);

    void RunDistributedGlobalPathPlanner();

private:
    PhaseSynchronizer m_ps;
};

}  // namespace DistributedGlobalPathPlanner