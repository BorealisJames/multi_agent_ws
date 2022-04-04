//
// Created by benson on 13/1/21.
//

#pragma once

#include "DistributedMultiRobotFormationHandler.h"
#include "PhaseSynchronizer/PhaseSynchronizer.h"

// Purpose: grab required inputs and manage the state

namespace DistributedFormation
{

class DistributedMultiRobotFormation
{
public:
    DistributedMultiRobotFormation();

    void AttachHandler(const std::shared_ptr<DistributedMultiRobotFormationHandler>& handlerPtr);

    void RunDistributedFormation();

private:
    PhaseSynchronizer m_ps;
};

}  // namespace DistributedFormation