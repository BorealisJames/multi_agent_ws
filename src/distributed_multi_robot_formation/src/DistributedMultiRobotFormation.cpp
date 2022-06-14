//
// Created by benson on 13/1/21.
//

#include "DistributedMultiRobotFormation.h"

namespace DistributedFormation
{
    DistributedMultiRobotFormation::DistributedMultiRobotFormation()
    : m_ps()
    {}

    void DistributedMultiRobotFormation::SetParameters(const Common::DistributedFormationParameters& params)
    {
        m_ps.SetDistributedFormationParameters(params);
    }

    void DistributedMultiRobotFormation::AttachHandler(const std::shared_ptr<DistributedMultiRobotFormationHandler>& handlerPtr)
    {
        m_ps.AttachHandler(handlerPtr);
    }

    void DistributedMultiRobotFormation::RunDistributedFormation()
    {
        m_ps.SyncPhasesOfAgents();
    }

}  // namespace DistributedFormation
