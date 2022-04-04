#ifndef TEAMING_PLANNER_CONSTANTS_H
#define TEAMING_PLANNER_CONSTANTS_H

namespace TeamingPlannerConstants
{
    enum class ModuleState
    {
        NOT_APPLICABLE = 0,
        INITILAISING = 1,
        READY = 2,
        RUNNING = 3,
        PAUSED = 4,
        COMPLETED = 5,
        CANCELLED = 6
    };
} // Teaming Planner

#endif // TEAMING_PLANNER_CONSTANTS_H