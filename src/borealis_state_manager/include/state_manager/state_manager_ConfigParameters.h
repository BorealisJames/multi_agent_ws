#ifndef STATE_MANAGER_CONFIG_PARAMETERS_H
#define STATE_MANAGER_CONFIG_PARAMETERS_H

namespace SM
{
    struct ConfigParameters
    {
        bool simMode;           // true if running in SIM, false otherwise
        double cmdLoopPeriod;
        double maxSpeed;
    };

}   // SM

#endif // STATE_MANAGER_CONFIG_PARAMETERS_H
