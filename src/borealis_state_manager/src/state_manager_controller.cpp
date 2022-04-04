#include "../include/state_manager/state_manager.h"

namespace SM
{

void StateManager::cmdloopCallback(const ros::TimerEvent& event)
{
    if(mConfigParameters.simMode)
    {
        mArmCommand.request.value = true;
        mOffboardSetMode.request.custom_mode = "OFFBOARD";
        mAutoLandSetMode.request.custom_mode = "AUTO.LAND";
        mMaxSpeedSet.request.param_id = "MPC_XY_VEL_MAX";
        mMaxSpeedSet.request.value.integer = 0;
        mMaxSpeedSet.request.value.real = mConfigParameters.maxSpeed;

        if (mCurrentState.connected)
        {
            if(ros::Time::now() - mControlStateCallbackTime > ros::Duration(5.0))
                mControlState = Common::Entity::ControlState::NOT_IN_CONTROL;
            
            switch(mControlState)
            {
                case (Common::Entity::ControlState::NOT_IN_CONTROL):
                {
                    if(mCurrentState.mode != "AUTO.LAND" && (ros::Time::now() - mSetupRequestTime > ros::Duration(5.0)))
                    {
                        if(mSetModeClient.call(mAutoLandSetMode) && mAutoLandSetMode.response.mode_sent)
                            mConsoleLog.INFO("Auto Land");
                        else
                            mConsoleLog.INFO("Error calling auto land service");
                        
                        mSetupRequestTime = ros::Time::now();
                    }
                    break;
                }
                case (Common::Entity::ControlState::IN_CONTROL):
                {
                    if(mCurrentState.mode != "OFFBOARD" && (ros::Time::now() - mSetupRequestTime > ros::Duration(5.0)))
                    {
                        if(mSetModeClient.call(mOffboardSetMode) && mOffboardSetMode.response.mode_sent)
                            mConsoleLog.INFO("Offboard enabled");
                        else
                            mConsoleLog.INFO("Error calling offboard service");

                        mSetupRequestTime = ros::Time::now();
                    }
                    else
                    {
                        if(!mCurrentState.armed && (ros::Time::now() - mSetupRequestTime > ros::Duration(1.0)))
                        {
                            if(mArmingClient.call(mArmCommand) && mArmCommand.response.success)
                                mConsoleLog.INFO("Multicoptor armed");
                            else
                                mConsoleLog.INFO("Error calling arming service");

                            mSetupRequestTime = ros::Time::now();

                            if(mSetMPCMaxVelClient.call(mMaxSpeedSet) && mMaxSpeedSet.response.success)
                            {
                                mConsoleLog.INFO("Max speed set");
                            }
                            else
                            {
                                mConsoleLog.INFO("Error setting max speed");
                            }
                        }


                    }
                    break;
                }
            }
        }
        else
        {
            // Wait for mavros to connect and gps to start
        }
    }
}

}   // SM