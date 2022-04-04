#include "../include/state_manager/state_manager.h"

namespace SM
{

StateManager::StateManager(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate):
    mNh(nh),
    mNhPrivate(nhPrivate),
    mConsoleLog(mNh,"state_manager"),
    mControlState(Common::Entity::ControlState::NOT_IN_CONTROL),
    mSetupRequestTime(ros::Time::now()),
    mConfigParameters()

{
    // Configuration Paramters
    mConfigFileReader.getParam(*mNhPrivate,"sim_mode",mConfigParameters.simMode,true,false);
    mConfigFileReader.getParam(*mNhPrivate,"cmdLoopPeriod",mConfigParameters.cmdLoopPeriod,0.05,false);
    mConfigFileReader.getParam(*mNhPrivate,"max_speed",mConfigParameters.maxSpeed,0.5,false);

    mMavrosStateSub = mNh->subscribe<mavros_msgs::State>("state", 10, &StateManager::mavrosStateCallback, this);
    mControlStateSub = mNh->subscribe<std_msgs::Int8>("control_state", 10, &StateManager::controlStateCallback, this);
    mBatteryStateSub = mNh->subscribe<sensor_msgs::BatteryState>("battery", 10, &StateManager::batteryStateCallback, this);

    mArmingClient = mNh->serviceClient<mavros_msgs::CommandBool>("armingclient");
    mSetModeClient = mNh->serviceClient<mavros_msgs::SetMode>("serviceclient");
    mSetMPCMaxVelClient = mNh->serviceClient<mavros_msgs::ParamSet>("velocityclient");

    mCmdloopTimer = mNh->createTimer(ros::Duration(mConfigParameters.cmdLoopPeriod), &StateManager::cmdloopCallback, this);
}

StateManager::~StateManager()
{
    // Destructor
}

}   // SM