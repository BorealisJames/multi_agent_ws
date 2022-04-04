#include "../include/state_manager/state_manager.h"

namespace SM
{
// ROS Publisher Functions

// ROS Subscriber Functions
void StateManager::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    mCurrentState = *msg;
}

void StateManager::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    mBatteryState = *msg;
}

void StateManager::controlStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
    switch (msg->data)
    {
        case (int(Common::Entity::ControlState::NOT_IN_CONTROL)):
        mControlState = Common::Entity::ControlState::NOT_IN_CONTROL;
        break;
        case (int(Common::Entity::ControlState::IN_CONTROL)):
        mControlState = Common::Entity::ControlState::IN_CONTROL;
        break;
    }
    mControlStateCallbackTime = ros::Time::now();
}

}   // SM