#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

// ROS Packages
#include <ros/ros.h>
#include <ros/subscribe_options.h>

// Messages
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h> 
#include <mavros_msgs/ParamSet.h>
#include <sensor_msgs/BatteryState.h> 
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>

// C++ Stuff
#include <dynamic_reconfigure/server.h>

// Common 
#include "../../../Common/ConstantsEnum.h"
#include "../../../Common/Config/ConfigFileReader.h"
#include "../../../Common/Log/ConsoleLog.h"

#include "state_manager_ConfigParameters.h"

namespace SM
{
class StateManager
{
    private: 
        ros::NodeHandlePtr mNh;
        ros::NodeHandlePtr mNhPrivate;
        ros::Time mControlStateCallbackTime;
        ros::Time mSetupRequestTime;

        // Logger
        Common::Log::ConsoleLog mConsoleLog;

        // Config
        Common::Utils::ConfigFileReader mConfigFileReader;
        ConfigParameters mConfigParameters;

        // ROS Service Clients
        ros::ServiceClient mArmingClient;
        ros::ServiceClient mSetModeClient;
        ros::ServiceClient mSetMPCMaxVelClient;

        // ROS Publishers

        // ROS Subscribers
        ros::Subscriber mMavrosStateSub;
        ros::Subscriber mBatteryStateSub;
        ros::Subscriber mControlStateSub;

        // ROS Timers
        ros::Timer mCmdloopTimer;

        // Variables
        mavros_msgs::SetMode mOffboardSetMode;
        mavros_msgs::SetMode mAutoLandSetMode;
        mavros_msgs::CommandBool mArmCommand;
        mavros_msgs::ParamSet mMaxSpeedSet;
        mavros_msgs::State mCurrentState;
        sensor_msgs::BatteryState mBatteryState;
        Common::Entity::ControlState mControlState;

        // ROS Publisher Functions

        // ROS Subscriber Functions
        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
        void controlStateCallback(const std_msgs::Int8::ConstPtr& msg);

        // ROS Timer Functions
        void cmdloopCallback(const ros::TimerEvent& event);
    
    public:
        StateManager(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& nhPrivate);
        virtual ~ StateManager();
};

}   //SM
#endif // STATE_MANAGER_H
