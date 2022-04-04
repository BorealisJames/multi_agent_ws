#include "ConsoleLog.h"

namespace Common
{

namespace Log
{
    ConsoleLog::ConsoleLog(const ros::NodeHandlePtr& nh, const std::string& nodeName):
        mNh(nh),
        mNamespaceTag("[" + mNh->getNamespace() + "]"),
        mNodeName("[" + nodeName + "] ")
    {
    }

    ConsoleLog::~ConsoleLog()
    {        
    }

    void ConsoleLog::INFO(const std::string& logStr)
    {
        ROS_INFO("%s",(mNamespaceTag + mNodeName + logStr).c_str());
    }

    void ConsoleLog::WARN(const std::string& logStr)
    {
        ROS_WARN("%s",(mNamespaceTag + mNodeName + logStr).c_str());
    }

    void ConsoleLog::ERROR(const std::string& logStr)
    {
        ROS_ERROR("%s",(mNamespaceTag + mNodeName + logStr).c_str());
    }

    void ConsoleLog::DEBUG(const std::string& logStr)
    {
        ROS_DEBUG("%s",(mNamespaceTag + mNodeName + logStr).c_str());
    }

}   // Log

}   // Common