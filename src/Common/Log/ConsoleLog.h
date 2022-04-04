#ifndef CONSOLE_LOG_H
#define CONSOLE_LOG_H

#include <ros/ros.h>
#include <string>

namespace Common
{

namespace Log
{
    class ConsoleLog
    {
        public:
            explicit ConsoleLog(const ros::NodeHandlePtr& nh, const std::string& nodeName="");
            ConsoleLog() = delete;
            ConsoleLog(const ConsoleLog& instance) = delete;
            ConsoleLog& operator=(const ConsoleLog& instance) = delete;
            
            ~ConsoleLog();

            void INFO(const std::string& logStr);
            void WARN(const std::string& logStr);
            void ERROR(const std::string& logStr);
            void DEBUG(const std::string& logStr);

        private:
            ros::NodeHandlePtr mNh;
            std::string mNamespaceTag;
            std::string mNodeName;
    };

}   // Log

}   // Common
#endif  // CONSOLE_LOG_H