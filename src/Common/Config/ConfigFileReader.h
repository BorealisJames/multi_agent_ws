#ifndef CONFIG_FILE_READER_H
#define CONFIG_FILE_READER_H

// ROS Packages
#include <ros/ros.h>
#include <ros/datatypes.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace Common
{
    namespace Utils
    {
        class ConfigFileReader
        {
            public:
                ConfigFileReader();
                ~ConfigFileReader();

                bool getParam(const ros::NodeHandle& nh, const std::string& paramName, float& param, const float& paramDefault, const bool& searchByNamespace=true);
                bool getParam(const ros::NodeHandle& nh, const std::string& paramName, double& param, const double& paramDefault, const bool& searchByNamespace=true);
                bool getParam(const ros::NodeHandle& nh, const std::string& paramName, int& param, const int& paramDefault, const bool& searchByNamespace=true);
                bool getParam(const ros::NodeHandle& nh, const std::string& paramName, bool& param, const bool& paramDefault, const bool& searchByNamespace=true);
                bool getParam(const ros::NodeHandle& nh, const std::string& paramName, std::string& param, const std::string& paramDefault, const bool& searchByNamespace=true);
                bool getParam(const ros::NodeHandle& nh, const std::string& paramName, uint32_t& param, const uint32_t& paramDefault, const bool& searchByNamespace=true);
                bool getParam(const ros::NodeHandle& nh, const std::string& paramName, std::vector<std::vector<double>>& param, const std::vector<std::vector<double>>& paramDefault, const bool& searchByNamespace=true);
                bool getTopicRemap(const ros::NodeHandle& nh, ros::M_string& topicRemap, const bool& searchByNamespace=true);

                double getXmlDoubleValue(XmlRpc::XmlRpcValue val);

            private:

        };
    }
}

#endif // CONFIG_FILE_READER_H