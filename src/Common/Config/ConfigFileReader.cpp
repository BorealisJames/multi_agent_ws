#include "ConfigFileReader.h"

namespace Common
{
    namespace Utils
    {
        ConfigFileReader::ConfigFileReader()
        {
        }

        ConfigFileReader::~ConfigFileReader()
        {
        }

        bool ConfigFileReader::getParam(const ros::NodeHandle& nh, const std::string& paramName, float& param, const float& paramDefault, const bool& searchByNamespace)
        {
            std::string paramNameStr = paramName;
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }
            
            const bool isSuccessful = nh.param<float>(paramNameStr, param, paramDefault);
            
            std::string reportStr = paramNameStr + " : " + std::to_string(param);
            if (!isSuccessful)
            {
                ROS_INFO("%s not found!",paramNameStr.c_str());
                reportStr += " [DEFAULT]";
            }
            
            ROS_INFO("%s",reportStr.c_str());

            return isSuccessful;
        }

        bool ConfigFileReader::getParam(const ros::NodeHandle& nh, const std::string& paramName, double& param, const double& paramDefault, const bool& searchByNamespace)
        {
            std::string paramNameStr = paramName;
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }
            
            const bool isSuccessful = nh.param<double>(paramNameStr, param, paramDefault);
            
            std::string reportStr = paramNameStr + " : " + std::to_string(param);
            if (!isSuccessful)
            {
                ROS_INFO("%s not found!",paramNameStr.c_str());
                reportStr += " [DEFAULT]";
            }
            
            ROS_INFO("%s",reportStr.c_str());

            return isSuccessful;
        }

        bool ConfigFileReader::getParam(const ros::NodeHandle& nh, const std::string& paramName, int& param, const int& paramDefault, const bool& searchByNamespace)
        {
            std::string paramNameStr = paramName;
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }
            
            const bool isSuccessful = nh.param<int>(paramNameStr, param, paramDefault);
            
            std::string reportStr = paramNameStr + " : " + std::to_string(param);
            if (!isSuccessful)
            {
                ROS_INFO("%s not found!",paramNameStr.c_str());
                reportStr += " [DEFAULT]";
            }
            
            ROS_INFO("%s",reportStr.c_str());

            return isSuccessful;
        }

        bool ConfigFileReader::getParam(const ros::NodeHandle& nh, const std::string& paramName, bool& param, const bool& paramDefault, const bool& searchByNamespace)
        {
            std::string paramNameStr = paramName;
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }
            
            const bool isSuccessful = nh.param<bool>(paramNameStr, param, paramDefault);
            
            std::string reportStr = paramNameStr + " : " + std::to_string(param);
            if (!isSuccessful)
            {
                ROS_INFO("%s not found!",paramNameStr.c_str());
                reportStr += " [DEFAULT]";
            }
            
            ROS_INFO("%s",reportStr.c_str());

            return isSuccessful;
        }

        bool ConfigFileReader::getParam(const ros::NodeHandle& nh, const std::string& paramName, std::string& param, const std::string& paramDefault, const bool& searchByNamespace)
        {
            std::string paramNameStr = paramName;
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }
            
            const bool isSuccessful = nh.param<std::string>(paramNameStr, param, paramDefault);
            
            std::string reportStr = paramNameStr + " : " + param;
            if (!isSuccessful)
            {
                ROS_INFO("%s not found!",paramNameStr.c_str());
                reportStr += " [DEFAULT]";
            }
            
            ROS_INFO("%s",reportStr.c_str());

            return isSuccessful;
        }

        bool ConfigFileReader::getParam(const ros::NodeHandle& nh, const std::string& paramName, uint32_t& param, const uint32_t& paramDefault, const bool& searchByNamespace)
        {
            std::string paramNameStr = paramName;
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }
            int tParam;
            const bool isSuccessful = nh.param<int>(paramNameStr, tParam, paramDefault);
            param = static_cast<uint32_t>(tParam);
            
            std::string reportStr = paramNameStr + " : " + std::to_string(param);
            if (!isSuccessful)
            {
                ROS_INFO("%s not found!",paramNameStr.c_str());
                reportStr += " [DEFAULT]";
            }
            
            ROS_INFO("%s",reportStr.c_str());

            return isSuccessful;
        }

        bool ConfigFileReader::getParam(const ros::NodeHandle& nh, const std::string& paramName, std::vector<std::vector<double>>& param, const std::vector<std::vector<double>>& paramDefault, const bool& searchByNamespace)
        {
            std::string paramNameStr = paramName;
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }
            XmlRpc::XmlRpcValue tParam;
            const bool isSuccessful = nh.getParam(paramNameStr, tParam);
            if (tParam.getType() == XmlRpc::XmlRpcValue::Type::TypeArray)
            {
                for (int i = 0; i < tParam.size(); i ++)
                {
                    std::vector<double> tmp;
                    XmlRpc::XmlRpcValue tValue = tParam[i];
                    for (int j = 0; j < tValue.size(); j ++)
                    {
                        tmp.emplace_back(getXmlDoubleValue(tValue[j]));
                    }
                    param.emplace_back(tmp);
                }
            }
            
            std::string reportStr = paramNameStr + " : ";
            for (auto indivParamList : param)
            {
                for (auto indivParam : indivParamList)
                {
                    reportStr = reportStr + std::to_string(indivParam);
                }
            }

            if (!isSuccessful)
            {
                ROS_INFO("%s not found!",paramNameStr.c_str());
                reportStr += " [DEFAULT]";
            }
            
            ROS_INFO("%s",reportStr.c_str());

            return isSuccessful;
        }

        bool ConfigFileReader::getTopicRemap(const ros::NodeHandle& nh, ros::M_string& topicRemap, const bool& searchByNamespace)
        {
            std::string paramNameStr = "topic_remap";
            if (searchByNamespace)
            {
                paramNameStr = nh.resolveName(paramNameStr);
            }

            std::string topicRemapStr;
            bool isSuccessful = nh.param<std::string>(paramNameStr, topicRemapStr,"");
            std::stringstream ss(topicRemapStr);
            std::vector<std::string> remap;
            
            while (isSuccessful && ss.good())
            {
                getline( ss, topicRemapStr, '\n' );
                std::stringstream subss(topicRemapStr);
                remap.clear();
                while(subss.good())
                {
                    getline( ss, topicRemapStr, ',' );
                    remap.push_back(topicRemapStr);
                }
                if (remap.size() == 2)
                {
                    topicRemap[remap[0]] = remap[1];
                }
                else
                {
                    isSuccessful = false;
                }
            }

            if (!isSuccessful)
            {
                ROS_ERROR("getTopicRemap Failed. Ensure Param ""topic_remap"" is specified in ros launch file");
            }

            return isSuccessful;
        }

        double ConfigFileReader::getXmlDoubleValue(XmlRpc::XmlRpcValue val)
        {
            double value;
            switch (val.getType())
            {
                case XmlRpc::XmlRpcValue::Type::TypeInt:
                {
                    value = double(int(val));
                    break;
                }
                case XmlRpc::XmlRpcValue::Type::TypeDouble:
                {
                    value = double(val);
                    break;
                }
                default:
                // do nothing.
                value = 0;
                break;
            }
            return value;
        }
    
    }   // Utils
}   // Common