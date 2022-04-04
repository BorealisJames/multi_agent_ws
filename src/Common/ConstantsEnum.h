#ifndef CONSTANTS_ENUM_H
#define CONSTANTS_ENUM_H

#include <ros/ros.h>

#include <type_traits>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <mt_msgs/mtTask.h>
#include <mt_msgs/mtAgent.h>
#include <tf/transform_datatypes.h>

namespace Common
{

namespace Entity
{
    static std::string AgentNamespace(const ros::NodeHandlePtr& Nh)
    {
        return static_cast<std::string>(Nh->getNamespace()).erase(0,1); // Remove leading '/'
    };

    static std::string TASK_TOPIC = "task";

    static std::string SYSTEM_FRAME = "odom";
    
    static std::string AgentLocalOdomFrame(const ros::NodeHandlePtr& Nh)
    {
        return AgentNamespace(Nh) + "/LOCAL/odom";
    };

    static std::string AgentBodyFrame(const ros::NodeHandlePtr& Nh)
    {
        return AgentNamespace(Nh) + "/body";
    };

    static std::string AgentLocalFrame(const ros::NodeHandlePtr& Nh)
    {
        return AgentNamespace(Nh) + "/LOCAL/map";
    };

    static std::string TaskGroupName(const uint64_t& taskGroupId)
    {
        return "task" + std::to_string(taskGroupId);
    }

    static std::string TaskGroupAgentTopic(const std::string& taskGroupNamespace)
    {
        return "/"+taskGroupNamespace+"/agent";
    }

    static std::string MTCommonAgentTopic = "/mt_communications/agent";

    static std::string TaskGroupCBBAReportTopic(const std::string& taskGroupNamespace)
    {
        return "/"+taskGroupNamespace+"/CBBA_report";
    }

    struct TopicRemap
    {
        std::string name;
        std::string topicName;
    };

    struct Position
    {
        double x;
        double y;
        double z;

        Position():
            x(0.0), y(0.0), z(0.0)
        {}

        Position(const double& ax, const double& ay, const double& az):
            x(ax), y(ay), z(az)
        {}

        Position operator+(const Position& other)
        {
            return Position(x+other.x,y+other.y,z+other.z);
        }

        Position& operator+=(const Position& other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        // Scalar product
        Position operator*(const double& scalar) const
        {
            return Position(scalar*x,scalar*y,scalar*z);
        }

        // Scalar product
        Position& operator*=(const double& scalar)
        {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        // Cross product
        Position operator*(const Position& other)
        {
            return Position(y*other.z - z*other.y,
                            z*other.x - x*other.z,
                            x*other.y - y*other.x);
        }

        // Cross product
        Position& operator*=(const Position& other)
        {
            const double xNew = y*other.z - z*other.y;
            const double yNew = z*other.x - x*other.z;
            const double zNew = x*other.y - y*other.x;
            x = xNew; y = yNew; z = zNew;
            return *this;
        }
    };

    struct Pose
    {
        Position position;
        double yaw;

        Pose():
            position(),yaw(0.0)
        {}

        Pose(const double& ax,const double& ay,const double& az,const double& ayaw):
            position(ax,ay,az),yaw(ayaw)
        {}

        Pose(const Position& aposition, const double& ayaw):
            position(aposition), yaw(ayaw)
        {}

        Pose(const geometry_msgs::PoseStamped& aPose):
            position(aPose.pose.position.x,aPose.pose.position.y,aPose.pose.position.z)
        {
            const tf::Quaternion q(aPose.pose.orientation.x,aPose.pose.orientation.y,aPose.pose.orientation.z,aPose.pose.orientation.w);
            const tf::Matrix3x3 R(q);
            double r,p,y;
            R.getRPY(r,p,y);
            yaw = y;
        }

        Pose(const nav_msgs::Odometry& aOdometry):
            position(aOdometry.pose.pose.position.x,aOdometry.pose.pose.position.y,aOdometry.pose.pose.position.z)
        {
            const tf::Quaternion q(aOdometry.pose.pose.orientation.x,
                                   aOdometry.pose.pose.orientation.y,
                                   aOdometry.pose.pose.orientation.z,
                                   aOdometry.pose.pose.orientation.w);
            const tf::Matrix3x3 R(q);
            double r,p,y;
            R.getRPY(r,p,y);
            yaw = y;
        }

        Pose operator+(const Pose& other)
        {
            return Pose(position+other.position,yaw+other.yaw);
        }

        Pose& operator+=(const Pose& other)
        {
            position += other.position;
            yaw += other.yaw;
            return *this;
        }

        // Scalar product
        Pose operator*(const double& scalar)
        {
            return Pose(position*scalar,scalar*yaw);
        }

        // Scalar product position vector
        Pose& operator*=(const double& scalar)
        {
            position *= scalar;
            yaw *= scalar;
            return *this;
        }
    };

    enum class SystemPoseSource : int
    {
        LOCALISATION = 0,
        PLUGIN = 1,
        MODEL_STATES = 2
    };
    using SystemPoseSourceType = std::underlying_type<SystemPoseSource>::type;

    struct Odom
    {
        geometry_msgs::PoseWithCovariance pose_c;
        geometry_msgs::TwistWithCovariance twist_c;

        Odom():
            pose_c(geometry_msgs::PoseWithCovariance()), twist_c(geometry_msgs::TwistWithCovariance())
        {}

        Odom(const geometry_msgs::PoseWithCovariance& aPose_c, const geometry_msgs::TwistWithCovariance& aTwist_c)
        {
            pose_c = aPose_c;
            twist_c = aTwist_c;
        }

        Odom(const geometry_msgs::Pose& aPose, const geometry_msgs::Twist& aTwist)
        {
            pose_c.pose = aPose;
            twist_c.twist = aTwist;
        }

        Odom(const nav_msgs::Odometry& aSystemPose):
            pose_c(aSystemPose.pose), twist_c(aSystemPose.twist)
        {}

        Odom(const gazebo_msgs::ModelState& aModelState)
        {
            pose_c.pose = aModelState.pose;
            twist_c.twist = aModelState.twist;
        }
    };

    enum class PlannerState 
    {
        NONE = 0,
        POSE = 1,
        VELOCITY = 2,
        RAW = 3,
        ATTITUDE = 4,
        TELEOP = 5
    };

    enum class ControlState 
    {
        NOT_IN_CONTROL = 0,
        IN_CONTROL = 1
    };

    enum class WaypointState
    {
        NOT_APPLICABLE = 0,
        X_POSITIVE_DIRECTION = 1,
        X_NEGATIVE_DIRECTION = -1,
        Y_POSITIVE_DIRECTION = 2,
        Y_NEGATIVE_DIRECTION = -2,
        Z_POSITIVE_DIRECTION = 3,
        Z_NEGATIVE_DIRECTION = -3,
        REACHED = 4
    };

    enum class MTTaskEnum : mt_msgs::mtTask::_type_type
    {
        UNDEFINED = 0,
        GO_THERE = 1,
        FOLLOW_ME = 2,
        DISTRACT_TARGET = 3
    };
    using MTTaskEnumType = std::underlying_type<MTTaskEnum>::type;

    struct MTTaskBundle
    {
        std_msgs::Header header;
        std::string nameSpace;
        MTTaskEnum type;
        uint64_t numAgents;
        std::vector<std::string> agents;
        std::vector<geometry_msgs::Pose> targets;

        MTTaskBundle():
            header(std_msgs::Header()), nameSpace(""), type(MTTaskEnum::UNDEFINED), numAgents(0u), agents(), targets()
        {}

        MTTaskBundle(const mt_msgs::mtTask& aTask):
            header(aTask.header),
            nameSpace(TaskGroupName(aTask.groupId)),
            type(static_cast<MTTaskEnum>(aTask.type)),
            numAgents(aTask.numAgents),
            agents(aTask.agents),
            targets(aTask.targets)
        {}

        bool compareTargets(const MTTaskBundle& other) const
        {
            bool common = true;
            std::vector<geometry_msgs::Pose> thisTargets = targets;
            std::vector<geometry_msgs::Pose> otherTargets = other.targets;

            std::vector<geometry_msgs::Pose>::iterator otherIt = otherTargets.begin();
            for (std::vector<geometry_msgs::Pose>::iterator thisIt = thisTargets.begin(); thisIt != thisTargets.end(); thisIt++)
            {
                common = false;
                for (otherIt = otherTargets.begin(); otherIt != otherTargets.end(); otherIt++)
                {
                    if (*thisIt == *otherIt)
                    {
                        common = true;
                        otherTargets.erase(otherIt);
                        break;
                    }
                }
                if (common == false){ break; }
            }
            return common;
        }

        bool operator==(const MTTaskBundle& other) const
        {
            bool equal = true;
            equal = equal && (nameSpace == other.nameSpace);
            equal = equal && (type == other.type);
            equal = equal && (numAgents == other.numAgents);
            std::vector<std::string> thisAgents = agents;
            std::vector<std::string> otherAgents = other.agents;
            std::sort(thisAgents.begin(),thisAgents.end());
            std::sort(otherAgents.begin(),otherAgents.end());
            equal = equal && (thisAgents == otherAgents);
            /*
            if (equal)
            {
                for (const auto& target : targets)
                {
                    equal = false;
                    for (const auto& otherTarget : other.targets)
                    {
                        if (target == otherTarget)
                        {
                            equal = true;
                            break;
                        }
                    }
                    if (equal == false){ break; }
                }
            }
            */
            return equal;
        }

        bool operator!=(const MTTaskBundle& other) const
        {
            return !operator==(other);
        }
    };

    struct MTAgent
    {
        std::string name;
        MTTaskEnum task;
        geometry_msgs::PoseStamped localPoseStamped;
        Common::Entity::Odom systemPose;
        bool taskAllocated;

        MTAgent():
            name(""), task(MTTaskEnum::UNDEFINED), localPoseStamped(geometry_msgs::PoseStamped()), systemPose(Odom()), taskAllocated(false)
        {}

        MTAgent(const std::string& aName):
            name(aName), task(MTTaskEnum::UNDEFINED), localPoseStamped(geometry_msgs::PoseStamped()), systemPose(Odom()), taskAllocated(false)
        {}

        MTAgent(const MTAgent& other):
            name(other.name), task(other.task), localPoseStamped(other.localPoseStamped), systemPose(other.systemPose), taskAllocated(other.taskAllocated)
        {}
        
        MTAgent(const mt_msgs::mtAgent& aAgent):
            name(aAgent.name), task(static_cast<MTTaskEnum>(aAgent.task)), localPoseStamped(aAgent.localPose), systemPose(Odom(aAgent.systemPose,aAgent.systemTwist)), taskAllocated(aAgent.taskAllocated.data)
        {}
    };

}   // Entity

}   // Common
#endif  // CONSTANTS_ENUM_H
