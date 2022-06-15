//
// Created by benson on 13/1/21.
//

#pragma once

#include <decomp_ros_utils/data_ros_utils.h>
#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>

#include "Common/Common.h"

namespace DistributedFormation
{

class DistributedMultiRobotFormationHandler
{
public:
    typedef std::shared_ptr<DistributedMultiRobotFormationHandler> Ptr;

    DistributedMultiRobotFormationHandler()
    : m_nh("~")
    {
        m_polyPubUAV = m_nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array_uav", 1, true);
        m_processedPointCloud = m_nh.advertise<sensor_msgs::PointCloud>("processed_point_cloud", 1, true);

        m_startPosition = m_nh.advertise<visualization_msgs::Marker>("start_position", 1, true);
        m_goalPosition = m_nh.advertise<visualization_msgs::Marker>("goal_position", 1, true);
        m_goalAlongPosesPosition = m_nh.advertise<visualization_msgs::Marker>("goal_along_poses_position", 1, true);
    }

    std::function<bool(int32_t& numberOfAgentsInTeam)> m_getNumberOfAgentsInTeam;
    std::function<bool(int32_t& ownAgentID)> m_getOwnAgentID;

    std::function<bool(std::vector<Common::Pose>& posesForFormationToTrack)> m_getPosesForFormationToTrack_mrf;
    std::function<bool(std::unordered_map<int32_t, Common::PhaseAndTime>& phasesAndTimeRecordOfAgents)>  m_getPhasesAndTimeRecordOfAgents;
    std::function<bool(const int32_t, const Common::PhaseAndTime& ownAgentPhaseAndTime)>  m_pubOwnPhaseAndTime;

    std::function<void()>  m_clearAgentsPoseBuffer;
    std::function<bool(const int32_t ownAgentID, const Common::Pose& ownAgentPose)>  m_pubOwnPoseFunc;
    std::function<bool(Common::Pose& ownAgentPose)>  m_getOwnAgentPose;
    std::function<bool(std::unordered_map<int32_t, Common::Pose>& agentsPose)>  m_getAgentsPose;
    std::function<bool(Common::Pose& humanPose)>  m_getHumanPose;

    std::function<void()>  m_clearAgentsDirectionUtilityBuffer;
    std::function<bool(const int32_t ownAgentID, const Common::DirectionUtility& ownDirectionUtility)>  m_pubOwnDirectionUtility;
    std::function<bool(std::unordered_map<int32_t, Common::DirectionUtility>& agentsDirectionUtility)>  m_getAgentsDirectionUtility;

    std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentLidarPointCloud;
    std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentCameraPointCloud;

    std::function<void()>  m_clearAgentsConvexRegion2DBuffer;
    std::function<void()>  m_clearAgentsConvexRegion3DBuffer;
    std::function<bool(const int32_t ownAgentID, const Common::ConvexRegion2D& ownConvexRegion2D)>  m_pubOwnConvex2DRegion;
    std::function<bool(const int32_t ownAgentID, const Common::ConvexRegion3D& ownConvexRegion3D)>  m_pubOwnConvex3DRegion;
    std::function<bool(std::unordered_map<int32_t, Common::ConvexRegion2D>& agentsConvexRegion2D)>  m_getAgentsConvex2DRegion;
    std::function<bool(std::unordered_map<int32_t, Common::ConvexRegion3D>& agentsConvexRegion3D)>  m_getAgentsConvex3DRegion;
    std::function<bool(const int32_t ownAgentID, const std::unordered_map<int32_t, Common::Pose>& ownTaskAssignments)>  m_pubOwnTaskAssignments;

    std::function<void()>  m_clearAgentsTaskAssignmentsBuffer;
    std::function<bool(std::unordered_map<int32_t, std::unordered_map<int32_t, Common::Pose>>& agentsTaskAssignments)>  m_getAgentsTaskAssignments;
    std::function<bool(const int32_t ownAgentID, const Common::Pose& ownAssignedPose)>  m_pubOwnAgentAssignedPose;

    //Viz
    void Pub2DPolyUAV (const vec_E<Polyhedron<2>>& polyhedron)
    {
        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedron);
        poly_msg.header.frame_id = "map";
        m_polyPubUAV.publish(poly_msg);
    }
    void Pub3DPolyUAV (const vec_E<Polyhedron<3>>& polyhedron)
    {
        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedron);
        poly_msg.header.frame_id = "map";
        m_polyPubUAV.publish(poly_msg);
    }
    void PubProcessedPointCloud (const sensor_msgs::PointCloud& pointCloud)
    {
        m_processedPointCloud.publish(pointCloud);
    }

    void PubStartPosition (const Eigen::Vector3d& startPosition)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "start";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = startPosition.x();
        marker.pose.position.y = startPosition.y();
        marker.pose.position.z = startPosition.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        m_startPosition.publish( marker );
    }
    void PubGoalPosition (const Eigen::Vector3d& goalPosition)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goalPosition.x();
        marker.pose.position.y = goalPosition.y();
        marker.pose.position.z = goalPosition.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        m_goalPosition.publish( marker );
    }
    void PubGoalAlongPosesPosition (const Eigen::Vector3d& goalAlongPosesPosition)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal_along_poses";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goalAlongPosesPosition.x();
        marker.pose.position.y = goalAlongPosesPosition.y();
        marker.pose.position.z = goalAlongPosesPosition.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        m_goalAlongPosesPosition.publish( marker );
    }

private:
    ros::NodeHandle m_nh;
    ros::Publisher m_polyPubUAV;
    ros::Publisher m_processedPointCloud;

    ros::Publisher m_startPosition;
    ros::Publisher m_goalPosition;
    ros::Publisher m_goalAlongPosesPosition;
};

}  // namespace DistributedFormation
