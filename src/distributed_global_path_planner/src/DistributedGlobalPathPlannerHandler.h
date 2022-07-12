//
// Created by benson on 13/1/21.
//

#pragma once

#include <functional>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>

#include "Common/Common.h"

namespace DistributedGlobalPathPlanner
{

class DistributedGlobalPathPlannerHandler
{
public:
    typedef std::shared_ptr<DistributedGlobalPathPlannerHandler> Ptr;

    DistributedGlobalPathPlannerHandler()
    : m_nh("~")
    {
        m_processedPointCloud = m_nh.advertise<sensor_msgs::PointCloud>("processed_point_cloud_cp", 1, true);
        m_ownPlannedPath = m_nh.advertise<visualization_msgs::Marker>("planned_path", 1, true);
        m_processedGoTherePath = m_nh.advertise<visualization_msgs::Marker>("processed_go_there_path", 1, true);

        m_ownPlannedPathMarker.header.frame_id = "/map";
        m_ownPlannedPathMarker.header.stamp = ros::Time();
        m_ownPlannedPathMarker.ns = "planned_path";
        m_ownPlannedPathMarker.id = 0;
        m_ownPlannedPathMarker.type = visualization_msgs::Marker::LINE_STRIP;
        m_ownPlannedPathMarker.action = visualization_msgs::Marker::ADD;
        m_ownPlannedPathMarker.pose.orientation.w = 1.0;
        m_ownPlannedPathMarker.scale.x = 0.05;
        m_ownPlannedPathMarker.color.a = 0.5;
        m_ownPlannedPathMarker.color.r = 0.0;
        m_ownPlannedPathMarker.color.g = 0.0;
        m_ownPlannedPathMarker.color.b = 0.0;
        m_ownPlannedPathMarker.lifetime = ros::Duration(0);

        m_processedGoTherePathMarker.header.frame_id = "/map";
        m_processedGoTherePathMarker.header.stamp = ros::Time();
        m_processedGoTherePathMarker.ns = "processed_go_there_path";
        m_processedGoTherePathMarker.id = 0;
        m_processedGoTherePathMarker.type = visualization_msgs::Marker::LINE_STRIP;
        m_processedGoTherePathMarker.action = visualization_msgs::Marker::ADD;
        m_processedGoTherePathMarker.pose.orientation.w = 1.0;
        m_processedGoTherePathMarker.scale.x = 0.05;
        m_processedGoTherePathMarker.color.a = 0.2;
        m_processedGoTherePathMarker.color.r = 1.0;
        m_processedGoTherePathMarker.color.g = 1.0;
        m_processedGoTherePathMarker.color.b = 1.0;
        m_processedGoTherePathMarker.lifetime = ros::Duration(0);
    }

    std::function<bool(int32_t& numberOfAgentsInTeam)> m_getNumberOfAgentsInTeam;
    std::function<bool(int32_t& ownAgentID)> m_getOwnAgentID;
    std::function<bool(std::vector<Common::Pose>& goTherePath)> m_getGoTherePath;
    std::function<void()>  m_clearPhasesAndTimeRecordOfAgentsBuffer;

    std::function<bool(std::unordered_map<int32_t, Common::PhaseAndTime>& phasesAndTimeRecordOfAgents)>  m_getPhasesAndTimeRecordOfAgents;
    std::function<bool(const int32_t, const Common::PhaseAndTime& ownAgentPhaseAndTime)>  m_pubOwnPhaseAndTime;

    std::function<void()>  m_clearAgentsPoseBuffer;
    std::function<bool(const int32_t ownAgentID, const Common::Pose& ownAgentPose)>  m_pubOwnPoseFunc;
    std::function<bool(Common::Pose& ownAgentPose)>  m_getOwnAgentPose;
    std::function<bool(std::unordered_map<int32_t, Common::Pose>& agentsPose)>  m_getAgentsPose;

    std::function<void()>m_clearAgentsPathAndWaypointProgressBuffer;
    std::function<bool(std::unordered_map<int32_t, Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgress)> m_getAgentsPathAndWaypointProgress;
    std::function<bool(const int32_t, const Common::PathAndWaypointProgress& goTherePathAndWaypointProgress)> m_pubOwnPathAndWaypointProgress;

    std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentLidarPointCloud;
    std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentCameraPointCloud;

    std::function<void()>  m_clearAgentsPlannedPathBuffer;
    std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath)> m_getAgentsPlannedPath;
    std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownPlannedPath)> m_pubOwnPlannedPath;

    std::function<void()>  m_clearAgentsProcessedPathOfAgentsBuffer;
    std::function<bool(std::unordered_map<int32_t, std::unordered_map<int32_t, Common::PathAndCost>>& agentsProcessedPathOfAgents)> m_getAgentsProcessedPathOfAgents;
    std::function<bool(const int32_t, const std::unordered_map<int32_t, Common::PathAndCost>& ownProcessedPathOfAgents)> m_pubOwnProcessedPathOfAgents;

    std::function<void()> m_clearAgentsBestProcessedPathBuffer;
    std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath)> m_getAgentsBestProcessedPath;
    std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownBestProcessedPath)> m_pubOwnBestProcessedPath;

    std::function<bool(const int32_t, const std::vector<Common::Pose>& processedGoTherePath)>  m_pubProcessedGoTherePath;

    //Viz
    void PubProcessedPointCloudViz (const sensor_msgs::PointCloud& pointCloud)
    {
        m_processedPointCloud.publish(pointCloud);
    }

    void PubOwnPlannedPathViz (const int agentId, const std::vector<Eigen::Vector3d>& ownPlannedPath)
    {
        if (ownPlannedPath.size()>=2)
        {
            m_ownPlannedPathMarker.id = agentId;

            if (agentId == 0)
            {
                m_ownPlannedPathMarker.color.r = 1.0;
            }
            else if (agentId == 1)
            {
                m_ownPlannedPathMarker.color.g = 1.0;
            }
            else
            {
                m_ownPlannedPathMarker.color.b = 1.0;
            }

            m_ownPlannedPathMarker.points.clear();
            for(auto&& plannedPathPts : ownPlannedPath)
            {
                geometry_msgs::Point p;
                p.x = plannedPathPts.x();
                p.y = plannedPathPts.y();
                p.z = plannedPathPts.z();

                m_ownPlannedPathMarker.points.push_back(p);
            }

            m_ownPlannedPath.publish(m_ownPlannedPathMarker);
        }
    }

    void PubProcessedGoTherePathViz (const int agentId, const std::vector<Eigen::Vector3d>& processedGoTherePath)
    {
        if (processedGoTherePath.size()>=2)
        {
            m_processedGoTherePathMarker.id = agentId;

            m_processedGoTherePathMarker.points.clear();
            for(auto&& goTherePathPts : processedGoTherePath)
            {
                geometry_msgs::Point p;
                p.x = goTherePathPts.x();
                p.y = goTherePathPts.y();
                p.z = goTherePathPts.z();

                m_processedGoTherePathMarker.points.push_back(p);
            }

            m_processedGoTherePath.publish(m_processedGoTherePathMarker);
        }
    }



private:
    ros::NodeHandle m_nh;
    ros::Publisher m_processedPointCloud;
    ros::Publisher m_ownPlannedPath;
    ros::Publisher m_processedGoTherePath;

    visualization_msgs::Marker m_ownPlannedPathMarker;
    visualization_msgs::Marker m_processedGoTherePathMarker;
};

}  // namespace DistributedFormation
