#pragma once

#include <chrono>
#include <Eigen/Dense>
#include <jps_basis/data_utils.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_collision/map_util.h>
#include <math.h>
#include <sensor_msgs/PointCloud.h>

#include "PathPlanningParams.h"

namespace pathplanning
{
class PathPlanning3DHandle
{
public:
    PathPlanning3DHandle();

    void setParams(const PathPlanningParams& params);

    void resetMap();

    void addInflatedObstaclesPoints(const sensor_msgs::PointCloud& pointCloud);

    void addObstaclesPoints(const sensor_msgs::PointCloud& pointCloud);

    void plan(const std::vector<Eigen::Vector3d>& inputWaypoints, std::vector<Eigen::Vector3d>& outputWaypoints);

    void plan(const std::vector<Eigen::Vector3d>& inputWaypoints, const bool& useJPS, std::vector<Eigen::Vector3d>& outputWaypoints);

    bool collisionFoundBetween2Points(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, Eigen::Vector3d& pointBeforeCollision);

    void prunePath(const std::vector<Eigen::Vector3d>& oldPath, std::vector<Eigen::Vector3d>& newPath);

private:
    void correctResolutionOffset();

    void generateRobotModel(std::vector<Eigen::Vector3d>& points);

    void loadFreespace(const double mapRes,
                       const double minBoundX, const double maxBoundX,
                       const double minBoundY, const double maxBoundY,
                       const double minBoundZ, const double maxBoundZ);

    PathPlanningParams m_params;
    std::shared_ptr<JPS::VoxelMapUtil> m_mapUtilPtr;
    std::unique_ptr<JPSPlanner3D> m_plannerJPSPtr;
    std::unique_ptr<DMPlanner3D> m_plannerDMPtr;

    std::vector<Eigen::Vector3d> m_robotModel;
};

} // namespace pathplanning