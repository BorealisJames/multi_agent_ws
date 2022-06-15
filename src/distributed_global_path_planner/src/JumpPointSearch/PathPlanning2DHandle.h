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
class PathPlanning2DHandle
{
public:
    PathPlanning2DHandle();

    void setParams(const PathPlanningParams& params);

    void resetMap();

    void addInflatedObstaclesPoints(const sensor_msgs::PointCloud& pointCloud);

    void addObstaclesPoints(const sensor_msgs::PointCloud& pointCloud);

    void plan(const std::vector<Eigen::Vector2d>& inputWaypoints, std::vector<Eigen::Vector2d>& outputWaypoints);

    void plan(const std::vector<Eigen::Vector2d>& inputWaypoints, const bool& useJPS, std::vector<Eigen::Vector2d>& outputWaypoints);

    bool collisionFoundBetween2Points(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, Eigen::Vector2d& pointBeforeCollision);

    void prunePath(const std::vector<Eigen::Vector2d>& oldPath, std::vector<Eigen::Vector2d>& newPath);

private:
    void correctResolutionOffset();

    void generateRobotModel(std::vector<Eigen::Vector2d>& points);

    void loadFreespace(const double mapRes,
                       const double minBoundX, const double maxBoundX,
                       const double minBoundY, const double maxBoundY);

    PathPlanningParams m_params;
    std::shared_ptr<JPS::OccMapUtil> m_mapUtilPtr;
    std::unique_ptr<JPSPlanner2D> m_plannerJPSPtr;
    std::unique_ptr<DMPlanner2D> m_plannerDMPtr;

    std::vector<Eigen::Vector2d> m_robotModel;
};

} // namespace pathplanning